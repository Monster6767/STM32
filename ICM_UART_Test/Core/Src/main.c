/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* SPI、UART 最近一次通信状态 */
volatile HAL_StatusTypeDef spi_status = HAL_OK;
volatile HAL_StatusTypeDef uart_status = HAL_OK;

volatile uint32_t spi_error_code = 0;
volatile uint32_t uart_error_count = 0;

/* 传感器身份和初始化状态 */
volatile uint8_t who_am_i = 0;
volatile uint8_t imu_init_ok = 0;
volatile uint8_t imu_init_step = 0;

/* 下标 0、1、2 分别表示 X、Y、Z */
volatile int16_t accel_raw[3] = {0};
volatile int16_t gyro_raw[3] = {0};

/* 供 Watch 观察的物理量 */
volatile float accel_g[3] = {0};
volatile float gyro_dps[3] = {0};

/* 数据读取统计 */
volatile uint32_t sample_count = 0;
volatile uint32_t read_error_count = 0;
volatile uint32_t invalid_sample_count = 0;

/* 最近一组有效数据读取完成后的系统时间，单位 ms */
volatile uint32_t sample_time_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* -------- Bank 0 寄存器地址 -------- */
#define ICM_DEVICE_CONFIG  0x11U
#define ICM_ACCEL_DATA_X1  0x1FU
#define ICM_INTF_CONFIG0   0x4CU
#define ICM_INTF_CONFIG1   0x4DU
#define ICM_PWR_MGMT0      0x4EU
#define ICM_GYRO_CONFIG0   0x4FU
#define ICM_ACCEL_CONFIG0  0x50U
#define ICM_WHO_AM_I       0x75U
#define ICM_BANK_SEL       0x76U


/*
 * 发送一段已经准备好的文字。
 * length 为实际字节数，不包含字符串结束符 '\0'。
 *
 * 使用阻塞发送，发送结束或超时后才返回。
 */
static void UART_SendText(const char *text, uint16_t length)
{
    uart_status = HAL_UART_Transmit(
        &huart2,
        (uint8_t *)text,
        length,
        100
    );

    if (uart_status != HAL_OK)
    {
        ++uart_error_count;
    }
}


/*
 * 向一个寄存器写入一个字节。
 *
 * tx[0]：最高位为 0，表示写；剩余位为地址。
 * tx[1]：写入内容。
 */
static HAL_StatusTypeDef ICM_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg & 0x7FU, value};

    HAL_GPIO_WritePin(
        IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET
    );

    HAL_StatusTypeDef result =
        HAL_SPI_Transmit(&hspi1, tx, 2, 100);

    /* 无论成功或失败，都释放片选 */
    HAL_GPIO_WritePin(
        IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET
    );

    spi_status = result;
    spi_error_code = HAL_SPI_GetError(&hspi1);

    return result;
}


/*
 * 连续读取 1～12 个数据字节。
 *
 * 命令字节最高位为 1，表示读。
 * 后续发送的 0 用来产生时钟，不是写寄存器。
 *
 * 接收数组的第一个字节属于命令阶段，丢弃；
 * 后面的字节才是目标数据。
 */
static HAL_StatusTypeDef ICM_ReadRegs(
    uint8_t reg,
    uint8_t *data,
    uint8_t length)
{
    uint8_t tx[13] = {0};
    uint8_t rx[13] = {0};

    if (data == 0 || length == 0 || length > 12)
    {
        spi_status = HAL_ERROR;
        return HAL_ERROR;
    }

    tx[0] = reg | 0x80U;

    HAL_GPIO_WritePin(
        IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET
    );

    /*
     * 命令和数据在同一次事务中传输，
     * 中间 CS 始终保持低电平。
     */
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
        &hspi1, tx, rx, length + 1U, 100
    );

    HAL_GPIO_WritePin(
        IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET
    );

    spi_status = result;
    spi_error_code = HAL_SPI_GetError(&hspi1);

    if (result == HAL_OK)
    {
        for (uint8_t i = 0; i < length; ++i)
        {
            data[i] = rx[i + 1U];
        }
    }

    return result;
}


/*
 * 写入普通配置寄存器后回读检查。
 * 返回 1：读回内容一致。
 * 返回 0：通信失败或内容不一致。
 *
 * 软件复位等自动清除的控制位不使用这个函数。
 */
static int ICM_WriteCheck(uint8_t reg, uint8_t value)
{
    uint8_t check = 0;

    if (ICM_WriteReg(reg, value) != HAL_OK)
    {
        return 0;
    }

    if (ICM_ReadRegs(reg, &check, 1) != HAL_OK)
    {
        return 0;
    }

    return check == value;
}


/*
 * 只修改 mask 中为 1 的位，保留其他位。
 * 用来避免误改保留位或不相关字段。
 */
static int ICM_UpdateBits(
    uint8_t reg,
    uint8_t mask,
    uint8_t value)
{
    uint8_t old_value = 0;

    if (ICM_ReadRegs(reg, &old_value, 1) != HAL_OK)
    {
        return 0;
    }

    uint8_t new_value =
        (old_value & (uint8_t)~mask) | (value & mask);

    return ICM_WriteCheck(reg, new_value);
}


/*
 * 初始化六轴。
 *
 * imu_init_step 停留在哪个编号，就说明该步骤未通过。
 * 返回 1 表示成功，0 表示失败。
 */
static int ICM_InitSixAxis(void)
{
    uint8_t value = 0;

    /* 步骤 1：选择 Bank 0，检查身份 */
    imu_init_step = 1;

    if (!ICM_WriteCheck(ICM_BANK_SEL, 0x00))
    {
        return 0;
    }

    if (ICM_ReadRegs(ICM_WHO_AM_I, &value, 1) != HAL_OK)
    {
        return 0;
    }

    who_am_i = value;

    if (value != 0x3B)
    {
        return 0;
    }

    /* 步骤 2：软件复位 */
    imu_init_step = 2;

    if (ICM_WriteReg(ICM_DEVICE_CONFIG, 0x01) != HAL_OK)
    {
        return 0;
    }

    /* 手册要求复位后至少等待 1 ms，这里使用 10 ms */
    HAL_Delay(10);

    if (ICM_ReadRegs(ICM_WHO_AM_I, &value, 1) != HAL_OK)
    {
        return 0;
    }

    who_am_i = value;

    if (value != 0x3B)
    {
        return 0;
    }

    /*
     * 步骤 3：正常工作字段。
     * INTF_CONFIG1 的 EN_TEST_MODE[7:6] 必须设置为 01。
     */
    imu_init_step = 3;

    if (!ICM_UpdateBits(ICM_INTF_CONFIG1, 0xC0, 0x40))
    {
        return 0;
    }

    /*
     * 步骤 4：高字节在前，关闭 I2C 接口。
     * bit4 = 1：大端数据格式。
     * bit1:0 = 11：关闭 I2C，保留 SPI。
     */
    imu_init_step = 4;

    if (!ICM_UpdateBits(ICM_INTF_CONFIG0, 0x13, 0x13))
    {
        return 0;
    }

    /*
     * 步骤 5：陀螺仪 ±2000 °/s，100 Hz。
     * 0x08 = 0000 1000。
     */
    imu_init_step = 5;

    if (!ICM_WriteCheck(ICM_GYRO_CONFIG0, 0x08))
    {
        return 0;
    }

    /*
     * 步骤 6：加速度 ±4 g，100 Hz。
     * 0x68 = 0110 1000。
     */
    imu_init_step = 6;

    if (!ICM_WriteCheck(ICM_ACCEL_CONFIG0, 0x68))
    {
        return 0;
    }

    /*
     * 步骤 7：开启加速度计和陀螺仪的低噪声模式。
     * 仅设置量程和采样率，不会自动开启测量。
     */
    imu_init_step = 7;

    if (ICM_WriteReg(ICM_PWR_MGMT0, 0x0F) != HAL_OK)
    {
        return 0;
    }

    /* 开启后等待稳定，期间不写入其他寄存器 */
    HAL_Delay(100);

    if (ICM_ReadRegs(ICM_PWR_MGMT0, &value, 1) != HAL_OK)
    {
        return 0;
    }

    if (value != 0x0F)
    {
        return 0;
    }

    imu_init_step = 8;
    return 1;
}


/* 将高、低字节合成为带正负号的 16 位数据 */
static int16_t ICM_ToInt16(uint8_t high, uint8_t low)
{
    uint16_t u = ((uint16_t)high << 8) | low;

    /* 按 16 位补码解释负数 */
    int32_t value = (u & 0x8000U)
                  ? (int32_t)u - 65536
                  : (int32_t)u;

    return (int16_t)value;
}


/*
 * 读取六轴。
 *
 * 返回：
 *  1：有效数据。
 *  0：SPI 读取失败。
 * -1：包含无效样本标记。
 */
static int ICM_ReadSixAxis(void)
{
    uint8_t data[12];
    int16_t next[6];

    /*
     * 从 0x1F 开始：
     * data[0..1]   加速度 X
     * data[2..3]   加速度 Y
     * data[4..5]   加速度 Z
     * data[6..7]   角速度 X
     * data[8..9]   角速度 Y
     * data[10..11] 角速度 Z
     */
    if (ICM_ReadRegs(ICM_ACCEL_DATA_X1, data, 12) != HAL_OK)
    {
        return 0;
    }

    for (uint8_t i = 0; i < 6; ++i)
    {
        next[i] = ICM_ToInt16(
            data[2U * i], data[2U * i + 1U]
        );

        /*
         * 当前数据格式和复位后的保持策略下，
         * -32768 表示无效样本。
         */
        if (next[i] == (-32767 - 1))
        {
            return -1;
        }
    }

    for (uint8_t i = 0; i < 3; ++i)
    {
        accel_raw[i] = next[i];
        gyro_raw[i] = next[i + 3U];

        /* ±4 g：8192 原始计数对应 1 g */
        accel_g[i] = next[i] / 8192.0f;

        /* ±2000 °/s：换算为度/秒 */
        gyro_dps[i] =
            next[i + 3U] * (2000.0f / 32768.0f);
    }

    /*
     * 记录数据读取完成后的系统时间。
     * 这不是传感器内部的精确采样时间戳。
     */
    sample_time_ms = HAL_GetTick();

    return 1;
}


/*
 * 将放大了 1000 倍的整数转换为带三位小数的文字。
 *
 * 示例：
 *     305 → "0.305"
 *     -12 → "-0.012"
 *
 * 正负号单独处理，确保 -0.012 不会丢失负号。
 */
static void FormatMilli(int32_t value, char *text, size_t size)
{
    const char *sign = "";

    if (value < 0)
    {
        sign = "-";
        value = -value;
    }

    snprintf(
        text,
        size,
        "%s%ld.%03ld",
        sign,
        (long)(value / 1000),
        (long)(value % 1000)
    );
}


/* 以中文说明输出一组六轴数据 */
static void ICM_SendSixAxisUART(void)
{
    char message[384];

    char ax_text[20], ay_text[20], az_text[20];
    char gx_text[20], gy_text[20], gz_text[20];

    /* 加速度换算为 mg，适用于当前 ±4 g 量程 */
    int32_t ax = (int32_t)accel_raw[0] * 1000 / 8192;
    int32_t ay = (int32_t)accel_raw[1] * 1000 / 8192;
    int32_t az = (int32_t)accel_raw[2] * 1000 / 8192;

    /* 角速度换算为 mdps，适用于当前 ±2000 度/秒量程 */
    int32_t gx = (int32_t)gyro_raw[0] * 15625 / 256;
    int32_t gy = (int32_t)gyro_raw[1] * 15625 / 256;
    int32_t gz = (int32_t)gyro_raw[2] * 15625 / 256;

    /* 转成以 g、度/秒为单位的三位小数文本 */
    FormatMilli(ax, ax_text, sizeof(ax_text));
    FormatMilli(ay, ay_text, sizeof(ay_text));
    FormatMilli(az, az_text, sizeof(az_text));

    FormatMilli(gx, gx_text, sizeof(gx_text));
    FormatMilli(gy, gy_text, sizeof(gy_text));
    FormatMilli(gz, gz_text, sizeof(gz_text));

    int length = snprintf(
        message,
        sizeof(message),
        "时间：%lu ms\r\n"
        "加速度：X=%s g，Y=%s g，Z=%s g\r\n"
        "角速度：X=%s 度/秒，Y=%s 度/秒，Z=%s 度/秒\r\n"
        "------------------------------------------------\r\n",
        (unsigned long)sample_time_ms,
        ax_text, ay_text, az_text,
        gx_text, gy_text, gz_text
    );

    if (length > 0 && length < (int)sizeof(message))
    {
        UART_SendText(message, (uint16_t)length);
    }
    else
    {
        ++uart_error_count;
    }
}


/* 输出错误位置和计数，方便不用 Watch 也能排查 */
static void ICM_SendDiagnostic(const char *reason)
{
    char message[192];

    int length = snprintf(
        message,
        sizeof(message),
        "# %s,step=%u,id=0x%02X,spi=%u,"
        "spi_error=%lu,read_errors=%lu,invalid=%lu\r\n",
        reason,
        (unsigned int)imu_init_step,
        (unsigned int)who_am_i,
        (unsigned int)spi_status,
        (unsigned long)spi_error_code,
        (unsigned long)read_error_count,
        (unsigned long)invalid_sample_count
    );

    if (length > 0 && length < (int)sizeof(message))
    {
        UART_SendText(message, (uint16_t)length);
    }
    else
    {
        ++uart_error_count;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 空闲时不选中 ICM */
  HAL_GPIO_WritePin(
      IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET
  );

  HAL_Delay(100);

  /* 先发送启动消息，确认程序已执行到这里 */
  const char startup[] = "# ICM UART test starting\r\n";
  UART_SendText(startup, sizeof(startup) - 1U);

  /* 初始化传感器 */
  imu_init_ok = ICM_InitSixAxis() ? 1U : 0U;

  if (imu_init_ok == 0)
  {
      /*
      * 初始化失败后，每秒打印一次诊断。
      * 根据 step 查找初始化函数中对应步骤。
      */
      while (1)
      {
          ICM_SendDiagnostic("INIT FAILED");
          HAL_Delay(1000);
      }
  }

  const char ready[] = "# ICM initialized, WHO_AM_I=0x3B\r\n";
  UART_SendText(ready, sizeof(ready) - 1U);

  /* CSV 表头，说明各列单位 */
  const char header[] =
      "六轴数据实时显示：加速度单位 g，角速度单位 度/秒\r\n";

  UART_SendText(header, sizeof(header) - 1U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    int result = ICM_ReadSixAxis();

    if (result == 1)
    {
        ++sample_count;

        /* 本次数据有效，发送一行 CSV */
        ICM_SendSixAxisUART();
    }
    else if (result == 0)
    {
        ++read_error_count;
        ICM_SendDiagnostic("SPI READ FAILED");
    }
    else
    {
        ++invalid_sample_count;
        ICM_SendDiagnostic("INVALID SAMPLE");
    }

    /*
    * 约每秒显示 10 组。
    * 周期还包括读取和串口发送耗时，
    * 所以不是严格的 100 ms 定时采样。
    */
    HAL_Delay(500);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
