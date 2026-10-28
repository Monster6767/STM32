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
volatile uint8_t test_stage = 0;
volatile uint32_t spi_ok_count = 0;
volatile uint32_t id_bad_count = 0;
volatile uint32_t spi_error_count = 0;
/* 最近一次 SPI 操作的执行状态 */
volatile HAL_StatusTypeDef spi_status = HAL_OK;

/* 最近一次 SPI 操作对应的 HAL 错误码，正常为 0 */
volatile uint32_t spi_error_code = 0;

/* 身份值，正常应为 0x3B，调试器可能显示十进制 59 */
volatile uint8_t who_am_i = 0;

/* 初始化结果：0 表示未完成或失败，1 表示成功 */
volatile uint8_t imu_init_ok = 0;

/*
 * 初始化步骤，用来定位失败位置：
 * 1：选择 Bank 0 并读取身份
 * 2：软件复位，再次检查身份
 * 3：设置正常工作字段
 * 4：配置数据字节顺序和通信接口
 * 5：配置陀螺仪
 * 6：配置加速度计
 * 7：开启六轴，并检查电源管理寄存器
 * 8：初始化完成
 */
volatile uint8_t imu_init_step = 0;

/*
 * 原始数据：
 * 数组下标 0、1、2 分别对应 X、Y、Z。
 * int16_t 可以保存正负数，范围为 -32768～32767。
 */
volatile int16_t accel_raw[3] = {0};
volatile int16_t gyro_raw[3]  = {0};

/* 换算后的物理量 */
volatile float accel_g[3]  = {0}; /* 单位：g */
volatile float gyro_dps[3] = {0}; /* 单位：度/秒 */

/* 成功读取有效六轴数据的次数 */
volatile uint32_t sample_count = 0;

/* SPI 读取失败的次数 */
volatile uint32_t read_error_count = 0;

/* 收到无效样本标记的次数 */
volatile uint32_t invalid_sample_count = 0;

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

/* ============================================================
 * 一、Bank 0 中使用到的寄存器地址
 * ============================================================ */

#define ICM_REG_DEVICE_CONFIG   0x11U
#define ICM_REG_ACCEL_DATA_X1   0x1FU
#define ICM_REG_INTF_CONFIG0    0x4CU
#define ICM_REG_INTF_CONFIG1    0x4DU
#define ICM_REG_PWR_MGMT0       0x4EU
#define ICM_REG_GYRO_CONFIG0    0x4FU
#define ICM_REG_ACCEL_CONFIG0   0x50U
#define ICM_REG_WHO_AM_I        0x75U
#define ICM_REG_BANK_SEL        0x76U

#define ICM_EXPECTED_ID         0x3BU


/* ============================================================
 * 二、基础 SPI 寄存器读写
 * ============================================================ */

/*
 * 写入一个寄存器。
 *
 * reg：寄存器地址。
 * value：要写入的一个字节。
 *
 * 发送格式：
 * 第一个字节：最高位 0 表示写，其余 7 位为地址。
 * 第二个字节：要写入的内容。
 *
 * 返回值是 HAL 状态，不是传感器返回的数据。
 */
static HAL_StatusTypeDef ICM_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];

    /* & 0x7F 将最高位清零，构成写命令 */
    tx[0] = reg & 0x7FU;
    tx[1] = value;

    /* CS 拉低，选中 ICM */
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    /*
     * 当前 SPI 配置为 8 Bits。
     * 长度 2 表示发送两个字节，共 16 个时钟周期。
     * 100 为超时限制，单位 ms。
     */
    HAL_StatusTypeDef result =
        HAL_SPI_Transmit(&hspi1, tx, 2, 100);

    /* 无论成功或失败，都结束本次事务 */
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);

    spi_status = result;
    spi_error_code = HAL_SPI_GetError(&hspi1);

    return result;
}


/*
 * 从起始寄存器连续读取多个字节。
 *
 * reg：起始寄存器地址。
 * data：接收结果保存到的数组。
 * length：需要读取的数据字节数，本函数支持 1～12。
 *
 * 例如读取 12 个数据字节：
 * 发送 1 个命令字节 + 12 个占位字节。
 * 同时接收 13 个字节，丢弃第一个，保留后面 12 个。
 */
static HAL_StatusTypeDef ICM_ReadRegs(uint8_t reg,
                                     uint8_t *data,
                                     uint8_t length)
{
    uint8_t tx[13] = {0};
    uint8_t rx[13] = {0};

    /* 检查参数，避免超出数组范围 */
    if (data == 0 || length == 0 || length > 12)
    {
        spi_status = HAL_ERROR;
        return HAL_ERROR;
    }

    /* | 0x80 将最高位置 1，构成读取命令 */
    tx[0] = reg | 0x80U;

    /*
     * tx[1] 之后保持为 0。
     * 这些字节用于产生时钟，不是向寄存器写入 0。
     */

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_RESET);

    /*
     * 在一次调用中完成命令和数据传输。
     * CS 在整个传输期间持续保持低电平。
     */
    HAL_StatusTypeDef result =
        HAL_SPI_TransmitReceive(&hspi1,
                                tx,
                                rx,
                                length + 1U,
                                100);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                      IMU_CS_Pin,
                      GPIO_PIN_SET);

    spi_status = result;
    spi_error_code = HAL_SPI_GetError(&hspi1);

    if (result == HAL_OK)
    {
        /*
         * rx[0] 是发送命令期间收到的字节，不作为数据使用。
         * rx[1] 才是起始寄存器的数据。
         */
        for (uint8_t i = 0; i < length; ++i)
        {
            data[i] = rx[i + 1U];
        }
    }

    return result;
}


/*
 * 写入后回读，检查寄存器内容是否一致。
 *
 * 仅用于内容可以直接回读的普通配置寄存器。
 * 不用于软件复位等会自动清零的控制位。
 *
 * 返回：
 * 1：写入、读取均完成，而且结果一致。
 * 0：通信失败，或者回读内容不一致。
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
 * 只修改寄存器中指定的位，保留其余位。
 *
 * mask 中为 1 的位：允许修改。
 * mask 中为 0 的位：保留原值。
 *
 * 例如修改最高两位：
 * mask = 0xC0，即二进制 1100 0000。
 */
static int ICM_UpdateBits(uint8_t reg,
                          uint8_t mask,
                          uint8_t value)
{
    uint8_t old_value = 0;

    if (ICM_ReadRegs(reg, &old_value, 1) != HAL_OK)
    {
        return 0;
    }

    /*
     * 第一部分清除需要修改的位。
     * 第二部分取出要写入这些位的新值。
     * 最后用按位或组合。
     */
    uint8_t new_value =
        (old_value & (uint8_t)~mask) | (value & mask);

    return ICM_WriteCheck(reg, new_value);
}


/* ============================================================
 * 三、传感器初始化
 * ============================================================ */

/*
 * 初始化加速度计和陀螺仪。
 *
 * 目标配置：
 * 加速度：±4 g、100 Hz、低噪声模式。
 * 陀螺仪：±2000 °/s、100 Hz、低噪声模式。
 *
 * 返回 1 表示完成；返回 0 表示失败。
 * 失败时检查 imu_init_step，确定失败步骤。
 */
static int ICM_InitSixAxis(void)
{
    uint8_t value = 0;

    imu_init_step = 1;

    /*
     * REG_BANK_SEL 在各个 Bank 中均可访问。
     * 即使只复位 MCU、传感器没有断电，也明确切回 Bank 0。
     */
    if (!ICM_WriteCheck(ICM_REG_BANK_SEL, 0x00))
    {
        return 0;
    }

    if (ICM_ReadRegs(ICM_REG_WHO_AM_I, &value, 1) != HAL_OK)
    {
        return 0;
    }

    who_am_i = value;

    if (value != ICM_EXPECTED_ID)
    {
        return 0;
    }

    imu_init_step = 2;

    /*
     * DEVICE_CONFIG bit0 = 1：触发软件复位。
     * 该位会自行清除，因此这里不用 ICM_WriteCheck。
     */
    if (ICM_WriteReg(ICM_REG_DEVICE_CONFIG, 0x01) != HAL_OK)
    {
        return 0;
    }

    /* 手册要求复位后等待至少 1 ms，这里使用 10 ms */
    HAL_Delay(10);

    /* 复位后重新检查身份 */
    if (ICM_ReadRegs(ICM_REG_WHO_AM_I, &value, 1) != HAL_OK)
    {
        return 0;
    }

    who_am_i = value;

    if (value != ICM_EXPECTED_ID)
    {
        return 0;
    }

    imu_init_step = 3;

    /*
     * INTF_CONFIG1：
     * EN_TEST_MODE[7:6] 必须为 01，手册规定正常工作需要此值。
     *
     * mask  = 1100 0000
     * value = 0100 0000
     *
     * 其余位保持原值，尤其不要随意修改保留位。
     */
    if (!ICM_UpdateBits(ICM_REG_INTF_CONFIG1, 0xC0, 0x40))
    {
        return 0;
    }

    imu_init_step = 4;

    /*
     * INTF_CONFIG0：
     * bit4 = 1：传感器数据使用大端格式，高字节在前。
     * bit1:0 = 11：关闭 I2C 接口，保留 SPI。
     *
     * 其他位不修改。
     */
    if (!ICM_UpdateBits(ICM_REG_INTF_CONFIG0, 0x13, 0x13))
    {
        return 0;
    }

    imu_init_step = 5;

    /*
     * GYRO_CONFIG0 = 0x08，即 0000 1000。
     *
     * bit7:5 = 000：量程 ±2000 °/s。
     * bit4   = 0：保留位保持默认。
     * bit3:0 = 1000：输出数据率 100 Hz。
     */
    if (!ICM_WriteCheck(ICM_REG_GYRO_CONFIG0, 0x08))
    {
        return 0;
    }

    imu_init_step = 6;

    /*
     * ACCEL_CONFIG0 = 0x68，即 0110 1000。
     *
     * bit7:5 = 011：量程 ±4 g。
     * bit4   = 0：保留位保持默认。
     * bit3:0 = 1000：输出数据率 100 Hz。
     */
    if (!ICM_WriteCheck(ICM_REG_ACCEL_CONFIG0, 0x68))
    {
        return 0;
    }

    imu_init_step = 7;

    /*
     * PWR_MGMT0 = 0x0F，即 0000 1111。
     *
     * bit3:2 = 11：陀螺仪进入低噪声模式。
     * bit1:0 = 11：加速度计进入低噪声模式。
     *
     * 配置量程和采样率不会自动开启传感器，
     * 因此必须执行这一步。
     */
    if (ICM_WriteReg(ICM_REG_PWR_MGMT0, 0x0F) != HAL_OK)
    {
        return 0;
    }

    /*
     * 开启后等待稳定，并避开模式切换后禁止立即写寄存器的时间。
     * 此期间不执行其他寄存器写入。
     */
    HAL_Delay(100);

    /* 回读电源管理寄存器，确认配置保留 */
    if (ICM_ReadRegs(ICM_REG_PWR_MGMT0, &value, 1) != HAL_OK)
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


/* ============================================================
 * 四、数据解析和六轴读取
 * ============================================================ */

/*
 * 将两个字节组合为一个有符号 16 位数据。
 *
 * high：高 8 位。
 * low：低 8 位。
 *
 * 示例：
 * high = 0x20，low = 0x00
 * 组合结果为 0x2000，即十进制 8192。
 */
static int16_t ICM_ToInt16(uint8_t high, uint8_t low)
{
    uint16_t unsigned_value =
        ((uint16_t)high << 8) | low;

    int32_t signed_value;

    /*
     * 如果最高位为 1，按 16 位补码解释为负数。
     * 使用较宽的 int32_t 完成转换，再保存为 int16_t。
     */
    if (unsigned_value & 0x8000U)
    {
        signed_value = (int32_t)unsigned_value - 65536;
    }
    else
    {
        signed_value = unsigned_value;
    }

    return (int16_t)signed_value;
}


/*
 * 连续读取一组六轴数据。
 *
 * 返回：
 *  1：读取到有效数据。
 *  0：SPI 收发失败。
 * -1：收到无效样本标记。
 */
static int ICM_ReadSixAxis(void)
{
    uint8_t data[12];
    int16_t next[6];

    /*
     * 从 0x1F 开始连续读取 12 个字节。
     *
     * 数组位置     寄存器地址     内容
     * data[0..1]   0x1F～0x20     加速度 X
     * data[2..3]   0x21～0x22     加速度 Y
     * data[4..5]   0x23～0x24     加速度 Z
     * data[6..7]   0x25～0x26     角速度 X
     * data[8..9]   0x27～0x28     角速度 Y
     * data[10..11] 0x29～0x2A     角速度 Z
     *
     * 每个轴占两个字节，高字节在前。
     */
    if (ICM_ReadRegs(ICM_REG_ACCEL_DATA_X1, data, 12) != HAL_OK)
    {
        return 0;
    }

    for (uint8_t i = 0; i < 6; ++i)
    {
        next[i] = ICM_ToInt16(data[2U * i],
                              data[2U * i + 1U]);

        /*
         * 当前配置未启用 FSYNC 标记，且保持复位后的
         * FIFO_HOLD_LAST_DATA_EN = 0。
         * 此时 -32768 是无效样本标记。
         *
         * 如果发现无效样本，保留上一组有效输出，不覆盖。
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

        /*
         * 加速度量程设为 ±4 g：
         * 32768 / 4 = 8192 LSB/g。
         *
         * 原始值 8192 对应约 1 g。
         */
        accel_g[i] = next[i] / 8192.0f;

        /*
         * 陀螺仪量程设为 ±2000 °/s：
         * 物理量 = 原始值 × 2000 / 32768。
         *
         * 这里得到的是角速度，不是角度。
         */
        gyro_dps[i] =
            next[i + 3U] * (2000.0f / 32768.0f);
    }

    return 1;
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

/* 空闲时不选中传感器 */
HAL_GPIO_WritePin(IMU_CS_GPIO_Port,
                  IMU_CS_Pin,
                  GPIO_PIN_SET);

/* 等待模块上电稳定 */
HAL_Delay(100);

/* 初始化成功返回 1，失败返回 0 */
imu_init_ok = ICM_InitSixAxis() ? 1U : 0U;

if (imu_init_ok == 0)
{
    /*
     * 初始化失败后不进入六轴读取。
     * 调试时暂停，查看 imu_init_step 和 who_am_i。
     *
     * 注意：寄存器回读不一致时，
     * spi_status 仍可能为 HAL_OK。
     */
    while (1)
    {
        HAL_Delay(100);
    }
}

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
    int result = ICM_ReadSixAxis();

    if (result == 1)
    {
        /* 成功得到一组六轴有效数据 */
        ++sample_count;
    }
    else if (result == 0)
    {
        /* SPI 收发操作失败 */
        ++read_error_count;
    }
    else
    {
        /* 收发完成，但数据含有无效样本标记 */
        ++invalid_sample_count;
    }

    /*
     * 约每 20 ms 读取一次，用于观察数据。
     *
     * 传感器输出数据率为 100 Hz，即每 10 ms 更新一次。
     * 这里会跳过部分样本，属于基础功能测试，
     * 还不是完整的数据记录程序。
     */
    HAL_Delay(20);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */
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
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
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
