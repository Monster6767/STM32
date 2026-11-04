/*
 * icm40609.c：ICM40609 六轴传感器驱动。
 *
 * 分层职责：
 *   spi.c 由 CubeMX 生成，决定 MCU 的 SPI 时钟、引脚和模式。
 *   本文件决定 ICM 寄存器、CS 时序、字节序和物理量转换。
 *   零偏标定在 app.c，姿态融合在 imu.c，串口在 telemetry.c。
 *
 * 当前固定配置：加速度 +/-4 g；角速度 +/-2000 度/秒；二者均 1000 Hz。
 * FIFO 关闭，内部数字滤波保留复位设置，不能据此声称已设置某个带宽。
 */
#include "icm40609.h"

/* 本例使用 Bank 0 的寄存器。用名称代替地址，便于对照数据手册。 */
#define REG_DEVICE_CONFIG   0x11U /* bit0：软件复位，触发后自行清除。 */
#define REG_ACCEL_DATA_X1   0x1FU /* 连续六轴数据首地址，高字节在前。 */
#define REG_INT_STATUS      0x2DU /* bit3：新数据就绪，读取具有清除语义。 */
#define REG_INTF_CONFIG0    0x4CU /* 数据字节序、串行接口等配置。 */
#define REG_INTF_CONFIG1    0x4DU /* 正常工作测试字段及内部时钟选择。 */
#define REG_PWR_MGMT0       0x4EU /* 加速度与陀螺仪电源模式。 */
#define REG_GYRO_CONFIG0    0x4FU /* 角速度量程与输出数据率。 */
#define REG_ACCEL_CONFIG0   0x50U /* 加速度量程与输出数据率。 */
#define REG_WHO_AM_I        0x75U /* 身份值，本型号应读到 0x3B。 */
#define REG_BANK_SEL        0x76U /* 寄存器 Bank 选择。 */
/* 温度高字节地址，后面依次是温度低字节和六轴数据。 */
#define REG_TEMP_DATA1 0x1DU

/* 单次事务超时，不是采样周期。正常 2 MHz SPI 事务只需很短时间。
 * 发生超时仍可能丢样，但不采用原先 100 ms 的长等待。 */
#define SPI_TIMEOUT_MS 2U

static HAL_StatusTypeDef read_regs(ICM40609_t *d, uint8_t reg,
                                    uint8_t *data, uint8_t count)
{
    /* 最多 1 个命令字节 + 12 个数据字节，不是 FIFO 批量读接口。 */
    /*
    * 1字节读命令 + 最多14字节数据。
    * 原来的13字节数组只能支持12字节数据，必须一起扩容。
    */
    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};

    if (data == NULL || count == 0 || count > 14) {
        return HAL_ERROR;
    }

    /* SPI 寄存器地址最高位为读写位：1=读，0=写。
     * tx 后续的零仅用于产生时钟，并不是写入寄存器的数据。 */
    tx[0] = reg | 0x80U;
    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET);

    /* 在同一事务中发命令并接收数据，CS 中间不拉高。
     * 阻塞 SPI 在返回前完成，因此使用局部 tx/rx 数组是安全的。 */
    d->last_status = HAL_SPI_TransmitReceive(d->spi, tx, rx,
                                            count + 1U, SPI_TIMEOUT_MS);

    /* 无论成功还是失败都释放片选，不能让器件一直处于选中状态。 */
    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);
    if (d->last_status == HAL_OK) {
        for (uint8_t i = 0; i < count; ++i) {
            /* rx[0] 属于命令阶段，丢弃；rx[1] 才是第一个寄存器值。 */
            data[i] = rx[i + 1U];
        }
    }
    return d->last_status;
}

static int write_reg(ICM40609_t *d, uint8_t reg, uint8_t value)
{
    /* 地址最高位清零表示写；第二个字节是目标寄存器的新值。 */
    uint8_t tx[2] = {reg & 0x7FU, value};
    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_RESET);
    d->last_status = HAL_SPI_Transmit(d->spi, tx, 2, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);
    return d->last_status == HAL_OK;
}

static int write_check(ICM40609_t *d, uint8_t reg, uint8_t value)
{
    /* 发送成功只能说明 HAL 完成了事务，不代表芯片接受了预期配置。
     * 普通寄存器写后回读验证；软件复位等自清零位不能用本函数验证。 */
    uint8_t readback = 0;
    if (!write_reg(d, reg, value)) {
        return 0;
    }
    if (read_regs(d, reg, &readback, 1) != HAL_OK) {
        return 0;
    }
    return readback == value;
}

static int update_bits(ICM40609_t *d, uint8_t reg,
                       uint8_t mask, uint8_t value)
{
    /* 读-改-写，只修改 mask 标出的位，保留不相关位和保留位。
     * 例如 mask=0xC0 表示只允许修改最高两个比特。 */
    uint8_t old_value = 0;
    if (read_regs(d, reg, &old_value, 1) != HAL_OK) {
        return 0;
    }
    uint8_t next_value = (old_value & (uint8_t)~mask) | (value & mask);
    return write_check(d, reg, next_value);
}

int ICM40609_Init(ICM40609_t *d)
{
    uint8_t value = 0;

    /* 先释放 CS，再给模块上电稳定时间。这些延时只在启动阶段执行。 */
    HAL_GPIO_WritePin(d->cs_port, d->cs_pin, GPIO_PIN_SET);
    HAL_Delay(100);

    /* 步骤1：显式切回 Bank0，兼容 MCU 复位但传感器未断电的情况。 */
    d->init_step = 1;
    if (!write_check(d, REG_BANK_SEL, 0x00)) {
        return 0;
    }
    if (read_regs(d, REG_WHO_AM_I, &value, 1) != HAL_OK) {
        return 0;
    }
    d->who_am_i = value;
    if (value != 0x3B) {
        return 0;
    }

    /* 步骤2：复位恢复已知寄存器状态，再检查身份。
     * 复位位会自行清除，不能要求回读仍为 0x01。 */
    d->init_step = 2;
    if (!write_reg(d, REG_DEVICE_CONFIG, 0x01)) {
        return 0;
    }
    HAL_Delay(10);
    if (!write_check(d, REG_BANK_SEL, 0x00)) {
        return 0;
    }
    if (read_regs(d, REG_WHO_AM_I, &value, 1) != HAL_OK) {
        return 0;
    }
    d->who_am_i = value;
    if (value != 0x3B) {
        return 0;
    }

    /* 步骤3：EN_TEST_MODE[7:6] 必须为 01 才是本器件的正常工作设置。
     * 不是清掉所有位，其余配置尤其保留位继续保留。 */
    d->init_step = 3;
    if (!update_bits(d, REG_INTF_CONFIG1, 0xC0, 0x40)) {
        return 0;
    }

    /* 步骤4：bit4=1，数据高字节在前；bit1:0=11，关闭 I2C 保留 SPI。
     * bit7 保留复位状态，因此按本例格式识别 -32768 无效数据标记。 */
    d->init_step = 4;
    if (!update_bits(d, REG_INTF_CONFIG0, 0x13, 0x13)) {
        return 0;
    }

    /* 步骤5：0x06=0000 0110。
     * [7:5]=000 -> +/-2000 度/秒；[3:0]=0110 -> 1000 Hz。
     * 低四位是编码，不是把二进制数字直接当作 Hz。 */
    d->init_step = 5;
    if (!write_check(d, REG_GYRO_CONFIG0, 0x06)) {
        return 0;
    }

    /* 步骤6：0x66=0110 0110。
     * [7:5]=011 -> +/-4 g；[3:0]=0110 -> 1000 Hz。 */
    d->init_step = 6;
    if (!write_check(d, REG_ACCEL_CONFIG0, 0x66)) {
        return 0;
    }

    /* 步骤7：低四位 1111，使陀螺仪、加速度计都进入低噪声模式。
     * 仅设置 ODR 和量程并不会自动开启测量。 */
    d->init_step = 7;
    if (!write_reg(d, REG_PWR_MGMT0, 0x0F)) {
        return 0;
    }

    /* 等待开启后稳定，并避开模式切换后禁止立即写寄存器的时间。
     * 等待期间不做其他配置写入，之后回读确认模式未丢失。 */
    HAL_Delay(100);
    if (read_regs(d, REG_PWR_MGMT0, &value, 1) != HAL_OK || value != 0x0F) {
        return 0;
    }
    d->init_step = 8;
    return 1;
}

static int16_t to_i16(uint8_t high, uint8_t low)
{
    /* 先组合成无符号 16 位，再按补码显式解释负数。
     * 例如 FF FF -> 65535 -> -1，而不是误当成很大的正数。 */
    uint16_t value = ((uint16_t)high << 8) | low;
    return (int16_t)((value & 0x8000U) ?
                     (int32_t)value - 65536 : (int32_t)value);
}

int ICM40609_Poll(ICM40609_t *d, ICM40609_Sample *s)
{
    uint8_t status = 0;
    uint8_t data[14];
    int16_t values[6];

    /* 先检查是否有新六轴数据。 */
    if (read_regs(d, REG_INT_STATUS, &status, 1) != HAL_OK) {
        return -1;
    }

    if ((status & 0x08U) == 0U) {
        return 0;
    }

    /*
     * 从0x1D开始连续读取14字节：
     *
     * data[0..1]   温度
     * data[2..3]   加速度X
     * data[4..5]   加速度Y
     * data[6..7]   加速度Z
     * data[8..9]   角速度X
     * data[10..11] 角速度Y
     * data[12..13] 角速度Z
     *
     * 当前传感器配置为高字节在前。
     */
    if (read_regs(d, REG_TEMP_DATA1, data, 14) != HAL_OK) {
        return -1;
    }

    /* 六轴数据现在从data[2]开始，而不是原来的data[0]。 */
    for (unsigned i = 0; i < 6; ++i) {
        unsigned offset = 2U + i * 2U;

        values[i] = to_i16(data[offset], data[offset + 1U]);

        /*
         * 按当前六轴无效值配置检查。
         * 不直接把该规则套用到温度字段。
         */
        if (values[i] == INT16_MIN) {
            return -2;
        }
    }

    /* 全部六轴有效后，再更新调用方的样本。 */
    for (unsigned i = 0; i < 3; ++i) {
        s->accel_raw[i] = values[i];
        s->gyro_raw[i] = values[i + 3U];

        /* 当前加速度量程为±4g。 */
        s->accel_g[i] = values[i] / 8192.0f;

        /* 当前角速度量程为±2000度/秒。 */
        s->gyro_dps[i] =
            values[i + 3U] * (2000.0f / 32768.0f);
    }

    /*
     * 温度原始值是16位有符号数。
     * 使用传感器寄存器温度公式，不使用FIFO的8位温度公式。
     */
    s->temperature_raw = to_i16(data[0], data[1]);

    s->temperature_c =
        (float)s->temperature_raw / 132.48f + 25.0f;

    return 1;
}
