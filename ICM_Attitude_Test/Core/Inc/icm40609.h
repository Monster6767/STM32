#ifndef ICM40609_H
#define ICM40609_H
#include "main.h"

/* ICM40609 硬件实例：把使用哪路 SPI、哪个片选引脚交给应用层决定。
 * 本驱动使用 HAL 阻塞 SPI 事务，运行时轮询数据就绪，不使用 FIFO。
 * 驱动只读取传感器，不做陀螺仪零偏标定，也不计算姿态。 */
typedef struct {
    SPI_HandleTypeDef *spi;        /* CubeMX 生成的 SPI 句柄，例如 &hspi1。 */
    GPIO_TypeDef *cs_port;         /* 片选 GPIO 端口，例如 GPIOB。 */
    uint16_t cs_pin;               /* 片选引脚位掩码，例如 GPIO_PIN_0。 */
    uint8_t who_am_i;              /* 最近读取到的身份值，本例应为 0x3B。 */
    uint8_t init_step;             /* 初始化停在哪一步；成功完成时为 8。 */
    HAL_StatusTypeDef last_status; /* 最近一次 SPI 事务状态；不代表回读值一定正确。 */
} ICM40609_t;

/*
 * 一组传感器样本。
 * 数组下标0、1、2分别表示X、Y、Z。
 */
typedef struct {
    int16_t accel_raw[3]; /* 加速度原始值 */
    int16_t gyro_raw[3];  /* 角速度原始值 */

    float accel_g[3];     /* 加速度，单位g */
    float gyro_dps[3];    /* 角速度，单位度/秒 */

    int16_t temperature_raw; /* 温度寄存器的16位有符号原始值 */
    float temperature_c;     /* 芯片内部温度，单位摄氏度 */
} ICM40609_Sample;

/* 调用前先填写 spi、cs_port、cs_pin，且相关外设必须已初始化。
 * 函数包含上电与复位等待，只能在启动阶段调用，不能逐样本调用。
 * 返回 1 表示成功；返回 0 时查看 init_step、who_am_i、last_status。 */
int ICM40609_Init(ICM40609_t *d);

/* 主循环中反复调用；只有数据就绪才读取完整六轴数据。
 * 返回  1：得到一组有效新数据，并更新 *s。
 * 返回  0：暂时没有新数据，正常情况，不计为通信错误。
 * 返回 -1：SPI 事务失败。
 * 返回 -2：数据中含当前格式定义的无效标记 -32768。
 * 非 1 返回时，不应将旧输出再次当成新样本送入姿态算法。
 * 数据就绪位不是样本计数器，本接口不能保证检出全部丢样。 */
int ICM40609_Poll(ICM40609_t *d, ICM40609_Sample *s);
#endif
