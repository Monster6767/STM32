#ifndef APP_H
#define APP_H
#include <stdint.h>

/*
 * app.h：把采集、标定、融合、输出组织成一个应用。
 * 主函数只需要调用 App_Init() 和 App_Run()。
 * 下列结构体提供给 VSCode 调试器 Watch 观察，不参与通信协议封装。
 */
typedef struct {
    uint32_t samples;               /* 累计有效采集次数，包含启动标定期间的数据。 */
    uint32_t updates;               /* 累计成功姿态更新次数，不含标定期间的数据。 */
    uint32_t spi_errors;            /* 数据就绪读取或六轴读取发生 SPI 错误的次数。 */
    uint32_t invalid_samples;       /* 六轴数据含无效标记的次数。 */
    uint32_t timing_gaps;           /* 更新间隔异常次数，不是精确丢样数。 */
    uint32_t algorithm_errors;      /* 姿态函数拒绝更新的次数。 */
    uint32_t calibration_restarts;  /* 静止条件不满足而重新累计标定的次数。 */
    uint32_t sample_hz;             /* 最近约一秒实际有效读取频率，单位 Hz。 */
    uint32_t update_hz;             /* 最近约一秒实际成功解算频率，单位 Hz。 */
    uint32_t max_fusion_us;         /* 历史最长 IMU_Update 调用耗时，单位 us。 */
    uint32_t max_loop_us;           /* 历史最长 App_Run 耗时，单位 us。 */
    uint32_t calibration_samples;   /* 当前连续静止标定窗口已收集的样本数。 */
    uint8_t state;                  /* 0=标定；1=运行；2=ICM初始化失败；3=计时器失败。 */
    uint8_t who_am_i;               /* 传感器身份值，应为 0x3B。 */
    uint8_t init_step;              /* 传感器初始化步骤，应最终到达 8。 */
    float bias_dps[3];              /* 标定得到的 XYZ 角速度零偏，单位 度/秒。 */
    float roll;                    /* 最近显示周期得到的横滚角，单位度。 */
    float pitch;                   /* 最近显示周期得到的俯仰角，单位度。 */
    float yaw;                     /* 最近显示周期得到的相对偏航角，单位度。 */
} App_Status;

/* volatile 便于观察运行中的状态变化；不意味着整个结构体可原子读取。
 * 正式程序应由应用层单独维护，不要从中断中随意改写这些字段。 */
extern volatile App_Status app_status;

/* 全部 MX_xxx_Init 执行后调用一次；内部包含传感器启动等待。 */
void App_Init(void);

/* 在 while(1) 内尽可能频繁调用，不要在循环中添加 HAL_Delay。 */
void App_Run(void);
#endif
