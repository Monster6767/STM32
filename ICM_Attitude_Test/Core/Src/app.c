/*
 * app.c：应用调度层，main.c 只需 App_Init() 和循环中的 App_Run()。
 *
 * 启动阶段：初始化 ICM -> 连续静止采样 -> 估计零偏 -> 初始化姿态。
 * 运行阶段：轮询新数据 -> 扣零偏 -> 度/秒转 rad/s -> 四元数融合。
 * 后台任务：每秒统计频率，每20ms转换欧拉角并尝试中断发送。
 *
 * 不使用 FIFO，本阶段无法恢复被覆盖的样本，也无法保证绝对不丢样。
 * 不能在 while(1) 里另加延时、逐样本打印或阻塞串口发送。
 */
#include "app.h"
#include "icm40609.h"
#include "imu.h"
#include "telemetry.h"
#include "spi.h"   /* CubeMX 生成，提供 hspi1。 */
#include "usart.h" /* CubeMX 生成，提供 huart2。 */
#include "tim.h"   /* CubeMX 生成，提供 htim3。 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CALIBRATION_COUNT 2000U /* 1000Hz 理想连续采集时约2秒，并非固定墙钟时长。 */
#define DEG2RAD 0.01745329252f  /* pi/180：角速度从度/秒转换为 rad/s。 */

/* 全局可观察状态。volatile 用于让编译器保留实际读取，不用于解决多线程同步。 */
volatile App_Status app_status;

/* static 把具体实例限制在本文件内，其他模块通过接口访问。 */
static ICM40609_t sensor;
static IMU_t attitude;
static ICM40609_Sample sample;

/* 标定累计：角速度和、角速度平方和、加速度和；下标均为 XYZ。 */
static float sum_g[3];
static float sum_g2[3];
static float sum_a[3];
static float bias[3];             /* 通过检查后使用的零偏，单位度/秒。 */
static uint32_t cal_count;        /* 当前标定窗口样本数。 */

/* TIM3 为16位1MHz计数器；通过频繁读取累计成32位微秒时间。 */
static uint16_t clock_previous;
static uint32_t clock_us;
static uint32_t previous_sample_us;

/* HAL毫秒时间用于低频任务，不用于精细测量1ms样本周期。 */
static uint32_t previous_report_ms;
static uint32_t previous_text_ms;
static uint32_t previous_samples;
static uint32_t previous_updates;
static uint8_t have_previous;     /* 第一组运行数据仅建立时间基准，不做积分。 */

static uint32_t micros_now(void)
{
    /* 前提：TIM3输入64MHz、PSC=63、ARR=65535，已用 Base_Start 启动。
     * 必须在65.536ms内至少调用一次，跨过多圈就无法仅凭16位值补回时间。
     * 例如上次65530、本次10，转为uint16_t后的差值是16，能处理一次回绕。
     * 这里只由主循环调用；不要在其他中断里并发调用本函数。 */
    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    clock_us += (uint16_t)(now - clock_previous);
    clock_previous = now;
    return clock_us;
}

static void reset_calibration(void)
{
    /* 检测到运动或窗口方差过大，放弃整段数据，从零重新累计。
     * 不把含运动的数据混进静止零偏，否则会把真实角速度当成传感器误差。 */
    cal_count = 0;
    memset(sum_g, 0, sizeof(sum_g));
    memset(sum_g2, 0, sizeof(sum_g2));
    memset(sum_a, 0, sizeof(sum_a));
    app_status.calibration_samples = 0;
    ++app_status.calibration_restarts;
}

static void calibrate(const ICM40609_Sample *s)
{
    float accel_norm_squared = 0.0f;
    float gyro_norm_squared = 0.0f;
    for (unsigned i = 0; i < 3; ++i) {
        accel_norm_squared += s->accel_g[i] * s->accel_g[i];
        gyro_norm_squared += s->gyro_dps[i] * s->gyro_dps[i];
    }

    /* 简单静止门限：加速度模0.9~1.1g，角速度模不大于5度/秒。
     * 只能拒绝明显运动；慢速匀速旋转仍可能通过，操作者必须保持板子静止。
     * 不要求三轴加速度都为零：静止时重力对应的输出本来就约为1g。 */
    if (accel_norm_squared < 0.81f || accel_norm_squared > 1.21f ||
        gyro_norm_squared > 25.0f) {
        reset_calibration();
        return;
    }

    for (unsigned i = 0; i < 3; ++i) {
        sum_g[i] += s->gyro_dps[i];
        sum_g2[i] += s->gyro_dps[i] * s->gyro_dps[i];
        sum_a[i] += s->accel_g[i];
    }
    ++cal_count;
    app_status.calibration_samples = cal_count;
    if (cal_count < CALIBRATION_COUNT) {
        return;
    }

    float average_accel[3];
    for (unsigned i = 0; i < 3; ++i) {
        /* 静止时真实转速近零，因此角速度平均值近似为传感器零偏。 */
        bias[i] = sum_g[i] / (float)cal_count;

        /* 方差=平方均值-均值平方，检查整段数据是否足够稳定。
         * 0.25的单位是(度/秒)^2，对应标准差0.5度/秒。
         * 浮点相减可能产生很小的负数；这里只检查上限，不对它开方。 */
        float variance = sum_g2[i] / (float)cal_count - bias[i] * bias[i];
        if (variance > 0.25f) {
            reset_calibration();
            return;
        }
        average_accel[i] = sum_a[i] / (float)cal_count;
    }

    /* 所有轴检查通过后才公开零偏，并初始化姿态。
     * 不把平均加速度直接作为零偏扣除，否则会错误地把重力也减掉。 */
    for (unsigned i = 0; i < 3; ++i) {
        app_status.bias_dps[i] = bias[i];
    }
    IMU_Init(&attitude, average_accel);
    have_previous = 0;
    app_status.state = 1;
}

void App_Init(void)
{
    /* 在全部 MX_xxx_Init 之后调用一次，不支持运行中反复重入初始化。 */
    memset((void *)&app_status, 0, sizeof(app_status));
    Telemetry_Init(&huart2);

    /* 把硬件绑定给驱动。更换SPI或片选时优先改这里和CubeMX配置。 */
    sensor.spi = &hspi1;
    sensor.cs_port = IMU_CS_GPIO_Port;
    sensor.cs_pin = IMU_CS_Pin;
    if (!ICM40609_Init(&sensor)) {
        app_status.state = 2;
    }
    app_status.who_am_i = sensor.who_am_i;
    app_status.init_step = sensor.init_step;

    /* 必须在传感器启动延时结束后建立微秒时基，避免延时跨越多次16位回绕。
     * Base_Start 不是 Base_Start_IT，本例不需要TIM3中断。 */
    if (HAL_TIM_Base_Start(&htim3) != HAL_OK) {
        app_status.state = 3;
    }
    clock_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    clock_us = 0;
    previous_report_ms = HAL_GetTick();
    previous_text_ms = previous_report_ms;
}

static void send_status(uint32_t now_ms)
{
    /* 正常姿态输出最多每20ms一次，即50Hz。
     * UART忙时直接跳过，不在这里等待，不影响下一轮采集。
     * 无符号相减可兼容HAL毫秒计数器的正常回绕。 */
    if ((uint32_t)(now_ms - previous_text_ms) < 20U || Telemetry_IsBusy()) {
        return;
    }
    previous_text_ms = now_ms;

    /* 临时格式化数组在函数返回后失效没关系：Telemetry_Send会复制内容。
     * 它不能直接作为HAL_UART_Transmit_IT的长期发送缓冲区。 */
    char text[240];
    int length;
    if (app_status.state == 1) {
        IMU_Angles angles;

        /* 三角函数只在显示频率执行，内部四元数仍逐样本更新。
         * 因此Watch里的欧拉角约50Hz变化，不代表姿态融合只有50Hz。 */
        IMU_GetAngles(&attitude, &angles);
        app_status.roll = angles.roll;
        app_status.pitch = angles.pitch;
        app_status.yaw = angles.yaw;

        /* 避免printf浮点格式化：角度乘1000并四舍五入，输出整数毫度。
         * ATT字段：roll_mdeg,pitch_mdeg,yaw_mdeg,采集Hz,解算Hz,gap数,最大融合us。
         * 对应%ld/%lu显式转换long，避免不同C平台整数类型差异导致格式错误。 */
        length = snprintf(text, sizeof(text),
            "ATT,%ld,%ld,%ld,%lu,%lu,%lu,%lu\r\n",
            (long)lroundf(angles.roll * 1000.0f),
            (long)lroundf(angles.pitch * 1000.0f),
            (long)lroundf(angles.yaw * 1000.0f),
            (unsigned long)app_status.sample_hz,
            (unsigned long)app_status.update_hz,
            (unsigned long)app_status.timing_gaps,
            (unsigned long)app_status.max_fusion_us);
    } else {
        /* 标定/失败时输出诊断，降到每200ms一次，避免无意义刷屏。
         * 函数内static变量跨调用保留，但只属于本函数。 */
        static uint32_t last_note_ms;
        if ((uint32_t)(now_ms - last_note_ms) < 200U) {
            return;
        }
        last_note_ms = now_ms;
        length = snprintf(text, sizeof(text),
            "STATE,%u,CAL,%lu,STEP,%u,ID,%02X\r\n",
            (unsigned)app_status.state,
            (unsigned long)app_status.calibration_samples,
            (unsigned)app_status.init_step,
            (unsigned)app_status.who_am_i);
    }

    /* snprintf返回“本来需要的字符数”，不含末尾0。
     * >=数组大小说明内容被截断，不发送这类不完整帧。 */
    if (length > 0 && length < (int)sizeof(text)) {
        (void)Telemetry_Send(text, (uint16_t)length);
    }
}

void App_Run(void)
{
    uint32_t loop_start_us = micros_now();

    /* 只有标定和正常运行状态才访问传感器；初始化失败时继续输出诊断。 */
    if (app_status.state <= 1) {
        int result = ICM40609_Poll(&sensor, &sample);
        if (result == -1) {
            ++app_status.spi_errors;
        }
        if (result == -2) {
            ++app_status.invalid_samples;
        }

        /* result=0只是没有新数据，不更新姿态，也不计为错误。 */
        if (result == 1) {
            uint32_t sample_us = micros_now();
            ++app_status.samples;
            if (app_status.state == 0) {
                calibrate(&sample);
            } else {
                /* 读取完成时间差近似采样间隔，不是传感器硬件时间戳。
                 * 轮询抖动和CPU调度会影响它，本阶段接受这一限制。 */
                uint32_t elapsed_us = sample_us - previous_sample_us;
                previous_sample_us = sample_us;

                if (!have_previous) {
                    /* 标定后首个样本只建立起点，避免使用无效的前一时刻。 */
                    have_previous = 1;
                } else if (elapsed_us < 200U || elapsed_us > 2000U) {
                    /* 间隔异常：跳过本次积分，下次从当前时刻重新计时。
                     * 不能用最后一个角速度推测丢失期间的完整运动。
                     * gap是异常间隔次数，既不是准确丢样数，也不保证检出所有漏样。 */
                    ++app_status.timing_gaps;
                } else {
                    float gyro_rad[3];
                    for (unsigned i = 0; i < 3; ++i) {
                        /* 顺序：先按度/秒扣零偏，再乘pi/180转rad/s。
                         * 如果安装方向需要轴映射，应同时处理加速度与角速度，
                         * 并确保标定和运行遵守同一套右手坐标约定。 */
                        gyro_rad[i] = (sample.gyro_dps[i] - bias[i]) * DEG2RAD;
                    }

                    /* 计时仅围绕融合函数，不含SPI和欧拉角/文本格式化。 */
                    uint32_t fusion_start_us = micros_now();
                    if (IMU_Update(&attitude, gyro_rad, sample.accel_g,
                                   (float)elapsed_us * 1.0e-6f)) {
                        ++app_status.updates;
                    } else {
                        ++app_status.algorithm_errors;
                    }
                    uint32_t fusion_cost_us = micros_now() - fusion_start_us;
                    if (fusion_cost_us > app_status.max_fusion_us) {
                        app_status.max_fusion_us = fusion_cost_us;
                    }
                }
            }
        }
    }

    /* 低频统计使用毫秒时基，每约一秒统计一次真实平均频率。
     * 不能把传感器配置的1000Hz直接打印成“实际采样频率”。 */
    uint32_t now_ms = HAL_GetTick();
    uint32_t interval_ms = now_ms - previous_report_ms;
    if (interval_ms >= 1000U) {
        app_status.sample_hz = (app_status.samples - previous_samples) *
                               1000U / interval_ms;
        app_status.update_hz = (app_status.updates - previous_updates) *
                               1000U / interval_ms;
        previous_samples = app_status.samples;
        previous_updates = app_status.updates;
        previous_report_ms = now_ms;
    }

    send_status(now_ms);

    /* 整轮耗时包括本轮SPI、融合、欧拉角和格式化，以及期间发生的中断。
     * 历史最大值有助于发现低频输出任务偶尔拖慢循环的问题。
     * 若卡住超过65ms，16位计时扩展可能丢圈，不能信任这个耗时值。 */
    uint32_t loop_cost_us = micros_now() - loop_start_us;
    if (loop_cost_us > app_status.max_loop_us) {
        app_status.max_loop_us = loop_cost_us;
    }
}
