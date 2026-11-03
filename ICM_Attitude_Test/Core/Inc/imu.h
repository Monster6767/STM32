#ifndef IMU_H
#define IMU_H
#include <stdint.h>
/*
 * imu.h：与芯片、SPI、STM32 HAL 无关的六轴姿态算法接口。
 *
 * 输入约定：
 *   1. 加速度与角速度必须处于同一个右手机体坐标系。
 *   2. 本示例水平静止时，加速度计 +Z 轴输出约为 +1 g。
 *   3. 加速度使用 g；角速度使用 rad/s；时间间隔使用秒。
 *   4. 四元数顺序为 [w, x, y, z]，表示机体系到参考系的旋转。
 * 输出约定：采用 Z-Y-X 欧拉角顺序，角度单位为度。
 * 六轴输入不含北向参考，因此 yaw 只是相对航向，会累积漂移。
 */
typedef struct {
    float q[4];          /* 单位四元数；四个分量平方和应接近 1。 */
    float integral[3];   /* 重力方向误差的积分补偿，最终作为 rad/s 加到角速度。 */
    float kp;            /* 比例反馈增益，控制倾角误差的校正速度。 */
    float ki;            /* 积分反馈增益；不能使不可观测的绝对航向变得可观测。 */
} IMU_t;

typedef struct {
    float roll;          /* 横滚角，通常输出范围为 -180~180 度。 */
    float pitch;         /* 俯仰角，输出范围为 -90~90 度。 */
    float yaw;           /* 偏航角，通常输出范围为 -180~180 度。 */
} IMU_Angles;

/* 用静止平均加速度初始化倾角，并把初始 yaw 设为 0。
 * accel_g 必须是有效、非零且主要反映重力的三轴加速度。 */
void IMU_Init(IMU_t *s, const float accel_g[3]);

/* 每得到一组有效新样本调用一次。
 * gyro_rad：rad/s；accel_g：g；dt：秒。
 * 当前版本只接受 0 < dt <= 0.002，适配本例 1000 Hz 的调度策略。
 * 返回 1：更新成功；返回 0：时间参数或归一化结果异常。
 * 加速度不满足重力校正门限时仍使用陀螺仪更新，而不是停止积分。 */
int IMU_Update(IMU_t *s, const float gyro_rad[3],
               const float accel_g[3], float dt);

/* 将当前四元数转换为欧拉角，不修改算法状态。
 * 计算涉及三角函数，本例只在约 50 Hz 的显示任务中调用。
 * pitch 接近 +/-90 度时存在欧拉角奇异性，不是四元数失效。 */
void IMU_GetAngles(const IMU_t *s, IMU_Angles *out);
#endif
