/*
 * imu.c：Mahony 思路的六轴姿态融合，与 HAL 和传感器型号无关。
 *
 * 核心过程：角速度预测姿态变化；加速度主要反映重力时校正倾斜。
 * 四元数保存三维姿态，避免对三个欧拉角分别积分造成旋转耦合错误。
 *
 * 加速度计测量的是比力：静止水平且 +Z 朝上时约为 [0,0,+1] g。
 * 本文的“重力参考”指算法采用的静止加速度参考，不应把输入随意反号。
 * 加速度不提供北向信息，所以六轴反馈不能消除长期航向漂移。
 */
#include "imu.h"
#include <math.h>
#include <string.h>

/* 数学库用弧度；显示用角度，转换系数为 180/pi。 */
#define RAD2DEG 57.2957795131f

/* 限幅：既限制积分反馈，也防止 asin 的输入因舍入略超出 [-1,1]。 */
static float clampf(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

void IMU_Init(IMU_t *s, const float accel_g[3])
{
    /* 调用方提供有效对象、非零静止平均加速度；清除此前的积分历史。 */
    memset(s, 0, sizeof(*s));

    /* 起步调试参数，不是已在此板上优化的参数。
     * 积分公式显式包含 dt，不沿用原 MPU6050 代码的每步积分系数。 */
    s->kp = 2.0f;
    s->ki = 0.02f;

    /* 用重力方向初始化实际倾角，不假设启动时板子一定水平。
     * atan2f 保留象限信息，输入比例在求角度时抵消，无需先归一化。 */
    float roll = atan2f(accel_g[1], accel_g[2]);
    float pitch = atan2f(-accel_g[0],
                        sqrtf(accel_g[1] * accel_g[1] +
                              accel_g[2] * accel_g[2]));

    /* 欧拉角转四元数要用半角。初始 yaw 人为设为 0，不是测出了北方。 */
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    s->q[0] = cr * cp;
    s->q[1] = sr * cp;
    s->q[2] = cr * sp;
    s->q[3] = -sr * sp;
}

int IMU_Update(IMU_t *s, const float gyro_rad[3],
               const float accel_g[3], float dt)
{
    /* dt 单位是秒，1000 Hz 时约 0.001 秒。
     * 大间断期间的运动未知，不用最后一个样本强行补完整段积分。
     * 这个写法也会拒绝 NaN，因为 NaN 的两个比较都不成立。 */
    if (!(dt > 0.0f && dt <= 0.002f)) {
        return 0;
    }

    /* 四个新分量必须全部基于同一组旧值，不能边算边覆盖。 */
    const float w = s->q[0];
    const float x = s->q[1];
    const float y = s->q[2];
    const float z = s->q[3];

    /* 应用层已经扣除静止零偏，并将角速度从 度/秒 转为 rad/s。 */
    float gx = gyro_rad[0];
    float gy = gyro_rad[1];
    float gz = gyro_rad[2];

    /* 模长平方，单位 g^2；先比较平方可以在不可信时省去开方。 */
    float accel_norm_squared = accel_g[0] * accel_g[0] +
                               accel_g[1] * accel_g[1] +
                               accel_g[2] * accel_g[2];

    /* 只在 0.9~1.1 g 之间使用重力校正，0.81/1.21 是边界平方。
     * 门限只能排除部分运动加速度，不能识别所有模长仍接近 1 g 的运动。
     * 不符合条件时仍继续陀螺仪积分，不会停止整个姿态更新。 */
    if (accel_norm_squared > 0.81f && accel_norm_squared < 1.21f) {
        float inverse_norm = 1.0f / sqrtf(accel_norm_squared);
        float ax = accel_g[0] * inverse_norm;
        float ay = accel_g[1] * inverse_norm;
        float az = accel_g[2] * inverse_norm;

        /* 将参考系 +Z 转到机体系，得到当前四元数预测的静止加速度方向。 */
        float vx = 2.0f * (x * z - w * y);
        float vy = 2.0f * (w * x + y * z);
        float vz = w * w - x * x - y * y + z * z;

        /* 测量单位向量 × 预测单位向量。
         * 小误差时，叉积近似给出旋转误差的方向和大小，不是欧拉角本身。 */
        float error[3] = {
            ay * vz - az * vy,
            az * vx - ax * vz,
            ax * vy - ay * vx
        };

        for (unsigned i = 0; i < 3; ++i) {
            /* 离散积分乘 dt，使积分强度不会仅因更新频率改变而放大。
             * +/-0.1 rad/s 是积分补偿限幅，不是陀螺仪量程。 */
            s->integral[i] = clampf(s->integral[i] + s->ki * error[i] * dt,
                                    -0.1f, 0.1f);
        }

        /* 比例反馈校正当前倾角误差。 */
        gx += s->kp * error[0];
        gy += s->kp * error[1];
        gz += s->kp * error[2];
    }

    /* 加速度暂时不可信时，保留先前得到的积分反馈。
     * 它不能校正绕重力方向不可观测的绝对航向。 */
    gx += s->integral[0];
    gy += s->integral[1];
    gz += s->integral[2];

    /* 运动学：q_dot = 0.5 * q ⊗ [0,gx,gy,gz]。
     * 使用一阶积分 q_new=q_old+q_dot*dt；这里要求角速度单位 rad/s。 */
    float half_dt = 0.5f * dt;
    float next_q[4] = {
        w + (-x * gx - y * gy - z * gz) * half_dt,
        x + ( w * gx + y * gz - z * gy) * half_dt,
        y + ( w * gy - x * gz + z * gx) * half_dt,
        z + ( w * gz + x * gy - y * gx) * half_dt
    };

    /* 积分误差会使四元数模长偏离 1，必须归一化。
     * 若模长非有限值或近零，返回失败且不覆盖原四元数。 */
    float quaternion_norm_squared = next_q[0] * next_q[0] +
                                    next_q[1] * next_q[1] +
                                    next_q[2] * next_q[2] +
                                    next_q[3] * next_q[3];
    if (!(quaternion_norm_squared > 0.000001f) ||
        !isfinite(quaternion_norm_squared)) {
        return 0;
    }

    float inverse_norm = 1.0f / sqrtf(quaternion_norm_squared);
    for (unsigned i = 0; i < 4; ++i) {
        s->q[i] = next_q[i] * inverse_norm;
    }
    return 1;
}

void IMU_GetAngles(const IMU_t *s, IMU_Angles *out)
{
    /* 使用已经更新、归一化的四元数，不使用旧一轮的乘积缓存。 */
    float w = s->q[0];
    float x = s->q[1];
    float y = s->q[2];
    float z = s->q[3];

    /* Z-Y-X 转换；atan2 保留象限信息，数学库输出乘 RAD2DEG 转为度。 */
    out->roll = atan2f(2.0f * (w * x + y * z),
                       1.0f - 2.0f * (x * x + y * y)) * RAD2DEG;

    /* 舍入误差可能让理论上 <=1 的值略大于 1，限幅防止 asinf 返回 NaN。 */
    out->pitch = asinf(clampf(2.0f * (w * y - z * x), -1.0f, 1.0f)) * RAD2DEG;

    /* yaw 来自完整三维姿态，不是独立积分 gyro_z。
     * +/-180 度边界跳变正常；pitch 接近 +/-90 度时欧拉角存在奇异性。 */
    out->yaw = atan2f(2.0f * (w * z + x * y),
                      1.0f - 2.0f * (y * y + z * z)) * RAD2DEG;
}
