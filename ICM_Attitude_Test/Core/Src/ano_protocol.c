/*
 * ano_protocol.c
 *
 * 按用户提供的 ANO_XIEYI.xlsx 中 STATUS 帧实现。
 *
 * 帧结构：
 *
 * 下标    内容
 * 0       0xAA：帧头第一字节
 * 1       0xAA：帧头第二字节
 * 2       0x01：STATUS 功能码
 * 3       0x0C：数据长度，共 12 字节
 * 4~5     int16：横滚角 × 100
 * 6~7     int16：俯仰角 × 100
 * 8~9     int16：偏航角 × 100
 * 10~13   int32：高度，单位 cm
 * 14      uint8：飞行模式
 * 15      uint8：锁定状态
 * 16      uint8：累加校验
 *
 * 总长度：
 *   2字节帧头 + 1字节功能码 + 1字节长度
 *   + 12字节数据 + 1字节校验 = 17字节
 *
 * 字节序：
 *   当前采用高字节在前。
 *   这一项还需通过配套示例 .c 或地面站实际接收确认。
 */

#include "ano_protocol.h"
#include "telemetry.h"

#include <math.h>
#include <stddef.h>

/* STATUS 数据区长度，不包含帧头、功能码、长度和校验。 */
#define ANO_STATUS_PAYLOAD_LENGTH  12U

/* 完整 STATUS 帧长度。 */
#define ANO_STATUS_FRAME_LENGTH   17U


/*
 * 功能：将一个有符号 16 位整数按高字节在前写入数组。
 *
 * 例如：
 *   value = 1000
 *   十六进制为 0x03E8
 *
 * 写入结果：
 *   buffer[0] = 0x03
 *   buffer[1] = 0xE8
 *
 * 对于负数：
 *   先转换为 uint16_t，取得对应的 16 位补码表示，
 *   再拆分字节，避免直接对负数右移带来的实现差异。
 */
static void ANO_WriteInt16BE(uint8_t *buffer, int16_t value)
{
    uint16_t unsigned_value = (uint16_t)value;

    buffer[0] = (uint8_t)(unsigned_value >> 8);
    buffer[1] = (uint8_t)(unsigned_value & 0xFFU);
}


/*
 * 功能：将一个有符号 32 位整数按高字节在前写入数组。
 *
 * 例如：
 *   value = 0x12345678
 *
 * 写入结果：
 *   buffer[0] = 0x12
 *   buffer[1] = 0x34
 *   buffer[2] = 0x56
 *   buffer[3] = 0x78
 */
static void ANO_WriteInt32BE(uint8_t *buffer, int32_t value)
{
    uint32_t unsigned_value = (uint32_t)value;

    buffer[0] = (uint8_t)(unsigned_value >> 24);
    buffer[1] = (uint8_t)(unsigned_value >> 16);
    buffer[2] = (uint8_t)(unsigned_value >> 8);
    buffer[3] = (uint8_t)(unsigned_value & 0xFFU);
}


/*
 * 功能：把浮点角度转换为协议使用的有符号整数。
 *
 * 协议规定：
 *   整数值 = 角度 × 100
 *
 * 例如：
 *   12.34 度 -> 1234
 *   -5.67 度 -> -567
 *
 * int16_t 范围：
 *   -32768 ~ 32767
 *
 * 所以本函数先限幅，再四舍五入，避免超范围转换。
 * NaN/Inf 已经在 ANO_SendStatus() 中提前检查。
 */
static int16_t ANO_AngleToInt16(float angle_deg)
{
    float scaled_angle = angle_deg * 100.0f;

    if (scaled_angle > 32767.0f) {
        scaled_angle = 32767.0f;
    }

    if (scaled_angle < -32768.0f) {
        scaled_angle = -32768.0f;
    }

    return (int16_t)lroundf(scaled_angle);
}


int ANO_SendStatus(float roll_deg,
                   float pitch_deg,
                   float yaw_deg,
                   int32_t altitude_cm,
                   uint8_t fly_mode,
                   uint8_t armed)
{
    /*
     * 数学计算异常时，不要把 NaN 或 Inf 转成整数发给上位机。
     */
    if (!isfinite(roll_deg) ||
        !isfinite(pitch_deg) ||
        !isfinite(yaw_deg)) {
        return 0;
    }

    /*
     * 上一帧还没发送完成时立即返回。
     * 不使用 while 等待，否则会影响 1000 Hz 采集。
     */
    if (Telemetry_IsBusy()) {
        return 0;
    }

    /*
     * 本地数组保存完整协议帧。
     *
     * 可以使用局部数组，是因为 Telemetry_Send() 会先复制
     * 到它自己的静态缓冲区，再启动串口中断发送。
     *
     * 不要绕过 Telemetry_Send()，直接把这个局部数组交给
     * HAL_UART_Transmit_IT() 后立即退出函数。
     */
    uint8_t frame[ANO_STATUS_FRAME_LENGTH] = {0};

    /* 第一步：填写固定帧头、功能码、数据长度。 */
    frame[0] = 0xAA;
    frame[1] = 0xAA;
    frame[2] = 0x01;
    frame[3] = ANO_STATUS_PAYLOAD_LENGTH;

    /* 第二步：把单位为度的角度转换成协议规定的“度 × 100”。 */
    int16_t roll_value  = ANO_AngleToInt16(roll_deg);
    int16_t pitch_value = ANO_AngleToInt16(pitch_deg);
    int16_t yaw_value   = ANO_AngleToInt16(yaw_deg);

    /* 第三步：按协议顺序填写三个姿态角。 */
    ANO_WriteInt16BE(&frame[4], roll_value);
    ANO_WriteInt16BE(&frame[6], pitch_value);
    ANO_WriteInt16BE(&frame[8], yaw_value);

    /* 第四步：填写高度、飞行模式和锁定状态。 */
    ANO_WriteInt32BE(&frame[10], altitude_cm);

    frame[14] = fly_mode;
    frame[15] = armed;

    /*
     * 第五步：计算累加校验。
     *
     * 累加范围：frame[0] ~ frame[15]。
     * 不包含校验字节 frame[16] 自身。
     *
     * 每次转换回 uint8_t，只保留和的低 8 位，
     * 符合表格中的 SUM 定义。
     */
    uint8_t checksum = 0;

    for (size_t i = 0; i < ANO_STATUS_FRAME_LENGTH - 1U; ++i) {
        checksum = (uint8_t)(checksum + frame[i]);
    }

    frame[16] = checksum;

    /*
     * 第六步：发送完整的 17 字节。
     *
     * 二进制数据可能包含 0x00，必须按明确长度发送。
     * 不能用 strlen()，因为它会把 0x00 当作字符串结尾。
     *
     * Telemetry_Send() 接口虽然使用 const char*，
     * 但内部按 length 复制字节，因此可以发送二进制。
     */
    return Telemetry_Send((const char *)frame,
                          (uint16_t)sizeof(frame));
}