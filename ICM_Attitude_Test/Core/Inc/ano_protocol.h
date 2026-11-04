#ifndef ANO_PROTOCOL_H
#define ANO_PROTOCOL_H

#include <stdint.h>

/*
 * 功能：发送匿名协议 STATUS 姿态状态帧。
 *
 * 参数：
 *   roll_deg   ：横滚角，单位“度”，直接传入解算结果。
 *   pitch_deg  ：俯仰角，单位“度”。
 *   yaw_deg    ：偏航角，单位“度”。
 *   altitude_cm：高度，单位厘米；当前没有高度数据，传入 0。
 *   fly_mode   ：飞行模式；当前只验证姿态显示，传入 0。
 *   armed      ：锁定状态；0=锁定，1=解锁，当前传入 0。
 *
 * 注意：
 *   角度乘以 100 的转换在函数内部完成。
 *   调用方不要提前乘 100，也不要传入之前的整数毫度。
 *
 * 返回：
 *   1：已经成功启动串口中断发送。
 *   0：角度无效、发送忙或串口启动失败。
 *
 * 本函数只负责协议打包。
 * 实际串口发送继续使用 telemetry.c 的 Telemetry_Send()。
 */
int ANO_SendStatus(float roll_deg,
                   float pitch_deg,
                   float yaw_deg,
                   int32_t altitude_cm,
                   uint8_t fly_mode,
                   uint8_t armed);

#endif