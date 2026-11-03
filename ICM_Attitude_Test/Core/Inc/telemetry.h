#ifndef TELEMETRY_H
#define TELEMETRY_H
#include "main.h"

/* 绑定串口句柄。应用启动时调用一次，此前 USART2 已由 CubeMX 初始化。 */
void Telemetry_Init(UART_HandleTypeDef *uart);

/* 从主循环调用的异步发送接口，不用于多个中断/任务并发提交。
 * 把 length 个字节复制到本模块的静态缓冲区，再启动中断发送。
 * 因而调用方的局部字符数组在本函数返回后可以失效。
 * 返回 1：已启动发送，不等于发送已经完成。
 * 返回 0：忙、参数无效或 HAL 启动失败；调用方不能阻塞等待。
 * 本例忙时跳过本次显示帧，保持采集和解算继续运行。 */
int Telemetry_Send(const char *data, uint16_t length);

/* 发送是否正在进行：非零表示静态缓冲区仍归串口发送过程所有。 */
int Telemetry_IsBusy(void);

/* 调试统计：启动中断发送失败次数、HAL UART 错误回调次数。 */
extern volatile uint32_t telemetry_start_errors;
extern volatile uint32_t telemetry_uart_errors;
#endif
