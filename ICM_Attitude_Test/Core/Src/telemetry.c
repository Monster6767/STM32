/*
 * telemetry.c：主循环提交数据，USART 中断负责发送。
 * 调用链：Telemetry_Send -> HAL_UART_Transmit_IT -> USART2_IRQHandler
 *         -> HAL_UART_IRQHandler -> HAL_UART_TxCpltCallback。
 * UART 在函数返回后仍要使用缓冲区，因此先复制到本模块静态数组。
 */
#include "telemetry.h"
#include <string.h>

static UART_HandleTypeDef *tx_uart; /* 绑定句柄，不把串口实例写死。 */
static uint8_t tx_buffer[256];      /* 发送完成前不得覆盖，整个运行期有效。 */
static volatile uint8_t tx_busy;    /* 主循环置位，中断完成回调清零。 */
volatile uint32_t telemetry_start_errors;
volatile uint32_t telemetry_uart_errors;

void Telemetry_Init(UART_HandleTypeDef *uart)
{
    /* 仅在启动、尚未发送时调用；不能在正在发送时重置 busy。 */
    tx_uart = uart;
    tx_busy = 0;
}

int Telemetry_IsBusy(void)
{
    return tx_busy != 0;
}

int Telemetry_Send(const char *data, uint16_t length)
{
    /* 忙时立即返回，不等待串口，以免阻塞采集。
     * length 为实际字节数，通常不含字符串末尾 '\0'。
     * 本接口限制由主循环单一生产者调用，不支持多个任务并发提交。 */
    if (tx_uart == NULL || tx_busy || data == NULL ||
        length == 0 || length > sizeof(tx_buffer)) {
        return 0;
    }

    memcpy(tx_buffer, data, length);

    /* 先置位，再启动，避免短帧很快完成后回调与主流程状态发生错序。 */
    tx_busy = 1;
    if (HAL_UART_Transmit_IT(tx_uart, tx_buffer, length) != HAL_OK) {
        /* HAL 启动失败，没有成功提交本帧，恢复可提交状态并计数。 */
        tx_busy = 0;
        ++telemetry_start_errors;
        return 0;
    }

    /* 仅表示“已启动”，真正完成要等待 HAL 的发送完成回调。 */
    return 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    /* 回调在中断里执行，只做简短操作，不做融合、格式化或延时。 */
    if (uart == tx_uart) {
        tx_busy = 0;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == tx_uart) {
        ++telemetry_uart_errors;
    }

    /* 错误不一定意味着 TX 已停止，不能随意清 busy 后覆盖缓冲。
     * 本例只统计，长期忙时停止测试、查 HAL 状态并复位。
     * 若后续要自动恢复，需明确实现中止完成后的恢复流程。
     * 工程只能有一份同名 HAL 回调，多串口要在这里按句柄分发。 */
}
