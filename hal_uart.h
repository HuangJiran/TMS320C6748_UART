#ifndef HAL_UART_H_
#define HAL_UART_H_

#include <stdbool.h>
#include <stdint.h>

#include "../ptl/circularBuffer.h"
#include <ti/sysbios/knl/Semaphore.h>
#include "edma3_drv.h"

#define UART_RX_BUFF_SIZE (0x200) // 512Byte
#define UART_TX_BUFF_SIZE (0x200) // 512Byte
#define UART_DATA_LENGTH  (64)

typedef enum HAL_UART_PORT
{
    HAL_UART0 = 0,
    HAL_UART1,
    HAL_UART2,
    HAL_UART_MAX
} HAL_UART_PORT_e;

typedef struct UART_HANDLE
{
    unsigned int uartBaseAdd;
    EDMA3_DRV_PaRAMRegs edma3Param;
    Semaphore_Handle uartRxSem; // 接收信号量，中断接收到数据写入缓冲区后发出此信号
    Semaphore_Handle uartTxEdmaSem; // DMA传输完成信号量，传输数据之前等待此信号量
    CIRCULAR_BUFFER_s uartTxBufferCtrl;
    CIRCULAR_BUFFER_s uartRxBufferCtrl;
    unsigned char uartRxBuff[UART_RX_BUFF_SIZE];
    unsigned char uartTxBuff[UART_TX_BUFF_SIZE];
} UART_HANDLE_s;

extern UART_HANDLE_s uartHandle[HAL_UART_MAX];

int8_t hal_uart_init(HAL_UART_PORT_e uartPort);
EDMA3_DRV_Result uart_edma_write(HAL_UART_PORT_e uartPort, volatile char *buffer, unsigned int buffLength);
void hal_uart_write(HAL_UART_PORT_e uartPort, unsigned char *data, unsigned int len);
int hal_uart_read(HAL_UART_PORT_e uartPort, unsigned int *dataAdder, unsigned int *len, uint32_t timeout);

#endif /* HAL_UART_H_ */
