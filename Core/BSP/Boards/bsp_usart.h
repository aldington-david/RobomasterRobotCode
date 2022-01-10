#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"
#include "referee_task.h"


extern void usart6_init(void);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);

void usart6_SendData(uint8_t *Data, uint16_t Size);
void usart_SendData(drv_uart_t *drv, uint8_t *txData, uint16_t size);

#endif
