#include "bsp_usart.h"
#include "main.h"
#include "memory.h"
#include "detect_task.h"

#define USART6_RX_BUF_LEN            200

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern void usart6_rxDataHandler(uint8_t *rxBuf);

uint8_t usart6_dma_rx_buf[USART6_RX_BUF_LEN];

static HAL_StatusTypeDef
DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength) {
    HAL_StatusTypeDef status = HAL_OK;

    __HAL_LOCK(hdma);
    if (HAL_DMA_STATE_READY == hdma->State) {
        //Change DMA peripheral state
        hdma->State = HAL_DMA_STATE_BUSY;

        //Initialize the error code
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        //Configure the source, destination address and the data length
        //Clear DBM bit
        hdma->Instance->CR &= (uint32_t) (~DMA_SxCR_DBM);

        //Configure DMA Stream data length
        hdma->Instance->NDTR = DataLength;

        //Memory to Peripheral
        if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH) {
            //Configure DMA Stream destination address
            hdma->Instance->PAR = DstAddress;

            //Configure DMA Stream source address
            hdma->Instance->M0AR = SrcAddress;
        } else {
            //Configure DMA Stream destination address
            hdma->Instance->PAR = SrcAddress;

            //Configure DMA Stream source address
            hdma->Instance->M0AR = DstAddress;
        }
        //Enable the Peripheral
        __HAL_DMA_ENABLE(hdma);
    } else {
        //Process unlocked
        __HAL_UNLOCK(hdma);
        status = HAL_BUSY;
    }
    return status;
}

void usart1_tx_dma_init(void) {

    //enable the DMA transfer for the receiver and transmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) &(USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t) (NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}

void usart1_tx_dma_enable(uint8_t *data, uint16_t len) {
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while (hdma_usart1_tx.Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t) (data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}


void usart6_init(void) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    //Enable the DMA transfer for the receiver request
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    DMA_Start(huart6.hdmarx, (uint32_t)&huart6.Instance->DR, (uint32_t)usart6_dma_rx_buf, USART6_RX_BUF_LEN);
}

void usart6_SendData(uint8_t *Data, uint16_t Size) {
    HAL_UART_Transmit(&huart6, Data, Size, 1);
}

void usart_SendData(drv_uart_t *drv, uint8_t *txData, uint16_t size) {
    if (drv->type == DRV_UART6)
        usart6_SendData(txData, size);
}

__WEAK void usart6_rxDataHandler(uint8_t *rxBuf) {

}


void USART6_IRQHandler(void) {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    /* handle dbus data dbus_buf from DMA */
    usart6_rxDataHandler(usart6_dma_rx_buf);
    memset(usart6_dma_rx_buf, 0, USART6_RX_BUF_LEN);
    /* restart dma transmission */
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    detect_hook(REFEREE_TOE);
}