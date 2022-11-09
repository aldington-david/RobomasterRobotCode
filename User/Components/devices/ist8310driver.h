/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310磁力计驱动函数，包括初始化函数，处理数据函数，通信读取函数
  *             本工程是将MPU6500 IIC_SLV0设置为自动读取IST8310数据，读取
  *             MPU_EXT_SENS_DATA_00保存了IST8310的Status，通过判断标志位，来更新
  *             数据。
  * @note       IST8310只支持IIC读取
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

#define I2C_MEG_LENGHT 6

typedef struct ist8310_real_data_t
{
  uint8_t status;
  float32_t mag[3];
} ist8310_real_data_t;

extern ist8310_real_data_t ist8310_real_data;

extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *rx_buf, float32_t mag[3]);
extern bool_t ist8310_read_mag(float32_t mag[3]);

extern uint8_t mag_dma_rx_buf[I2C_MEG_LENGHT];
#endif
