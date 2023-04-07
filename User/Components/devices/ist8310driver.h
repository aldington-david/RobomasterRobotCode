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

//Defines a register address of the IST8310
#define IST8310_REG_WIA				0x00	//Who I am
#define IST8310_REG_INFO			0x01	//More Info
#define IST8310_REG_DATAX			0x03	//Output Value x
#define IST8310_REG_DATAY			0x05	//Output Value y
#define IST8310_REG_DATAZ			0x07	//Output Value z
#define IST8310_REG_STAT1			0x02	//Status register
#define IST8310_REG_STAT2			0x09	//Status register
#define IST8310_REG_CNTRL1			0x0A	//Control setting register 1
#define IST8310_REG_CNTRL2			0x0B	//Control setting register 2
#define IST8310_REG_CNTRL3			0x0D	//Control setting register 3
#define IST8310_REG_OFFSET_START	0xDC	//Offset
#define IST8310_REG_SELECTION_REG   0x42    //Sensor Selection register
#define IST8310_REG_TEST_REG        0x40    //Chip Test register
#define IST8310_REG_AVGCNTL         0x41    //Average Control register
#define IST8310_REG_TUNING_REG      0x47    //Bandgap Tuning register
#define IST8310_SLA                 0x0C    //IST8310 Slave Address

#define IST8310_ODR_MODE            0x01    //Force mode
#define IST8310_DATA_NUM            6       //XYZ High&Low
#define IST8310_AXES_NUM          3
#define crossaxisinv_bitshift (16)
/*---IST8310 cross-axis matrix Address-----------------danny-----*/
#define IST8310_REG_XX_CROSS_L       0x9C   //cross axis xx low byte
#define IST8310_REG_XX_CROSS_H       0x9D   //cross axis xx high byte
#define IST8310_REG_XY_CROSS_L       0x9E   //cross axis xy low byte
#define IST8310_REG_XY_CROSS_H       0x9F   //cross axis xy high byte
#define IST8310_REG_XZ_CROSS_L       0xA0   //cross axis xz low byte
#define IST8310_REG_XZ_CROSS_H       0xA1   //cross axis xz high byte

#define IST8310_REG_YX_CROSS_L       0xA2   //cross axis yx low byte
#define IST8310_REG_YX_CROSS_H       0xA3   //cross axis yx high byte
#define IST8310_REG_YY_CROSS_L       0xA4   //cross axis yy low byte
#define IST8310_REG_YY_CROSS_H       0xA5   //cross axis yy high byte
#define IST8310_REG_YZ_CROSS_L       0xA6   //cross axis yz low byte
#define IST8310_REG_YZ_CROSS_H       0xA7   //cross axis yz high byte

#define IST8310_REG_ZX_CROSS_L       0xA8   //cross axis zx low byte
#define IST8310_REG_ZX_CROSS_H       0xA9   //cross axis zx high byte
#define IST8310_REG_ZY_CROSS_L       0xAA   //cross axis zy low byte
#define IST8310_REG_ZY_CROSS_H       0xAB   //cross axis zy high byte
#define IST8310_REG_ZZ_CROSS_L       0xAC   //cross axis zz low byte
#define IST8310_REG_ZZ_CROSS_H       0xAD   //cross axis zz high byte

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
void ist8310_Crossaxis_Matrix(int bitshift, int enable);
void ist8310_CrossaxisTransformation(int16_t *xyz);
extern void ist8310_MergeHighLowData(uint8_t *raw, int16_t *xyz);
#endif
