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

#include "ist8310driver.h"
#include "ist8310driver_middleware.h"
#include "SEGGER_RTT.h"

//#define MAG_SEN 0.3f //转换成 uT
#define MAG_SEN 1.0f //转换成 uT

#define IST8310_WHO_AM_I 0x00       //ist8310 who am I 寄存器
#define IST8310_WHO_AM_I_VALUE 0x10 //设备 ID

#define IST8310_WRITE_REG_NUM 4 //IST8310需要设置的寄存器数目

uint8_t mag_dma_rx_buf[I2C_MEG_LENGHT];

#define OTPsensitivity (330)
float32_t crossaxis_inv[9];
int32_t crossaxis_det[1];
//average 2 times
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] =
        {
                {0x0B, 0x08, 0x01},
                {0x41, 0x24, 0x02},
                {0x42, 0xC0, 0x03},
                {0x0A, 0x0B, 0x04}};

//static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] =
//        {
//                {0x0B, 0x08, 0x01},
//                {0x41, 0x24, 0x02},
//                {0x42, 0xC0, 0x03},
//                {0x0A, 0x02, 0x04}};

uint8_t ist8310_init(void) {
    static const uint8_t wait_time = 1;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    ist8310_RST_L();
    ist8310_delay_ms(sleepTime);
    ist8310_RST_H();
    ist8310_delay_ms(sleepTime);

    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE) {
        return IST8310_NO_SENSOR;
    }
    ist8310_delay_ms(wait_time);
    //set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++) {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0],
                                     ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_ms(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_ms(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1]) {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    int crossaxis_enable = 0;
    char cross_mask[1];
    uint8_t wbuffer[2];

    cross_mask[0] = 0xFF;
    ist8310_IIC_read_muli_reg(IST8310_REG_XX_CROSS_L, wbuffer, 2);

    if ((wbuffer[0] == cross_mask[0]) && (wbuffer[1] == cross_mask[0]))
        crossaxis_enable = 0;
    else
        crossaxis_enable = 1;

    ist8310_Crossaxis_Matrix(crossaxisinv_bitshift, crossaxis_enable);

    return IST8310_NO_ERROR;
}

//void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
//{
//
//    if (status_buf[0] & 0x01)
//    {
//        int16_t temp_ist8310_data = 0;
//        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;
//
//        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
//        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
//        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
//        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
//        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
//        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
//    }
//    else
//    {
//        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
//    }
//}

void ist8310_read_over(uint8_t *rx_buf, float32_t mag[3]) {
    int16_t temp_ist8310_data[3] = {0};
    temp_ist8310_data[0] = (int16_t) ((rx_buf[1] << 8) | rx_buf[0]);
    temp_ist8310_data[1] = (int16_t) ((rx_buf[3] << 8) | rx_buf[2]);
    temp_ist8310_data[2] = (int16_t) ((rx_buf[5] << 8) | rx_buf[4]);
    ist8310_CrossaxisTransformation(temp_ist8310_data);
    mag[0] = MAG_SEN * temp_ist8310_data[0];
    mag[1] = MAG_SEN * temp_ist8310_data[1];
    mag[2] = MAG_SEN * temp_ist8310_data[2];
}

bool_t ist8310_read_mag(float32_t mag[3]) {
    uint8_t buf[6];
    int16_t temp_ist8310_data[3] = {0};
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data[0] = (int16_t) ((buf[1] << 8) | buf[0]);
    temp_ist8310_data[1] = (int16_t) ((buf[3] << 8) | buf[2]);
    temp_ist8310_data[2] = (int16_t) ((buf[5] << 8) | buf[4]);
    ist8310_CrossaxisTransformation(temp_ist8310_data);
    mag[0] = MAG_SEN * temp_ist8310_data[0];
    mag[1] = MAG_SEN * temp_ist8310_data[1];
    mag[2] = MAG_SEN * temp_ist8310_data[2];

    if ((mag[0] == 0) && (mag[1] == 0) && (mag[2] == 0)) {
        return 0;
    } else {
        return 1;
    }
}


void ist8310_Crossaxis_Matrix(int bitshift, int enable) {
//    int ret;
    int i = 0;
    uint8_t crossxbuf[6];
    uint8_t crossybuf[6];
    uint8_t crosszbuf[6];
    short OTPcrossaxis[9] = {0};

    float inv[9] = {0};


    if (enable == 0) {
        DET_eql_0:

        *crossaxis_inv = 1;

        *(crossaxis_inv + 1) = 0;
        *(crossaxis_inv + 2) = 0;
        *(crossaxis_inv + 3) = 0;

        *(crossaxis_inv + 4) = 1;

        *(crossaxis_inv + 5) = 0;
        *(crossaxis_inv + 6) = 0;
        *(crossaxis_inv + 7) = 0;

        *(crossaxis_inv + 8) = 1;

        *crossaxis_det = 1;

//        for (i=0; i<9; i++)
//        {
//            printf("*(crossaxis_inv + %d) = %lld\n", i, *(crossaxis_inv+i));
//        }
//        printf("det = %d\n",*crossaxis_det);
        return;
    } else {
        ist8310_IIC_read_muli_reg(IST8310_REG_XX_CROSS_L, crossxbuf, 6);

        ist8310_IIC_read_muli_reg(IST8310_REG_YX_CROSS_L, crossybuf, 6);

        ist8310_IIC_read_muli_reg(IST8310_REG_ZX_CROSS_L, crosszbuf, 6);


        OTPcrossaxis[0] = ((int16_t) crossxbuf[1]) << 8 | crossxbuf[0];
        OTPcrossaxis[3] = ((int16_t) crossxbuf[3]) << 8 | crossxbuf[2];
        OTPcrossaxis[6] = ((int16_t) crossxbuf[5]) << 8 | crossxbuf[4];
        OTPcrossaxis[1] = ((int16_t) crossybuf[1]) << 8 | crossybuf[0];
        OTPcrossaxis[4] = ((int16_t) crossybuf[3]) << 8 | crossybuf[2];
        OTPcrossaxis[7] = ((int16_t) crossybuf[5]) << 8 | crossybuf[4];
        OTPcrossaxis[2] = ((int16_t) crosszbuf[1]) << 8 | crosszbuf[0];
        OTPcrossaxis[5] = ((int16_t) crosszbuf[3]) << 8 | crosszbuf[2];
        OTPcrossaxis[8] = ((int16_t) crosszbuf[5]) << 8 | crosszbuf[4];
        *crossaxis_det = ((int32_t) OTPcrossaxis[0]) * OTPcrossaxis[4] * OTPcrossaxis[8] +
                         ((int32_t) OTPcrossaxis[1]) * OTPcrossaxis[5] * OTPcrossaxis[6] +
                         ((int32_t) OTPcrossaxis[2]) * OTPcrossaxis[3] * OTPcrossaxis[7] -
                         ((int32_t) OTPcrossaxis[0]) * OTPcrossaxis[5] * OTPcrossaxis[7] -
                         ((int32_t) OTPcrossaxis[2]) * OTPcrossaxis[4] * OTPcrossaxis[6] -
                         ((int32_t) OTPcrossaxis[1]) * OTPcrossaxis[3] * OTPcrossaxis[8];

        if (*crossaxis_det == 0) {
            goto DET_eql_0;
        }

        inv[0] = (float) OTPcrossaxis[4] * OTPcrossaxis[8] - (float) OTPcrossaxis[5] * OTPcrossaxis[7];
        inv[1] = (float) OTPcrossaxis[2] * OTPcrossaxis[7] - (float) OTPcrossaxis[1] * OTPcrossaxis[8];
        inv[2] = (float) OTPcrossaxis[1] * OTPcrossaxis[5] - (float) OTPcrossaxis[2] * OTPcrossaxis[4];
        inv[3] = (float) OTPcrossaxis[5] * OTPcrossaxis[6] - (float) OTPcrossaxis[3] * OTPcrossaxis[8];
        inv[4] = (float) OTPcrossaxis[0] * OTPcrossaxis[8] - (float) OTPcrossaxis[2] * OTPcrossaxis[6];
        inv[5] = (float) OTPcrossaxis[2] * OTPcrossaxis[3] - (float) OTPcrossaxis[0] * OTPcrossaxis[5];
        inv[6] = (float) OTPcrossaxis[3] * OTPcrossaxis[7] - (float) OTPcrossaxis[4] * OTPcrossaxis[6];
        inv[7] = (float) OTPcrossaxis[1] * OTPcrossaxis[6] - (float) OTPcrossaxis[0] * OTPcrossaxis[7];
        inv[8] = (float) OTPcrossaxis[0] * OTPcrossaxis[4] - (float) OTPcrossaxis[1] * OTPcrossaxis[3];

        for (i = 0; i < 9; i++) {
            crossaxis_inv[i] = inv[i] * OTPsensitivity / (*crossaxis_det);
        }
    }
}
void ist8310_CrossaxisTransformation(int16_t *xyz){
    int i = 0;

    float outputtmp[3];

    for( i = 0; i < 9; i++){
        if(crossaxis_inv[i]!=0)
            break;
        if(i == 8)
            ist8310_init();
    }

    outputtmp[0] = xyz[0] * crossaxis_inv[0] +
                   xyz[1] * crossaxis_inv[1] +
                   xyz[2] * crossaxis_inv[2];

    outputtmp[1] = xyz[0] * crossaxis_inv[3] +
                   xyz[1] * crossaxis_inv[4] +
                   xyz[2] * crossaxis_inv[5];

    outputtmp[2] = xyz[0] * crossaxis_inv[6] +
                   xyz[1] * crossaxis_inv[7] +
                   xyz[2] * crossaxis_inv[8];

    xyz[0]= (short)(outputtmp[0]);
    xyz[1]= (short)(outputtmp[1]);
    xyz[2]= (short)(outputtmp[2]);

}

void ist8310_MergeHighLowData(uint8_t *raw, int16_t *xyz){
    xyz[0] = (((int16_t)raw[1]) << 8) | raw[0];
//    xyz[0] = SMA(xyz[0], 0);

    xyz[1] = (((int16_t)raw[3]) << 8) | raw[2];
//    xyz[1] = SMA(xyz[1], 1);

    xyz[2] = (((int16_t)raw[5]) << 8) | raw[4];
//    xyz[2] = SMA(xyz[2], 2);
}