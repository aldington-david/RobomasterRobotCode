/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INS_Task_H
#define INS_Task_H

#include <stdint.h>
#include "struct_typedef.h"
#include "DWT.h"
#include "FusionAhrs.h"
#include "ahrs_ukf.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "fifo.h"


#define SPI_DMA_GYRO_LENGHT       8
//详见BMI088数据手册SPI读取部分,ACC读9舍1刚好8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_IST_DR_SHFITS        0 //片外DataReady标志，在外部中断设置
#define IMU_IST_SPI_I2C_SHFITS       1 //DMA占用标志
#define IMU_IST_UPDATE_SHFITS    2 //读取完成，新数据已装入缓存标志
#define IMU_IST_NOTIFY_SHFITS    3 //通知INS_task标志


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4950.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4950.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

#define MAG_FIFO_BUF_LENGTH 960

typedef struct {
    float32_t rotation_factor[3][3];
    float32_t offset[3];
    float32_t scale[3];
} IMU_IST_Cali_t;

extern float32_t INS_angle[3];

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c3;

/**
  * @brief          获取INS_task栈大小
  * @param[in]      none
  * @retval         INS_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_INS_task(void);

/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void INS_task(void const *pvParameters);

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
extern void INS_cali_gyro(float32_t cali_scale[3], float32_t cali_offset[3], uint16_t *time_count);

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         none
  */
extern void gyro_set_cali(float32_t cali_scale[3], float32_t cali_offset[3]);

extern void mag_set_cali(float32_t cali_scale[3], float32_t cali_offset[3]);

extern void
calc_mag_cali(float32_t *mag_x_offset, float32_t *mag_y_offset, float32_t *mag_z_offset, float32_t *mag_x_scale,
              float32_t *mag_y_scale, float32_t *mag_z_scale,
              float32_t *mag_x_max, float32_t *mag_x_min, float32_t *mag_y_max, float32_t *mag_y_min);

extern void mag_cali_data_record(float32_t *mag_x_max, float32_t *mag_x_min, float32_t *mag_y_max, float32_t *mag_y_min,
                                 float32_t mag[3]);
/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
extern void
imu_ist_rotate(float32_t gyro[3], float32_t accel[3], float32_t mag[3], bmi088_real_data_t *bmi088,
               ist8310_real_data_t *ist8310);

extern void
imu_ist_cali(float32_t gyro[3], float32_t accel[3], float32_t mag[3], float32_t gyro_cali[3], float32_t accel_cali[3],
             float32_t mag_cali[3]);

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */
/**
  * @brief          获取四元数
  * @param[in]      none
  * @retval         INS_quat的指针
  */
extern const float32_t *get_INS_quat_point(void);


/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
  */
extern const float32_t *get_INS_angle_point(void);


/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const float32_t *get_gyro_data_point(void);


/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const float32_t *get_accel_data_point(void);

/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
  * @param[in]      none
  * @retval         INS_mag的指针
  */
extern const float32_t *get_mag_data_point(void);

extern AHRS_time_record_t IMU_time_record;
extern float32_t INS_gyro[3];
extern float32_t INS_accel[3];
extern float32_t INS_mag[3];
extern float32_t INS_gyro_cali[3];
extern float32_t INS_accel_cali[3];
extern float32_t INS_mag_cali[3];
extern float32_t INS_quat[4];
extern float32_t INS_angle_ukf[3];
extern AHRS_t IMU;
extern fifo_s_t mag_data_tx_fifo;
extern IMU_IST_Cali_t gyro_cali_data;
extern IMU_IST_Cali_t accel_cali_data;
extern IMU_IST_Cali_t mag_cali_data;
#endif
