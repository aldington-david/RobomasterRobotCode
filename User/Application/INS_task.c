/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.仰头pitch +,左侧顷roll +,右旋yaw +
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

#include "INS_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bsp_i2c.h"
#include "BMI088driver.h"
#include "ist8310driver_middleware.h"
#include "ist8310driver.h"
#include "pid.h"
#include "AHRS.h"
#include "detect_task.h"
#include "DWT.h"
#include "SEGGER_RTT.h"
#include "fifo.h"
#include "bsxlite_interface.h"
#include "gimbal_task.h"


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                     \

AHRS_time_record_t IMU_time_record;



/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(float32_t temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);
/**
  * @brief          open the I2C DMA accord to the value of ist_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据ist_update_flag的值开启I2C DMA
  * @param[in]      none
  * @retval         none
  */
static void ist_cmd_i2c_dma(void);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t INS_task_stack;
#endif


TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};


volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;


bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;
IMU_MAG_Cali_t gyro_cali_data = {{BMI088_BOARD_INSTALL_SPIN_MATRIX},
                                 {0},
                                 {1.0f, 1.0f, 1.0f}};
IMU_MAG_Cali_t accel_cali_data = {{BMI088_BOARD_INSTALL_SPIN_MATRIX},
                                  {0},
                                  {1.0f, 1.0f, 1.0f}};
IMU_MAG_Cali_t mag_cali_data = {{IST8310_BOARD_INSTALL_SPIN_MATRIX},
                                {0},
                                {1.0f, 1.0f, 1.0f}};

static uint8_t first_temperate = 0;
static const float32_t imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;


float32_t INS_gyro[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_accel[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_mag[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_gyro_cali[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_accel_cali[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_mag_cali[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_angle[3] = {0.0f, 0.0f, 0.0f};      //yaw-pitch-roll euler angle, unit rad.欧拉角 单位 rad
float32_t INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; //w x y z 标量在前同matlab
fifo_s_t mag_data_tx_fifo;
uint8_t mag_data_tx_buf[MAG_FIFO_BUF_LENGTH];

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

void INS_task(void const *pvParameters) {
    //wait a time
    vTaskDelay(pdMS_TO_TICKS(INS_TASK_INIT_TIME));
    while (BMI088_init()) {
        osDelay(100);
    }
    while (ist8310_init()) {
        osDelay(100);
    }
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    while (!ist8310_read_mag(ist8310_real_data.mag)) {
        osDelay(1);
    }
    //rotate and zero drift
    imu_mag_rotate(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    imu_mag_cali(INS_gyro, INS_accel, INS_mag, INS_gyro_cali, INS_accel_cali, INS_mag_cali);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT, 1000, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0);
    //get the handle of task
    //获取当前任务的任务句柄，
//    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    INS_task_local_handler = xTaskGetCurrentTaskHandle();

    SPI1_DMA_init((uint32_t) gyro_dma_tx_buf, (uint32_t) gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

    memset(&mag_data_tx_buf, 0, MAG_FIFO_BUF_LENGTH);
    fifo_s_init(&mag_data_tx_fifo, mag_data_tx_buf, MAG_FIFO_BUF_LENGTH);
    vector_3d_t accel_in, gyro_in;
    int32_t w_time_stamp = 0U;
    bsxlite_out_t bsxlite_fusion_out;
    bsxlite_return_t result;
    bsxlite_instance_t instance = 0x00;
    result = bsxlite_init(&instance);
    float32_t init_relative_angle = motor_ecd_to_yaw_angle_change(
            gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control.gimbal_yaw_motor.offset_ecd);
    memset(&accel_in, 0x00, sizeof(accel_in));
    memset(&gyro_in, 0x00, sizeof(gyro_in));
    TickType_t LoopStartTime;
    while (1) {
        DWT_update_task_time_us(&global_task_time.tim_INS_task);
        LoopStartTime = xTaskGetTickCount();
        //wait spi DMA tansmit done
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }

        if (accel_temp_update_flag & (1 << IMU_MAG_UPDATE_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_MAG_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                         &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
        //以统一间隔更新
        if ((gyro_update_flag & (1 << IMU_MAG_NOTIFY_SHFITS)) && (accel_update_flag & (1 << IMU_MAG_UPDATE_SHFITS)) &&
            (mag_update_flag & (1 << IMU_MAG_UPDATE_SHFITS))) {
            gyro_update_flag &= ~(1 << IMU_MAG_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
            DWT_update_task_time_us(&IMU_time_record.gyro);
            accel_update_flag &= ~(1 << IMU_MAG_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel,
                                   &bmi088_real_data.time);
            DWT_update_task_time_us(&IMU_time_record.accel);
            mag_update_flag &= ~(1 << IMU_MAG_UPDATE_SHFITS);
            ist8310_read_over(mag_dma_rx_buf, ist8310_real_data.mag);
            DWT_update_task_time_us(&IMU_time_record.mag);
            imu_mag_rotate(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
            imu_mag_cali(INS_gyro, INS_accel, INS_mag, INS_gyro_cali, INS_accel_cali, INS_mag_cali);
            if (fifo_s_free(&mag_data_tx_fifo)) {
                fifo_s_puts(&mag_data_tx_fifo, (char *) INS_mag_cali, sizeof(INS_mag_cali));
//                SEGGER_RTT_WriteString(0,"yes\r\n");
            } else {
                fifo_s_flush(&mag_data_tx_fifo);
                fifo_s_puts(&mag_data_tx_fifo, (char *) INS_mag_cali, sizeof(INS_mag_cali));
//                SEGGER_RTT_WriteString(0,"no\r\n");
            }
            accel_in.x = INS_accel_cali[0];
            accel_in.y = INS_accel_cali[1];
            accel_in.z = INS_accel_cali[2];
//            SEGGER_RTT_printf(0,"accel:%f,%f,%f\r\n",accel_in.x,accel_in.y,accel_in.z);
            gyro_in.x = INS_gyro_cali[0];
            gyro_in.y = INS_gyro_cali[1];
            gyro_in.z = INS_gyro_cali[2];
            w_time_stamp += 9000U;
            result = bsxlite_do_step(&instance,
                                     w_time_stamp,
                                     &accel_in,
                                     &gyro_in,
                                     &(bsxlite_fusion_out));
            /** evaluate library return */
            if (result != BSXLITE_OK) {
                switch (result) {
                    case (BSXLITE_E_DOSTEPS_TSINTRADIFFOUTOFRANGE): {
                        SEGGER_RTT_WriteString(0,
                                               "Error: Subsequent time stamps in input data were found to be out of range from the expected sample rate!!!\n");
                        break;
                    }
                    case (BSXLITE_E_FATAL): {
                        SEGGER_RTT_WriteString(0, "Fatal Error: Process terminating!!!\n");
                        break;
                    }
                    case (BSXLITE_I_DOSTEPS_NOOUTPUTSRETURNABLE): {
                        SEGGER_RTT_WriteString(0,
                                               "Info: Sufficient memory not allocated for output,  all outputs cannot be returned because no memory provided!!!\n");
                        break;
                    }
                    default: {
                        SEGGER_RTT_WriteString(0, "Info: Unknown return \n");
                        break;
                    }
                }
            }

//                RTT_PrintWave(3,
//                              &bsxlite_fusion_out.orientation.yaw,
//                              &bsxlite_fusion_out.orientation.pitch,
//                              &bsxlite_fusion_out.orientation.roll);
            if ((bsxlite_fusion_out.orientation.yaw + init_relative_angle) > (2 * M_PI)) {
                INS_angle[0] = bsxlite_fusion_out.orientation.yaw + init_relative_angle - 2 * M_PI;
            } else {
                INS_angle[0] = bsxlite_fusion_out.orientation.yaw + init_relative_angle;
            }
//            INS_angle[0]=bsxlite_fusion_out.orientation.yaw;
            INS_angle[1] = bsxlite_fusion_out.orientation.pitch;
//            if (INS_angle[1] > gimbal_control.gimbal_pitch_motor.max_relative_angle) {
//                INS_angle[1] = gimbal_control.gimbal_pitch_motor.max_relative_angle;
//            } else if (INS_angle[1] < gimbal_control.gimbal_pitch_motor.min_relative_angle) {
//                INS_angle[1] = gimbal_control.gimbal_pitch_motor.min_relative_angle;
//            }

            INS_angle[2] = bsxlite_fusion_out.orientation.roll;

            INS_quat[0] = bsxlite_fusion_out.rotation_vector.w;
            INS_quat[1] = bsxlite_fusion_out.rotation_vector.x;
            INS_quat[2] = bsxlite_fusion_out.rotation_vector.y;
            INS_quat[3] = bsxlite_fusion_out.rotation_vector.z;

#if INCLUDE_uxTaskGetStackHighWaterMark
            INS_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
            vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(9));
        }
    }

}

/**
  * @brief          获取INS_task栈大小
  * @param[in]      none
  * @retval         INS_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_INS_task(void) {
    return INS_task_stack;
}


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
void
imu_mag_cali(float32_t gyro[3], float32_t accel[3], float32_t mag[3], float32_t gyro_cali[3], float32_t accel_cali[3],
             float32_t mag_cali[3]) {
//    for (uint8_t i = 0; i < 3; i++) {
//        gyro_cali[i] = gyro[i] * gyro_cali_data.scale[i] + gyro_cali_data.offset[i];
//        accel_cali[i] = accel[i] * accel_cali_data.scale[i] + accel_cali_data.offset[i];
//        mag_cali[i] = mag[i] * mag_cali_data.scale[i] + mag_cali_data.offset[i];
//    }
    for (uint8_t i = 0; i < 3; i++) {
        gyro_cali[i] = gyro[i] * gyro_cali_data.scale[i];
        accel_cali[i] = accel[i] * accel_cali_data.scale[i];
        mag_cali[i] = mag[i] * mag_cali_data.scale[i];
    }
}

void imu_mag_rotate(float32_t gyro[3], float32_t accel[3], float32_t mag[3], bmi088_real_data_t *bmi088,
                    ist8310_real_data_t *ist8310) {
    for (uint8_t i = 0; i < 3; i++) {
        gyro[i] = bmi088->gyro[0] * gyro_cali_data.rotation_factor[i][0] +
                  bmi088->gyro[1] * gyro_cali_data.rotation_factor[i][1] +
                  bmi088->gyro[2] * gyro_cali_data.rotation_factor[i][2];
        accel[i] = bmi088->accel[0] * accel_cali_data.rotation_factor[i][0] +
                   bmi088->accel[1] * accel_cali_data.rotation_factor[i][1] +
                   bmi088->accel[2] * accel_cali_data.rotation_factor[i][2];
        mag[i] = ist8310->mag[0] * mag_cali_data.rotation_factor[i][0] +
                 ist8310->mag[1] * mag_cali_data.rotation_factor[i][1] +
                 ist8310->mag[2] * mag_cali_data.rotation_factor[i][2];
    }
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(float32_t temp) {
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate) {
        ALL_PID(&imu_temp_pid, temp, 50.0f);
        if (imu_temp_pid.out < 0.0f) {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t) imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
//        SEGGER_RTT_printf(0,"%d\r\n",tempPWM);
    } else {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp < 50.0f) {
            temp_constant_time++;
            if (temp_constant_time > 200) {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.out = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
  * @retval         none
  */
void gyro_offset_calc(float32_t gyro_offset[3], float32_t gyro[3], uint16_t *offset_time_count) {
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL) {
        return;
    }

    gyro_offset[0] -= 0.0003f * gyro[0];
    gyro_offset[1] -= 0.0003f * gyro[1];
    gyro_offset[2] -= 0.0003f * gyro[2];
    (*offset_time_count)++;
}

void mag_cali_data_record(float32_t *mag_x_max, float32_t *mag_x_min, float32_t *mag_y_max, float32_t *mag_y_min,
                          float32_t mag[3]) {
    if (mag_x_max == NULL || mag_x_min == NULL || mag_y_max == NULL || mag_y_min == NULL || mag == NULL) {
        return;
    }
    if (*mag_x_max < mag[0]) {
        *mag_x_max = mag[0];
    }
    if (*mag_x_min > mag[0]) {
        *mag_x_min = mag[0];
    }
    if (*mag_y_max < mag[1]) {
        *mag_y_max = mag[1];
    }
    if (*mag_y_min > mag[1]) {
        *mag_y_min = mag[1];
    }
}

void calc_mag_cali(float32_t *mag_x_offset, float32_t *mag_y_offset, float32_t *mag_z_offset, float32_t *mag_x_scale,
                   float32_t *mag_y_scale, float32_t *mag_z_scale, float32_t *mag_x_max, float32_t *mag_x_min,
                   float32_t *mag_y_max, float32_t *mag_y_min) {
    if (mag_x_offset == NULL || mag_y_offset == NULL || mag_z_offset == NULL || mag_x_scale == NULL ||
        mag_y_scale == NULL || mag_z_scale == NULL ||
        mag_x_max == NULL || mag_x_min == NULL || mag_y_max == NULL || mag_y_min == NULL) {
        return;
    }
    float32_t mag_x_len, mag_y_len;

    mag_x_len = *mag_x_max - *mag_x_min;
    mag_y_len = *mag_y_max - *mag_y_min;
    *mag_x_scale = 1.0f;
    *mag_y_scale = mag_x_len / mag_y_len;
    *mag_z_scale = 1.0f;
    *mag_x_offset = *mag_x_scale * (0.5f * mag_x_len - *mag_x_max);
    *mag_y_offset = *mag_y_scale * (0.5f * mag_y_len - *mag_y_max);
    *mag_z_offset = 0.0f;

}

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
void INS_cali_gyro(float32_t cali_scale[3], float32_t cali_offset[3], uint16_t *time_count) {
    if (*time_count == 0) {
        cali_offset[0] = 0.0f;
        cali_offset[1] = 0.0f;
        cali_offset[2] = 0.0f;
    }
    gyro_offset_calc(cali_offset, INS_gyro, time_count);
    cali_scale[0] = 1.0f;
    cali_scale[1] = 1.0f;
    cali_scale[2] = 1.0f;

}

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
void gyro_set_cali(float32_t cali_scale[3], float32_t cali_offset[3]) {
    gyro_cali_data.offset[0] = cali_offset[0];
    gyro_cali_data.offset[1] = cali_offset[1];
    gyro_cali_data.offset[2] = cali_offset[2];
    gyro_cali_data.scale[0] = cali_scale[0];
    gyro_cali_data.scale[1] = cali_scale[1];
    gyro_cali_data.scale[2] = cali_scale[2];
}

void mag_set_cali(float32_t cali_scale[3], float32_t cali_offset[3]) {
    mag_cali_data.offset[0] = cali_offset[0];
    mag_cali_data.offset[1] = cali_offset[1];
    mag_cali_data.offset[2] = cali_offset[2];
    mag_cali_data.scale[0] = cali_scale[0];
    mag_cali_data.scale[1] = cali_scale[1];
    mag_cali_data.scale[2] = cali_scale[2];
}

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
const float32_t *get_INS_quat_point(void) {
    return INS_quat;
}
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
const float32_t *get_INS_angle_point(void) {
    return INS_angle;
}

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
extern const float32_t *get_gyro_data_point(void) {
    return INS_gyro_cali;
}
/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_accel
  */
/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_accel的指针
  */
extern const float32_t *get_accel_data_point(void) {
    return INS_accel_cali;
}
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
extern const float32_t *get_mag_data_point(void) {
    return INS_mag_cali;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT1_ACCEL_Pin) {
        detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_MAG_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_MAG_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == INT1_GYRO_Pin) {
        detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_MAG_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == DRDY_IST8310_Pin) {
        detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_MAG_DR_SHFITS;
        if (imu_start_dma_flag) {
            ist_cmd_i2c_dma();
        }
//        ist8310_IIC_read_muli_reg(0x03, mag_dma_rx_buf, 6);
    } else if (GPIO_Pin == GPIO_PIN_0) {

        //wake up the task
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }


}

/**
  * @brief          open the I2C DMA accord to the value of ist_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据ist_update_flag的值开启I2C DMA
  * @param[in]      none
  * @retval         none
  */
static void ist_cmd_i2c_dma(void) {
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    //开启磁力计的DMA传输
    if ((mag_update_flag & (1 << IMU_MAG_DR_SHFITS)) && !(hi2c3.hdmarx->Instance->CR & DMA_SxCR_EN)) {
        mag_update_flag &= ~(1 << IMU_MAG_DR_SHFITS);
        mag_update_flag |= (1 << IMU_MAG_SPI_I2C_SHFITS);
        ist8310_IIC_DMA_read_muli_reg(0x03, mag_dma_rx_buf, 6);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      none
  * @retval         none
  */
static void imu_cmd_spi_dma(void) {
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    //开启陀螺仪的DMA传输
    if ((gyro_update_flag & (1 << IMU_MAG_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS)) &&
        !(accel_temp_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS))) {
        gyro_update_flag &= ~(1 << IMU_MAG_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_MAG_SPI_I2C_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) gyro_dma_tx_buf, (uint32_t) gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    //开启加速度计的DMA传输
    if ((accel_update_flag & (1 << IMU_MAG_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS)) &&
        !(accel_temp_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS))) {
        accel_update_flag &= ~(1 << IMU_MAG_DR_SHFITS);
        accel_update_flag |= (1 << IMU_MAG_SPI_I2C_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) accel_dma_tx_buf, (uint32_t) accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }


    if ((accel_temp_update_flag & (1 << IMU_MAG_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS)) &&
        !(accel_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS))) {
        accel_temp_update_flag &= ~(1 << IMU_MAG_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_MAG_SPI_I2C_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) accel_temp_dma_tx_buf, (uint32_t) accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


void DMA2_Stream2_IRQHandler(void) {

    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if (gyro_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_MAG_SPI_I2C_SHFITS);
            gyro_update_flag |= (1 << IMU_MAG_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

        }

        //accel read over
        //加速度计读取完毕
        if (accel_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_MAG_SPI_I2C_SHFITS);
            accel_update_flag |= (1 << IMU_MAG_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if (accel_temp_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_MAG_SPI_I2C_SHFITS);
            accel_temp_update_flag |= (1 << IMU_MAG_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //循环DMA读取(每次读取1一个8位寄存器)
        imu_cmd_spi_dma();
        //以更新速率最快的触发IMU数据处理任务(按需修改)
        if (gyro_update_flag & (1 << IMU_MAG_UPDATE_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_MAG_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_MAG_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c3) {
        if (mag_update_flag & (1 << IMU_MAG_SPI_I2C_SHFITS)) {
            mag_update_flag &= ~(1 << IMU_MAG_SPI_I2C_SHFITS);
            mag_update_flag |= (1 << IMU_MAG_UPDATE_SHFITS);
        }
    }
}
