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

#include "INS_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "BMI088driver.h"
#include "ist8310driver_middleware.h"
#include "ist8310driver.h"
#include "pid.h"
#include "AHRS.h"

#include "calibrate_task.h"
#include "detect_task.h"
#include "DWT.h"
#include "matrix.h"
#include "ahrs_ukf.h"
#include "SEGGER_RTT.h"
#include "FusionAhrs.h"
//#include "calibrate_ukf.h"

FusionAhrs FAhrs;

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                    \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {0.0f, 1.0f, 0.0f},                     \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, -1.0f}                      \

AHRS_time_record_t IMU_time_record;
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
imu_cali_slove(float32_t gyro[3], float32_t accel[3], float32_t mag[3], bmi088_real_data_t *bmi088,
               ist8310_real_data_t *ist8310);

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

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t INS_task_stack;
#endif

extern SPI_HandleTypeDef hspi1;


static TaskHandle_t INS_task_local_handler;

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
float32_t gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float32_t gyro_offset[3];
float32_t gyro_cali_offset[3];

float32_t accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float32_t accel_offset[3];
float32_t accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
float32_t mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
float32_t mag_offset[3];
float32_t mag_cali_offset[3];

static uint8_t first_temperate;
static const float32_t imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static pid_type_def imu_temp_pid;

static const float timing_time = 0.005f;   //tast run time , unit s.任务运行的时间 单位 s


//加速度计低通滤波
static float32_t accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float32_t accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float32_t accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float32_t fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


float32_t INS_gyro[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_accel[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_mag[3] = {0.0f, 0.0f, 0.0f};
float32_t INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float32_t INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
float32_t INS_angle_ukf[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad

//TRICAL_instance_t mag_calib;
float32_t expected_field[3] = {-3.7151f, 28.3972f, -46.1396f};
float calibrated_reading[3] = {0};
//new_AHRS
AHRS_t IMU;

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
    ist8310_read_over(mag_dma_rx_buf, ist8310_real_data.mag);
    //rotate and zero drift
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 0);
//    FusionAhrsInitialise(&FAhrs, 10.0f);
//    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
//    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
//    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
    //get the handle of task
    //获取当前任务的任务句柄，
//    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));
    INS_task_local_handler = xTaskGetCurrentTaskHandle();
    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }


    SPI1_DMA_init((uint32_t) gyro_dma_tx_buf, (uint32_t) gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;

//    SEGGER_RTT_printf(0,"g=%f\r\n",bmi088_real_data.gyro[0]);
//    SEGGER_RTT_printf(0,"A=%f\r\n",bmi088_real_data.accel[0]);
//    SEGGER_RTT_printf(0,"m=%f\r\n",ist8310_real_data.mag[0]);
//    SEGGER_RTT_printf(0,"%f\r\n",FAhrs.quaternion.array[0]);
//    SEGGER_RTT_printf(0,"%f\r\n",FAhrs.quaternion.array[1]);
//    SEGGER_RTT_printf(0,"%f\r\n",FAhrs.quaternion.array[2]);
//    SEGGER_RTT_printf(0,"%f\r\n",FAhrs.quaternion.array[3]);
//    AHRS_init(INS_quat, INS_accel, INS_mag);
    NEWAHRS_init(&IMU);
    float32_t P_check_num = 10;
    while (!(P_check_num < 2e-6f)) {
        if (mag_update_flag & (1 << IMU_DR_SHFITS)) {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            ist8310_read_over(mag_dma_rx_buf, ist8310_real_data.mag);
            imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
            float32_t Bx_RLS = INS_mag[0];
            float32_t By_RLS = INS_mag[1];
            float32_t Bz_RLS = INS_mag[2];
            matrix_f32_t P_check;
            Matrix_vinit(&P_check);
            matrix_f32_t _temp_T;
            matrix_f32_t _temp_M;
            matrix_f32_t _temp_M2;
            matrix_f32_t _temp_AS;
            Matrix_vinit(&_temp_T);
            Matrix_vinit(&_temp_M);
            Matrix_vinit(&_temp_M2);
            Matrix_vinit(&_temp_AS);
            RLS_in.p2Data[0][0] = Bx_RLS;
            RLS_in.p2Data[1][0] = By_RLS;
            RLS_in.p2Data[2][0] = Bz_RLS;
            RLS_in.p2Data[3][0] = 1;
            RLS_out.p2Data[0][0] = (Bx_RLS * Bx_RLS) + (By_RLS * By_RLS) + (Bz_RLS * Bz_RLS);
            Matrix_vTranspose_nsame(&RLS_in, &_temp_T);
            Matrix_vmult_nsame(&_temp_T, &RLS_theta, &_temp_M);

            Matrix_vsub(&RLS_out, &_temp_M, &_temp_AS);
            float32_t err = 0.01f*_temp_AS.p2Data[0][0];
            SEGGER_RTT_printf(0, "%f\r\n", P_check_num);
            Matrix_vmult_nsame(&_temp_T, &RLS_P, &_temp_M);
            Matrix_vmult_nsame(&_temp_M, &RLS_in, &_temp_M2);
            Matrix_vRoundingadd(&_temp_M2, RLS_lambda);
            Matrix_vmult_nsame(&RLS_P, &RLS_in, &_temp_M);
            Matrix_vscale(&_temp_M, (1.0f / _temp_M2.p2Data[0][0]));
            Matrix_vMove(&_temp_M, &RLS_gain);
            Matrix_vmult_nsame(&RLS_gain, &_temp_T, &_temp_M);
            Matrix_vmult_nsame(&_temp_M, &RLS_P, &_temp_M2);
            Matrix_vsub(&RLS_P, &_temp_M2, &RLS_P);
            Matrix_vscale(&RLS_P, (1.0f / RLS_lambda));
            Matrix_vCopy(&RLS_gain, &_temp_AS);
            Matrix_vscale(&_temp_AS, err);
            Matrix_vsub(&RLS_theta, &_temp_AS, &RLS_theta);

            Matrix_vGetDiagonalEntries(&RLS_P, &P_check);
            Matrix_vTranspose_nsame(&P_check, &_temp_T);
            Matrix_vmult_nsame(&_temp_T, &P_check, &_temp_M);
            P_check_num = _temp_M.p2Data[0][0];
            Matrix_vSetMatrixInvalid(&P_check);
            Matrix_vSetMatrixInvalid(&_temp_T);
            Matrix_vSetMatrixInvalid(&_temp_M);
            Matrix_vSetMatrixInvalid(&_temp_M2);
            Matrix_vSetMatrixInvalid(&_temp_AS);
        }
        osDelay(5);
    }
    SEGGER_RTT_WriteString(0, "D\r\n");
    IMU.HARD_IRON_BIAS.p2Data[0][0] = RLS_theta.p2Data[0][0] / 2.0f;
    IMU.HARD_IRON_BIAS.p2Data[1][0] = RLS_theta.p2Data[1][0] / 2.0f;
    IMU.HARD_IRON_BIAS.p2Data[2][0] = RLS_theta.p2Data[2][0] / 2.0f;
    SEGGER_RTT_printf(0, "%f\r\n", IMU.HARD_IRON_BIAS.p2Data[0][0]);
    SEGGER_RTT_printf(0, "%f\r\n", IMU.HARD_IRON_BIAS.p2Data[1][0]);
    SEGGER_RTT_printf(0, "%f\r\n", IMU.HARD_IRON_BIAS.p2Data[2][0]);
    uint16_t stable_cnt = 0;

    while (stable_cnt < 400) {
        BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
        ist8310_read_over(mag_dma_rx_buf, ist8310_real_data.mag);
        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
        bmi088_real_data.accel_err[0] = bmi088_real_data.accel_last[0] - bmi088_real_data.accel[0];
        bmi088_real_data.accel_err[1] = bmi088_real_data.accel_last[1] - bmi088_real_data.accel[1];
        bmi088_real_data.accel_err[2] = bmi088_real_data.accel_last[2] - bmi088_real_data.accel[2];
        if ((0.1f > fabsf(bmi088_real_data.accel_err[0])) && (0.1f > fabsf(bmi088_real_data.accel_err[1])) &&
            (0.1f > fabsf(bmi088_real_data.accel_err[2]))) {
            stable_cnt++;
        }
        bmi088_real_data.accel_last[0] = bmi088_real_data.accel[0];
        bmi088_real_data.accel_last[1] = bmi088_real_data.accel[1];
        bmi088_real_data.accel_last[2] = bmi088_real_data.accel[2];
        osDelay(5);
    }
    AHRS_vset_north(&IMU);
//    INS_mag[0] -=IMU.HARD_IRON_BIAS.p2Data[0][0];
//    INS_mag[1] -=IMU.HARD_IRON_BIAS.p2Data[1][0];
//    INS_mag[2] -=IMU.HARD_IRON_BIAS.p2Data[2][0];
//    AHRS_init(INS_quat, INS_accel, INS_mag);

//    Matrix_data_creat(&quaternionData, SS_X_LEN, 1, INS_quat, InitMatWithZero);
    UKF_vReset(&UKF_IMU, &quaternionData, &UKF_PINIT, &UKF_Rv, &UKF_Rn);
//    TRICAL_init(&mag_calib);
//    TRICAL_norm_set(&mag_calib, 54.305f);
//    TRICAL_noise_set(&mag_calib, 2e-1f);
    TickType_t LoopStartTime;
    while (1) {
        DWT_update_task_time_us(&global_task_time.tim_INS_task);
        LoopStartTime = xTaskGetTickCount();
        //wait spi DMA tansmit done
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
        }


//        if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS)) {
//            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
//            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
//            DWT_update_task_time_us(&IMU_time_record.gyro);
//        }
//
//        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS)) {
//            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
//            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel,
//                                   &bmi088_real_data.time);
//            DWT_update_task_time_us(&IMU_time_record.accel);
//        }
//
//        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS)) {
//            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
//            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
//                                         &bmi088_real_data.temp);
//            imu_temp_control(bmi088_real_data.temp);
//        }
//
//        if (mag_update_flag & (1 << IMU_DR_SHFITS)) {
//            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
//            ist8310_read_over(mag_dma_rx_buf, ist8310_real_data.mag);
//            DWT_update_task_time_us(&IMU_time_record.mag);
//        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                         &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }

        if ((gyro_update_flag & (1 << IMU_NOTIFY_SHFITS)) && (accel_update_flag & (1 << IMU_UPDATE_SHFITS)) &&
            (mag_update_flag & (1 << IMU_DR_SHFITS))) {
            gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
            DWT_update_task_time_us(&IMU_time_record.gyro);
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel,
                                   &bmi088_real_data.time);
            DWT_update_task_time_us(&IMU_time_record.accel);
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            ist8310_read_over(mag_dma_rx_buf, ist8310_real_data.mag);
            DWT_update_task_time_us(&IMU_time_record.mag);
            imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
//            TRICAL_estimate_update(&mag_calib, ist8310_real_data.mag, expected_field);




//        TRICAL_measurement_calibrate(&mag_calib, ist8310_real_data.mag,
//                                     calibrated_reading);
//        SEGGER_RTT_printf(0, "x=%f, y=%f, z=%f\r\n", calibrated_reading[0], calibrated_reading[1],
//                          calibrated_reading[2]);

//        float32_t Ax = bmi088_real_data.accel[0];
//        float32_t Ay = bmi088_real_data.accel[1];
//        float32_t Az = bmi088_real_data.accel[2];
//        float32_t Bx = ist8310_real_data.mag[0];
//        float32_t By = ist8310_real_data.mag[1];
//        float32_t Bz = ist8310_real_data.mag[2];
//        float32_t Bx = ist8310_real_data.mag[0] - IMU.HARD_IRON_BIAS.p2Data[0][0];
//        float32_t By = ist8310_real_data.mag[1] - IMU.HARD_IRON_BIAS.p2Data[1][0];
//        float32_t Bz = ist8310_real_data.mag[2] - IMU.HARD_IRON_BIAS.p2Data[2][0];
//        float32_t p = bmi088_real_data.gyro[0];
//        float32_t q = bmi088_real_data.gyro[1];
//        float32_t r = bmi088_real_data.gyro[2];
            float32_t Ax = INS_accel[0];
            float32_t Ay = INS_accel[1];
            float32_t Az = INS_accel[2];
            float32_t Bx = INS_mag[0];
            float32_t By = INS_mag[1];
            float32_t Bz = INS_mag[2];
//        float32_t Bx = ist8310_real_data.mag[0] - IMU.HARD_IRON_BIAS.p2Data[0][0];
//        float32_t By = ist8310_real_data.mag[1] - IMU.HARD_IRON_BIAS.p2Data[1][0];
//        float32_t Bz = ist8310_real_data.mag[2] - IMU.HARD_IRON_BIAS.p2Data[2][0];
            float32_t p = INS_gyro[0];
            float32_t q = INS_gyro[1];
            float32_t r = INS_gyro[2];
            Matrix_vassignment(&U, 1, 1, p);
            Matrix_vassignment(&U, 2, 1, q);
            Matrix_vassignment(&U, 3, 1, r);
            Matrix_vassignment(&Y, 1, 1, Ax);
            Matrix_vassignment(&Y, 2, 1, Ay);
            Matrix_vassignment(&Y, 3, 1, Az);
            Matrix_vassignment(&Y, 4, 1, Bx);
            Matrix_vassignment(&Y, 5, 1, By);
            Matrix_vassignment(&Y, 6, 1, Bz);

            /* Normalizing the output vector */
            float32_t _normG = sqrtf(Y.p2Data[0][0] * Y.p2Data[0][0]) + (Y.p2Data[1][0] * Y.p2Data[1][0]) +
                               (Y.p2Data[2][0] * Y.p2Data[2][0]);
            Y.p2Data[0][0] = Y.p2Data[0][0] / _normG;
            Y.p2Data[1][0] = Y.p2Data[1][0] / _normG;
            Y.p2Data[2][0] = Y.p2Data[2][0] / _normG;
            float32_t _normM = sqrtf(Y.p2Data[3][0] * Y.p2Data[3][0]) + (Y.p2Data[4][0] * Y.p2Data[4][0]) +
                               (Y.p2Data[5][0] * Y.p2Data[5][0]);
            Y.p2Data[3][0] = Y.p2Data[3][0] / _normM;
            Y.p2Data[4][0] = Y.p2Data[4][0] / _normM;
            Y.p2Data[5][0] = Y.p2Data[5][0] / _normM;
            /* ------------------ Read the sensor data / simulate the system here ------------------ */
            /* ============================= Update the Kalman Filter ============================== */
            if (!UKF_bUpdate(&UKF_IMU, &Y, &U, &IMU)) {
                Matrix_vSetToZero(&quaternionData);
                Matrix_vassignment(&quaternionData, 1, 1, 1.0f);
                UKF_vReset(&UKF_IMU, &quaternionData, &UKF_PINIT, &UKF_Rv, &UKF_Rn);
            }
        }
        /* ----------------------------- Update the Kalman Filter ------------------------------ */
        /* ================== Read the sensor data / simulate the system here ================== */
//        float32_t P_check_num = 0;
//        while (P_check_num < 1e-4f) {
//            matrix_f32_t P_check;
//            Matrix_vinit(&P_check);
//            matrix_f32_t _temp_T;
//            matrix_f32_t _temp_M;
//            matrix_f32_t _temp_M2;
//            matrix_f32_t _temp_AS;
//            Matrix_vinit(&_temp_T);
//            Matrix_vinit(&_temp_M);
//            Matrix_vinit(&_temp_M2);
//            Matrix_vinit(&_temp_AS);
//            RLS_in.p2Data[0][0] = Bx;
//            RLS_in.p2Data[1][0] = By;
//            RLS_in.p2Data[2][0] = Bz;
//            RLS_in.p2Data[3][0] = 1;
//            RLS_out.p2Data[0][0] = (Bx * Bx) + (By * By) + (Bz * Bz);
//            Matrix_vTranspose_nsame(&RLS_in, &_temp_T);
//            Matrix_vmult_nsame(&_temp_T, &RLS_theta, &_temp_M);
//            Matrix_vsub(&RLS_out, &_temp_M, &_temp_AS);
//            float32_t err = _temp_AS.p2Data[0][0];
//            Matrix_vmult_nsame(&_temp_T, &RLS_P, &_temp_M);
//            Matrix_vmult_nsame(&_temp_M, &RLS_in, &_temp_M2);
//            Matrix_vRoundingadd(&_temp_M2, RLS_lambda);
//            Matrix_vmult_nsame(&RLS_P, &RLS_in, &_temp_M);
//            Matrix_vscale(&_temp_M, (1.0f / _temp_M2.p2Data[0][0]));
//            Matrix_vMove(&_temp_M, &RLS_gain);
//            Matrix_vmult_nsame(&RLS_gain, &_temp_T, &_temp_M);
//            Matrix_vmult_nsame(&_temp_M, &RLS_P, &_temp_M2);
//            Matrix_vsub(&RLS_P, &_temp_M2, &RLS_P);
//            Matrix_vscale(&RLS_P, (1.0f / RLS_lambda));
//            Matrix_vCopy(&RLS_gain, &_temp_AS);
//            Matrix_vscale(&_temp_AS, err);
//            Matrix_vsub(&RLS_theta, &_temp_AS, &RLS_theta);
//
//            Matrix_vGetDiagonalEntries(&RLS_P, &P_check);
//            Matrix_vTranspose_nsame(&P_check, &_temp_T);
//            Matrix_vmult_nsame(&_temp_T, &P_check, &_temp_M);
//            P_check_num = _temp_M.p2Data[0][0];
//            Matrix_vSetMatrixInvalid(&P_check);
//            Matrix_vSetMatrixInvalid(&_temp_T);
//            Matrix_vSetMatrixInvalid(&_temp_M);
//            Matrix_vSetMatrixInvalid(&_temp_M2);
//            Matrix_vSetMatrixInvalid(&_temp_AS);
//        }

//        IMU.HARD_IRON_BIAS.p2Data[0][0] = RLS_theta.p2Data[0][0] / 2.0f;
//        IMU.HARD_IRON_BIAS.p2Data[1][0] = RLS_theta.p2Data[1][0] / 2.0f;
//        IMU.HARD_IRON_BIAS.p2Data[2][0] = RLS_theta.p2Data[2][0] / 2.0f;
        //rotate and zero drift
//        imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);


        //加速度计低通滤波
//        accel low-pass filter
//        accel_fliter_1[0] = accel_fliter_2[0];
//        accel_fliter_2[0] = accel_fliter_3[0];
//
//        accel_fliter_3[0] =
//                accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];
//
//        accel_fliter_1[1] = accel_fliter_2[1];
//        accel_fliter_2[1] = accel_fliter_3[1];
//
//        accel_fliter_3[1] =
//                accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];
//
//        accel_fliter_1[2] = accel_fliter_2[2];
//        accel_fliter_2[2] = accel_fliter_3[2];
//
//        accel_fliter_3[2] =
//                accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];


//        AHRS_update(INS_quat, timing_time, bmi088_real_data.gyro, bmi088_real_data.accel, ist8310_real_data.mag);
//        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET,
//                  INS_angle + INS_ROLL_ADDRESS_OFFSET);
        get_angle(UKF_IMU.X_Est.arm_matrix.pData, INS_angle_ukf + INS_YAW_ADDRESS_OFFSET,
                  INS_angle_ukf + INS_PITCH_ADDRESS_OFFSET,
                  INS_angle_ukf + INS_ROLL_ADDRESS_OFFSET);


#if INCLUDE_uxTaskGetStackHighWaterMark
        INS_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(5));
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
//static void
//imu_cali_slove(float32_t gyro[3], float32_t accel[3], float32_t mag[3], bmi088_real_data_t *bmi088,
//               ist8310_real_data_t *ist8310) {
//    for (uint8_t i = 0; i < 3; i++) {
//        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] +
//                  bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
//        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] +
//                   bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
//        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] +
//                 ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
//    }
//}
 void
imu_cali_slove(float32_t gyro[3], float32_t accel[3], float32_t mag[3], bmi088_real_data_t *bmi088,
               ist8310_real_data_t *ist8310) {
    for (uint8_t i = 0; i < 3; i++) {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] +
                  bmi088->gyro[2] * gyro_scale_factor[i][2];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] +
                   bmi088->accel[2] * accel_scale_factor[i][2];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] +
                 ist8310->mag[2] * mag_scale_factor[i][2];
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
        ALL_PID(&imu_temp_pid, temp, 28.0f);
        if (imu_temp_pid.out < 0.0f) {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t) imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    } else {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp < 28) {
            temp_constant_time++;
            if (temp_constant_time > 200) {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
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

    gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
    gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
    gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
    (*offset_time_count)++;
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
        gyro_offset[0] = gyro_cali_offset[0];
        gyro_offset[1] = gyro_cali_offset[1];
        gyro_offset[2] = gyro_cali_offset[2];
    }
    gyro_offset_calc(gyro_offset, INS_gyro, time_count);

    cali_offset[0] = gyro_offset[0];
    cali_offset[1] = gyro_offset[1];
    cali_offset[2] = gyro_offset[2];
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
void INS_set_cali_gyro(float32_t cali_scale[3], float32_t cali_offset[3]) {
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
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
    return INS_gyro;
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
    return INS_accel;
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
    return INS_mag;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT1_ACCEL_Pin) {
        detect_hook(BOARD_ACCEL_TOE);
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == INT1_GYRO_Pin) {
        detect_hook(BOARD_GYRO_TOE);
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if (imu_start_dma_flag) {
            imu_cmd_spi_dma();
        }
    } else if (GPIO_Pin == DRDY_IST8310_Pin) {
        detect_hook(BOARD_MAG_TOE);
        mag_update_flag |= 1 << IMU_DR_SHFITS;
        ist8310_IIC_read_muli_reg(0x03, mag_dma_rx_buf, 6);
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
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void) {
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    //开启陀螺仪的DMA传输
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) gyro_dma_tx_buf, (uint32_t) gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    //开启加速度计的DMA传输
    if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) accel_dma_tx_buf, (uint32_t) accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }


    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS))) {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

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
        if (gyro_update_flag & (1 << IMU_SPI_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

        }

        //accel read over
        //加速度计读取完毕
        if (accel_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS)) {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

