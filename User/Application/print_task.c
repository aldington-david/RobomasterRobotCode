/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       print_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "print_task.h"
#include "global_control_define.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "SEGGER_RTT.h"
#include "referee_task.h"
#include "remote_control.h"
#include "printf.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "INS_task.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "shoot.h"
#include "servo_task.h"
#include "vision_task.h"
#include "calibrate_task.h"
#include "chassis_task.h"
#include "matlab_sync_task.h"
#include "PC_receive_task.h"
#include "led_flow_task.h"
#include "gimbal_behaviour.h"
#include "DWT.h"
#include "arm_math.h"
#include "matrix.h"


#if PRINTF_MODE == RTT_MODE
#define LOG(format, args...)  SEGGER_RTT_printf(0, "[%s:%d] "format, __FILE__, __LINE__, ##args)
#define printf(format, args...)  SEGGER_RTT_printf(0, format, ##args)
#endif

#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t print_task_stack;
#endif

static uint8_t print_buf[256];
static uint8_t read_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_print_local;
float32_t bias_angle_test = 0.0;
float32_t add_angle_test = 0.0;
int8_t imu_temp = 0;
float32_t sp_err = 0.0;
float32_t re_err = 0.0;
float32_t kalman_test = 0.0;
char switch_test = 0;
uint32_t id_test = 0;

float32_t Dout1_test = 0;
float32_t Dout2_test = 0;
float32_t KF_Dout1_test = 0;
float32_t KF_Dout2_test = 0;

float32_t tracer1 = 0;

float32_t vision_pitch_probe = 0;
float32_t vision_yaw_probe = 0;

float32_t zero_line = 0;
char vision_or_probe[128] = {0};

float32_t pid_out_probe = 0;
float32_t pid_pout_probe = 0;
float32_t pid_iout_probe = 0;
float32_t pid_dout_probe = 0;

//for_test
arm_matrix_instance_f32 testmatrix1;
arm_matrix_instance_f32 testmatrix2;
matrix_f32_t mymatrix1;
float matrixdata1[9] = {1,2,3,4,5,6,7,8,9};
float matrixdata2[9] = {0};
void print_task(void const *argument) {
    if (PRINTF_MODE == USB_MODE) {
        error_list_print_local = get_error_list_point();
        vTaskDelay(pdMS_TO_TICKS(500));
        TickType_t LoopStartTime;
        while (1) {
            DWT_update_task_time_us(&global_task_time.tim_print_task);
            LoopStartTime = xTaskGetTickCount();
            usb_printf(
                    "******************************\r\n\
voltage percentage:%d%% \r\n\
DBUS:%s\r\n\
chassis motor1:%s\r\n\
chassis motor2:%s\r\n\
chassis motor3:%s\r\n\
chassis motor4:%s\r\n\
yaw motor:%s\r\n\
pitch motor:%s\r\n\
trigger motor:%s\r\n\
gyro sensor:%s\r\n\
accel sensor:%s\r\n\
mag sensor:%s\r\n\
referee usart:%s\r\n\
******************************\r\n",
                    get_battery_percentage(),
                    status[error_list_print_local[DBUS_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR1_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR2_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR3_TOE].error_exist],
                    status[error_list_print_local[CHASSIS_MOTOR4_TOE].error_exist],
                    status[error_list_print_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
                    status[error_list_print_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
                    status[error_list_print_local[TRIGGER_MOTOR_TOE].error_exist],
                    status[error_list_print_local[BOARD_GYRO_TOE].error_exist],
                    status[error_list_print_local[BOARD_ACCEL_TOE].error_exist],
                    status[error_list_print_local[BOARD_MAG_TOE].error_exist],
                    status[error_list_print_local[REFEREE_RX_TOE].error_exist]);
            vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(50));
        }
    }
    if (PRINTF_MODE == RTT_MODE) {
        error_list_print_local = get_error_list_point();
        vTaskDelay(pdMS_TO_TICKS(500));
        //for_test
        Matrix_data_creat(&mymatrix1,3,3,matrixdata1,NoInitMatZero);
//        arm_mat_init_f32(&testmatrix1,3,3,matrixdata1);
//        arm_mat_init_f32(&testmatrix2,3,3,matrixdata2);
        for(int8_t i =0;i<9;i++){
            SEGGER_RTT_printf(0,"1[%d] = %f\r\n", i, matrixdata1[i]);
        }
        for(int8_t i =0;i<3;i++){
            for(int8_t j =0;j<3;j++){
            SEGGER_RTT_printf(0,"my1[%d][%d] = %f\r\n", i,j, mymatrix1.p2Data[i][j]);
            }
        }
        for(int8_t i =0;i<3;i++){
                SEGGER_RTT_printf(0,"my1[%d] = %f\r\n", i, mymatrix1.arm_matrix.pData[i]);
        }
//        arm_mat_trans_f32(&testmatrix1,&testmatrix2);
//        for(int8_t i =0;i<9;i++){
//            SEGGER_RTT_printf(0,"2[%d] = %f\r\n", i, matrixdata2[i]);
//        }
        TickType_t LoopStartTime;
        while (1) {
            DWT_update_task_time_us(&global_task_time.tim_print_task);
            LoopStartTime = xTaskGetTickCount();
//            SEGGER_RTT_printf(0,"testfunction\n");
            /***********************打印数据 Start *****************************/
            //执行时间
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "test=%d\r\ncali=%d\r\ndetect=%d\r\nchassis=%d\r\ngimbal=%d\r\nINS=%d\r\nvision_rx=%d\r\nservo=%d\r\nreferee_rx=%d\r\nreferee_tx=%d\r\nvision_tx=%d\r\nmatlab=%d\r\nprint=%d\r\nPC_receive=%d\r\nbattery_voltage=%d\r\nled_RGB_flow=%d\r\n",
//                    global_task_time.tim_test_task.time, global_task_time.tim_calibrate_task.time,
//                    global_task_time.tim_detect_task.time, global_task_time.tim_chassis_task.time,
//                    global_task_time.tim_gimbal_task.time, global_task_time.tim_INS_task.time,
//                    global_task_time.tim_vision_rx_task.time, global_task_time.tim_servo_task.time,
//                    global_task_time.tim_referee_rx_task.time, global_task_time.tim_referee_tx_task.time,
//                    global_task_time.tim_vision_tx_task.time, global_task_time.tim_matlab_sync_task.time,
//                    global_task_time.tim_print_task.time, global_task_time.tim_PC_receive_task.time,
//                    global_task_time.tim_battery_voltage_task.time, global_task_time.tim_led_RGB_flow_task.time);
//            SEGGER_RTT_WriteString(0, print_buf);
            //裁判系统
            //裁判系统限速
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "power=%fW,bullet_speed=%f，robot_id=%d\r\n",
//                    global_judge_info.PowerHeatData.chassis_power,
//                    global_judge_info.ShootData.bullet_speed,
//                    global_judge_info.GameRobotStatus.robot_id
//                    );
//            SEGGER_RTT_WriteString(0, print_buf);


            //传感器
            //mag
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "mag_x=%f,mag_y=%f,mag_z=%f,\r\n",
//                    ist8310_real_data.mag[0],
//                    ist8310_real_data.mag[1],
//                    ist8310_real_data.mag[2]);
//            SEGGER_RTT_WriteString(0, print_buf);

            //IMU数据
//            SEGGER_RTT_SetTerminal(5);
//            sprintf(print_buf, "imu_tmp=%f,YAW=%f,PITCH=%f,ROLL=%f\r\n",
//                    bmi088_real_data.temp,
//                    INS_angle[0],
//                    INS_angle[1],
//                    INS_angle[2]);
//            SEGGER_RTT_WriteString(0, print_buf);


            //视觉

//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "vision_yaw=%f,vision_pitch=%f,fps=%d\r\n",
//                    global_vision_info.vision_control.yaw_angle,
//                    global_vision_info.vision_control.pitch_angle,
//                    global_vision_info.vision_control.fps);
//            SEGGER_RTT_WriteString(0, print_buf);
//
//            SEGGER_RTT_SetTerminal(2);
//            sprintf(print_buf,
//                    "p_dead_sen=%f,y_dead_sen=%f,p_contr_sen=%f,y_contr_sen=%f,p_lpf=%f,y_lpf=%f\r\n",
//                    &vision_pitch_angle_deadband_sen,
//                    &vision_yaw_angle_deadband_sen,
//                    &vision_pitch_control_sen,
//                    &vision_yaw_control_sen,
//                    &vision_pitch_lpf_factor,
//                    &vision_yaw_control_lpf_factor);
//            SEGGER_RTT_WriteString(0, print_buf);
//
//            RTT_PrintWave(3,
//                          &zero_line,
//                          &global_vision_info.vision_control.yaw_angle,
//                          &global_vision_info.vision_control.pitch_angle);

//            SEGGER_RTT_SetTerminal(2);
//            sprintf(print_buf,
//                    "vision_yaw=%f,vision_pitch=%f\r\n",
//                    vision_yaw_probe,
//                    vision_pitch_probe);
//            SEGGER_RTT_WriteString(0, print_buf);
//
//            SEGGER_RTT_SetTerminal(3);
//            sprintf(print_buf,
//                    "now_yaw=%f,now_pitch=%f\r\n",
//                    gimbal_control.gimbal_yaw_motor.relative_angle,
//                    gimbal_control.gimbal_pitch_motor.relative_angle);
//            SEGGER_RTT_WriteString(0, print_buf);

//缓冲区
//            SEGGER_RTT_SetTerminal(4);
//            sprintf(print_buf,
//                    "vision_info=%s\r\n",
//                    vision_or_probe);
//            SEGGER_RTT_WriteString(0, print_buf);
//接收进度调试
//            SEGGER_RTT_SetTerminal(2);
//            sprintf(print_buf,
//                    "vision_index=%d\r\n",
//                    global_vision_info.pack_info->index);
//            SEGGER_RTT_WriteString(0, print_buf);

//            //舵机pwm
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "bullet_box_pwm=%d\r\n",
//                    bullet_box_pwm);
//            SEGGER_RTT_WriteString(0, print_buf);
            //鼠标控制
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "mouse_x=%d,mouse_y=%d,mouse_z=%d,mouse_l=%d,mouse_r=%d\r\nkey=%d\r\n,",
//                    rc_ctrl.mouse.x,
//                    rc_ctrl.mouse.y,
//                    rc_ctrl.mouse.z,
//                    rc_ctrl.mouse.press_l,
//                    rc_ctrl.mouse.press_r,
//                    rc_ctrl.key.v);
//            SEGGER_RTT_WriteString(0, print_buf);

//            //功率显示
//            RTT_PrintWave(1,&global_judge_info.PowerHeatData.chassis_power);
//            //CAN_id
//            SEGGER_RTT_SetTerminal(9);
//            sprintf(print_buf, "can2_id=%d\r\n",
//                    id_test);
//            SEGGER_RTT_WriteString(0, print_buf);

//            cali校准
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "Yaw_now_max_ang=%f,Yaw_now_min_ang=%f,Yaw_now_ang=%f,Yaw_now_ecd=%d,Yaw_total_ecd=%d,Yaw_turncount=%d,Yaw_offset_ecd=%d,Yaw_cl_max_ecd=%d,Yaw_cl_min_ecd=%d\r\n",
//                    gimbal_control.gimbal_yaw_motor.max_relative_angle,
//                    gimbal_control.gimbal_yaw_motor.min_relative_angle,
//                    gimbal_control.gimbal_yaw_motor.relative_angle,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->turnCount,
//                    gimbal_control.gimbal_yaw_motor.offset_ecd,
//                    gimbal_control.gimbal_cali.max_yaw_ecd,
//                    gimbal_control.gimbal_cali.min_yaw_ecd);
//            SEGGER_RTT_WriteString(0, print_buf);
//
//            SEGGER_RTT_SetTerminal(2);
//            sprintf(print_buf,
//                    "Pitch_max_ang=%f,Pitch_min_ang=%f,Pitch_now_ang=%f,Pitch_now_ecd=%d,Pitch_total_ecd=%d,Pitch_turncount=%d,Pitch_offset_ecd=%d,Pitch_cl_max_ecd=%d,Pitch_cl_min_ecd=%d\r\n",
//                    gimbal_control.gimbal_pitch_motor.max_relative_angle,
//                    gimbal_control.gimbal_pitch_motor.min_relative_angle,
//                    gimbal_control.gimbal_pitch_motor.relative_angle,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->total_ecd,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->turnCount,
//                    gimbal_control.gimbal_pitch_motor.offset_ecd,
//                    gimbal_control.gimbal_cali.max_pitch_ecd,
//                    gimbal_control.gimbal_cali.min_pitch_ecd);
//            SEGGER_RTT_WriteString(0, print_buf);

            //拨盘pid
//            拨盘数据
//            SEGGER_RTT_SetTerminal(8);
//            sprintf(print_buf,
//                    "shoot_mode=%d,pwm1=%d,pwm2=%d,ecd=%d,ecd_count=%d,angle=%f,t_sp_set=%f,mv_flag=%d,switch_test=%x\r\n",
//                    shoot_control.shoot_mode,
//                    shoot_control.fric_pwm1,
//                    shoot_control.fric_pwm2,
//                    shoot_control.shoot_motor_measure->ecd,
//                    shoot_control.shoot_motor_measure->turnCount,
//                    shoot_control.angle,
//                    shoot_control.trigger_speed_set,
//                    shoot_control.move_flag,
//                    switch_test);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //拨盘pid
//            SEGGER_RTT_SetTerminal(9);
//            sprintf(print_buf,
//                    "p=%f,i=%f,d=%f\r\n",
//                    shoot_control.trigger_motor_speed_pid.Kp,
//                    shoot_control.trigger_motor_speed_pid.Ki,
//                    shoot_control.trigger_motor_speed_pid.Kd);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //摩擦轮pid
//            SEGGER_RTT_SetTerminal(2);
//            sprintf(print_buf,
//                    "tr_p=%f,tr_i=%f,tr_d=%f,fr1_p=%f,fr1_i=%f,fr1_d=%f,fr1_IS=%f,fr2_p=%f,fr2_i=%f,fr2_d=%f,fr2_IS=%f,err=%f\r\n",
//                    shoot_control.trigger_motor_speed_pid.Kp,
//                    shoot_control.trigger_motor_speed_pid.Ki,
//                    shoot_control.trigger_motor_speed_pid.Kd,
//                    shoot_control.fric1_motor_pid.Kp,
//                    shoot_control.fric1_motor_pid.Ki,
//                    shoot_control.fric1_motor_pid.Kd,
//                    shoot_control.fric1_motor_pid.Integral_Separation,
//                    shoot_control.fric2_motor_pid.Kp,
//                    shoot_control.fric2_motor_pid.Ki,
//                    shoot_control.fric2_motor_pid.Kd,
//                    shoot_control.fric2_motor_pid.Integral_Separation,
//                    err_test);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //摩擦轮pwm
//            SEGGER_RTT_SetTerminal(10);
//            sprintf(print_buf,
//                    "pwm1=%f,pwm2=%f,add=%d\r\n",
//                    shoot_control.fric1_ramp.max_value,
//                    shoot_control.fric2_ramp.max_value,
//                    shoot_control.pwm);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //摩擦轮电流
//            SEGGER_RTT_SetTerminal(9);
//            sprintf(print_buf,
//                    "fric1_current_set=%d,fric2_current_set=%d,fric1_current=%d,fric2_current=%d\r\n",
//                    gimbal_control.fric1_give_current,
//                    gimbal_control.fric2_give_current,
//                    shoot_control.fric1_motor_measure->given_current,
//                    shoot_control.fric2_motor_measure->given_current);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //拨盘波形显示
//            RTT_PrintWave(4,
//                          &shoot_control.speed_set,
//                          &shoot_control.speed,
//                          &shoot_control.angle_set,
//                          &shoot_control.angle);
//            //摩擦轮波形显示
//            RTT_PrintWave(3,
//                          &shoot_control.fric_all_speed,
//                          &shoot_control.fric1_speed,
//                          &shoot_control.fric2_speed);
            //relative_mode

////            Pitch
//            SEGGER_RTT_SetTerminal(2);
//            //pid
//            sprintf(print_buf,
//                    "LpfFactor=%f,rekp=%f,reki=%f,rekd=%f,spkp=%f,spki=%f,spkd=%f,spIS=%f,err=%f,maxiout=%f,maxout=%f\r\n",
//                    gimbal_control.gimbal_pitch_motor.LpfFactor,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.kp,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.ki,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.kd,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Kp,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Ki,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Kd,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Integral_Separation,
//                    err_test,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.max_iout,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.max_out);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //卡尔曼系数
//            SEGGER_RTT_SetTerminal(3);
//            sprintf(print_buf,
//                    "err_kalman_MR=%f,err_kalman_SQ=%f,pid_kalman_MR=%f,pid_kalman_SQ=%f\r\n",
//                    gimbal_control.gimbal_pitch_motor.Cloud_Motor_Current_Kalman_Filter.R,
//                    gimbal_control.gimbal_pitch_motor.Cloud_Motor_Current_Kalman_Filter.Q,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.Cloud_OCKalman.R,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.Cloud_OCKalman.Q);
//            SEGGER_RTT_WriteString(0, print_buf);
            //rad角度数据
//            SEGGER_RTT_SetTerminal(4);
//            sprintf(print_buf, "setangle=%f,maxangle=%f,nowangle=%f,minangle=%f\r\n",
//                    gimbal_control.gimbal_pitch_motor.relative_angle_set,
//                    gimbal_control.gimbal_pitch_motor.max_relative_angle,
//                    gimbal_control.gimbal_pitch_motor.relative_angle,
//                    gimbal_control.gimbal_pitch_motor.min_relative_angle);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //电流数据
//            SEGGER_RTT_SetTerminal(5);
//            sprintf(print_buf, "current=%f,motor_gyro_set=%f,given_current=%d\r\n",
//                    gimbal_control.gimbal_pitch_motor.current_set,
//                    gimbal_control.gimbal_pitch_motor.motor_gyro_set,
//                    gimbal_control.gimbal_pitch_motor.given_current);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //波形显示
//            RTT_PrintWave(5,
//                          &gimbal_control.gimbal_pitch_motor.relative_angle_set,
//                          &gimbal_control.gimbal_pitch_motor.relative_angle,
//                          &gimbal_control.gimbal_pitch_motor.motor_gyro_set,
//                          &gimbal_control.gimbal_pitch_motor.motor_gyro,
//                          &gimbal_control.gimbal_pitch_motor.motor_speed);

//            //YAW

//            pid
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "LpfFactor=%f,rekp=%f,reki=%f,rekd=%f,spkp=%f,spki=%f,spkd=%f,spIS=%f,maxiout=%f,maxout=%f\r\n",
//                    gimbal_control.gimbal_yaw_motor.LpfFactor,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Kp,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Ki,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Kd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Integral_Separation,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.max_iout,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.max_out);
//            SEGGER_RTT_WriteString(0, print_buf);
////            //PID功能控制
//            SEGGER_RTT_SetTerminal(2);
//            sprintf(print_buf,
//                    "re_D_First_on=%d,re_D_First_Ratio=%f,sp_D_First_on=%d,sp_D_First_Ratio=%f,re_NF_D_on=%d,re_D_Alpha=%f,sp_NF_D_on=%d,sp_D_Alpha=%f\r\n",
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.D_First,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.D_Filter_Ratio,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_First,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_Filter_Ratio,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.NF_D,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.D_Alpha,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.NF_D,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_Alpha);
//            SEGGER_RTT_WriteString(0, print_buf);
//
//            SEGGER_RTT_SetTerminal(3);
//            sprintf(print_buf,
//                    "sp_err=%f,sp_Var_I_on=%d,sp_I_ratio=%f,sp_I_Down=%f,sp_I_Up=%f,re_err=%f,re_Var_I_on=%d,re_I_ratio=%f,re_I_Down=%f,I_Up=%f\r\n",
//                    sp_err,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.I_ratio,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I_Down,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I_UP,
//                    re_err,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Variable_I,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.I_ratio,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Variable_I_Down,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Variable_I_UP);
//            SEGGER_RTT_WriteString(0, print_buf);
////            //卡尔曼系数
//            SEGGER_RTT_SetTerminal(4);
//            sprintf(print_buf,
//                    "err_kalman_MR=%f,err_kalman_SQ=%f\r\n",
//                    gimbal_control.gimbal_yaw_motor.Cloud_Motor_Current_Kalman_Filter.R,
//                    gimbal_control.gimbal_yaw_motor.Cloud_Motor_Current_Kalman_Filter.Q);
//            SEGGER_RTT_WriteString(0, print_buf);
//            rad角度数据
//            SEGGER_RTT_SetTerminal(5);
//            sprintf(print_buf, "setangle=%f,maxangle=%f,nowangle=%f,minangle=%f,total_ecd=%d,offset_ecd=%d\r\n",
//                    gimbal_control.gimbal_yaw_motor.relative_angle_set,
//                    gimbal_control.gimbal_yaw_motor.max_relative_angle,
//                    gimbal_control.gimbal_yaw_motor.relative_angle,
//                    gimbal_control.gimbal_yaw_motor.min_relative_angle,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd,
//                    gimbal_control.gimbal_yaw_motor.offset_ecd);
//            SEGGER_RTT_WriteString(0, print_buf);
            //电流数据
//            SEGGER_RTT_SetTerminal(6);
////            sprintf(print_buf, "current=%f,motor_gyro_set=%f,lms=%f,given_current=%d\r\n",
////                    gimbal_control.gimbal_yaw_motor.current_set,
////                    gimbal_control.gimbal_yaw_motor.motor_gyro_set,
////                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.PID_lms.outputF32,
////                    gimbal_control.gimbal_yaw_motor.given_current);
////            SEGGER_RTT_WriteString(0, print_buf);
            //遥控器数据
//            SEGGER_RTT_SetTerminal(7);
//            sprintf(print_buf, "ch1=%d,ch2=%d,ch3=%d,ch4=%d,ch5=%d\r\n",
//                    rc_ctrl.rc.ch[0],
//                    rc_ctrl.rc.ch[1],
//                    rc_ctrl.rc.ch[2],
//                    rc_ctrl.rc.ch[3],
//                    rc_ctrl.rc.ch[4]);
//            SEGGER_RTT_WriteString(0, print_buf);

//            波形显示
            //pid累计数据
//            RTT_PrintWave(4,
//                          &pid_out_probe,
//                          &pid_pout_probe,
//                          &pid_iout_probe,
//                          &pid_dout_probe);
//            RTT_PrintWave(6,
//                          &gimbal_control.gimbal_yaw_motor.relative_angle_set,
//                          &gimbal_control.gimbal_yaw_motor.relative_angle,
//                          &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.PID_lms.outputF32,
//                          &gimbal_control.gimbal_yaw_motor.motor_gyro_set,
//                          &gimbal_control.gimbal_yaw_motor.motor_gyro,
//                          &gimbal_control.gimbal_yaw_motor.motor_speed);
//            RTT_PrintWave_np(4,
//                          gimbal_control.gimbal_yaw_motor.relative_angle_set,
//                          gimbal_control.gimbal_yaw_motor.relative_angle,
//                          gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.PID_lms.outputF32,
//                          (float)rc_ctrl.rc.ch[2]);
//遥控器波形打印
//            RTT_PrintWave_np(5,
//                             (float) rc_ctrl.rc.ch[0],
//                             (float) rc_ctrl.rc.ch[1],
//                             (float) rc_ctrl.rc.ch[2],
//                             (float) rc_ctrl.rc.ch[3],
//                             (float) rc_ctrl.rc.ch[4]);

//
////            RTT_PrintWave(2,
////                          &gimbal_control.gimbal_yaw_motor.current_set,
////                          &kalman_test);
//
////            RTT_PrintWave(4,
////                          &Dout1_test,
////                          &KF_Dout1_test,
////                          &Dout2_test,
////                          &KF_Dout2_test);
//
////            RTT_PrintWave(1,
////                          &tracer1);



            //abosolute_mode
            //angle_data
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf, "bias_angle=%f,add_angle=%f\r\n", bias_angle_test, add_angle_test);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //pid
//            SEGGER_RTT_SetTerminal(2);
//            sprintf(print_buf, "rekp=%f,reki=%f,rekd=%f,spkp=%f,spki=%f,spkd=%f\r\n",
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.kp,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.ki,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.kd,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Kp,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Ki,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid.Kd);
//            SEGGER_RTT_WriteString(0, print_buf);
            //pitchrad角度数据校准
//            SEGGER_RTT_SetTerminal(3);
//            sprintf(print_buf, "ab_setangle=%f,now_ab_angle=%f,maxangle=%f,relative_angle_set=%f,now_rel_angle=%f,minangle=%f\r\n",
//                    gimbal_control.gimbal_pitch_motor.absolute_angle_set,
//                    gimbal_control.gimbal_pitch_motor.absolute_angle,
//                    gimbal_control.gimbal_pitch_motor.max_relative_angle,
//                    gimbal_control.gimbal_pitch_motor.relative_angle_set,
//                    gimbal_control.gimbal_pitch_motor.relative_angle,
//                    gimbal_control.gimbal_pitch_motor.min_relative_angle);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //电流数据
//            SEGGER_RTT_SetTerminal(4);
//            sprintf(print_buf, "current=%f,motor_gyro_set=%f,given_current=%d\r\n",
//                    gimbal_control.gimbal_pitch_motor.current_set,
//                    gimbal_control.gimbal_pitch_motor.motor_gyro_set,
//                    gimbal_control.gimbal_pitch_motor.given_current);
//            SEGGER_RTT_WriteString(0, print_buf);
            //波形显示
//            RTT_PrintWave(4,
//                          &gimbal_control.gimbal_pitch_motor.absolute_angle_set,
//                          &gimbal_control.gimbal_pitch_motor.absolute_angle,
//                          &gimbal_control.gimbal_pitch_motor.motor_gyro_set,
//                          &gimbal_control.gimbal_pitch_motor.motor_gyro);
            //FreeRTOS 任务栈打印
//            SEGGER_RTT_SetTerminal(9);
//            sprintf(print_buf,
//                    "******************************\r\nAllStack_min_size=%d\r\ncalibrate_task=%d\r\ndetect_task=%d\r\nchassis_task=%d\r\ngimbal_task=%d\r\nINS_task=%d\r\nvision_rx_task=%d\r\nservo_task=%d\r\nreferee_rx_task=%d\r\nUSART6TX_active_task=%d\r\nUSART1TX_active_task=%d\r\nreferee_tx_task=%d\r\nvision_tx_task=%d\r\nmatlab_sync_task=%d\r\nprint_task=%d\r\nPC_receive_task=%d\r\nbattery_voltage_task=%d\r\nled_RGB_flow_task=%d\r\n",
//                    (int)xPortGetMinimumEverFreeHeapSize(),
//                    (int)get_stack_of_calibrate_task(),
//                    (int)get_stack_of_detect_task(),
//                    (int)get_stack_of_chassis_task(),
//                    (int)get_stack_of_gimbal_task(),
//                    (int)get_stack_of_INS_task(),
//                    (int)get_stack_of_vision_rx_task(),
//                    (int)get_stack_of_servo_task(),
//                    (int)get_stack_of_referee_rx_task(),
//                    (int)get_stack_of_USART6TX_active_task(),
//                    (int)get_stack_of_USART1TX_active_task(),
//                    (int)get_stack_of_referee_tx_task(),
//                    (int)get_stack_of_vision_tx_task(),
//                    (int)get_stack_of_matlab_sync_task(),
//                    (int)get_stack_of_print_task(),
//                    (int)get_stack_of_PC_receive_task(),
//                    (int)get_stack_of_battery_voltage_task(),
//                    (int)get_stack_of_led_RGB_flow_task());
//            SEGGER_RTT_WriteString(0, print_buf);
            /***********************打印数据 End *****************************/
//离线检测

//            printf(
//                    "******************************\r\n\
//voltage percentage:%d%% \r\n\
//DBUS:%s\r\n\
//chassis motor1:%s\r\n\
//chassis motor2:%s\r\n\
//chassis motor3:%s\r\n\
//chassis motor4:%s\r\n\
//yaw motor:%s\r\n\
//pitch motor:%s\r\n\
//trigger motor:%s\r\n\
//gyro sensor:%s\r\n\
//accel sensor:%s\r\n\
//mag sensor:%s\r\n\
//referee rx usart:%s\r\n\
//******************************\r\n",
//                    get_battery_percentage(),
//                    status[error_list_print_local[DBUS_TOE].error_exist],
//                    status[error_list_print_local[CHASSIS_MOTOR1_TOE].error_exist],
//                    status[error_list_print_local[CHASSIS_MOTOR2_TOE].error_exist],
//                    status[error_list_print_local[CHASSIS_MOTOR3_TOE].error_exist],
//                    status[error_list_print_local[CHASSIS_MOTOR4_TOE].error_exist],
//                    status[error_list_print_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
//                    status[error_list_print_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
//                    status[error_list_print_local[TRIGGER_MOTOR_TOE].error_exist],
//                    status[error_list_print_local[BOARD_GYRO_TOE].error_exist],
//                    status[error_list_print_local[BOARD_ACCEL_TOE].error_exist],
//                    status[error_list_print_local[BOARD_MAG_TOE].error_exist],
//                    status[error_list_print_local[REFEREE_RX_TOE].error_exist]);

//控制输出

//            printf(
//                    "*******Variable Start*********\r\n\
//$param:Channal1=%d;\r\n\
//$param:Channal2=%d;\r\n\
//$param:Channal3=%d;\r\n\
//$param:Channal4=%d;\r\n\
//$param:Channal5=%d;\r\n\
//$param:SwitchRight=%d;\r\n\
//$param:SwitchLeft=%d;\r\n\
//$param:MouseX=%d;\r\n\
//$param:MouseY=%d;\r\n\
//$param:MouseZ=%d;\r\n\
//$param:MouseLeft=%d;\r\n\
//$param:MouseRight=%d;\r\n\
//$param:Key=%d;\r\n\
//*******Variable End***********\r\n",
//                    rc_ctrl.rc.ch[0],
//                    rc_ctrl.rc.ch[1],
//                    rc_ctrl.rc.ch[2],
//                    rc_ctrl.rc.ch[3],
//                    rc_ctrl.rc.ch[4],
//                    rc_ctrl.rc.s[0],
//                    rc_ctrl.rc.s[1],
//                    rc_ctrl.mouse.x,
//                    rc_ctrl.mouse.y,
//                    rc_ctrl.mouse.z,
//                    rc_ctrl.mouse.press_l,
//                    rc_ctrl.mouse.press_r,
//                    rc_ctrl.key.v);

#if INCLUDE_uxTaskGetStackHighWaterMark
            print_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
            vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(60));
        }
    }
}

/**
  * @brief          获取print_task栈大小
  * @param[in]      none
  * @retval         print_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_print_task(void) {
    return print_task_stack;
}

static void usb_printf(const char *fmt, ...) {
    static va_list ap;
    static uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *) print_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(print_buf, len);
}

/**
  * @brief          RTT波形打印，格式为富莱安H7-tool显示格式，参数为浮点数指针
  * @param[in]      num_args : 参数数目
  * @retval         none
  */
void RTT_PrintWave(int num_args, ...) {
    float32_t *param_point[num_args];
    int i;
    char buf[256];
    va_list arg;
    va_start(arg, num_args);
    int len = 0;
    for (i = 0; i < num_args; i++) {
        param_point[i] = va_arg(arg, float32_t *);

        if (i == (num_args - 1)) {
            len += sprintf((buf + len), "%.3f\r\n", *(float *) param_point[i]);
        } else {
            len += sprintf((buf + len), "%.3f,", *(float *) param_point[i]);
        }
    }
    va_end(arg);
    SEGGER_RTT_SetTerminal(0);
    SEGGER_RTT_WriteString(0, buf);
}

/**
  * @brief          RTT波形打印(整形用)，格式为富莱安H7-tool显示格式，参数为整形转浮点数非指针
  * @param[in]      num_args : 参数数目
  * @retval         none
  */
void RTT_PrintWave_np(int num_args, ...) {
    float32_t param_point[num_args];
    int i;
    char buf[256];
    va_list arg;
    va_start(arg, num_args);
    int len = 0;
    for (i = 0; i < num_args; i++) {
        param_point[i] = va_arg(arg, double);

        if (i == (num_args - 1)) {
            len += sprintf((buf + len), "%.3f\r\n", (float) param_point[i]);
        } else {
            len += sprintf((buf + len), "%.3f,", (float) param_point[i]);
        }
    }
    va_end(arg);
    SEGGER_RTT_SetTerminal(0);
    SEGGER_RTT_WriteString(0, buf);
}

/**
  * @brief          Timer7溢出中断调用打印，Timer7配置务必在STM32Cubemx调节，APB1 82Mhz,建议时间不小于60us
  * @param[in]      none
  * @retval         none
  */
void RTT_timer_trigger(void) {
//                RTT_PrintWave(3,
//                          &gimbal_control.gimbal_yaw_motor.motor_gyro_set,
//                          &gimbal_control.gimbal_yaw_motor.motor_gyro,
//                          &gimbal_control.gimbal_yaw_motor.motor_speed);

//                RTT_PrintWave_np(2,
//                              gimbal_control.gimbal_yaw_motor.relative_angle_set,
//                                 (float)gimbal_control.gimbal_rc_ctrl->rc.ch[YAW_CHANNEL]*YAW_RC_SEN);

//                RTT_PrintWave(2,
//                          &gimbal_control.gimbal_yaw_motor.motor_gyro_set,
//                          &gimbal_control.gimbal_yaw_motor.motor_speed);
//    RTT_PrintWave(2,
//                  &gimbal_control.gimbal_yaw_motor.relative_angle_set,
//                  &gimbal_control.gimbal_yaw_motor.relative_angle);

//    RTT_PrintWave(2,
//                  &kalman_test,
//                  &gimbal_control.gimbal_yaw_motor.motor_speed);

//    RTT_PrintWave(3,
//                  &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Pout,
//                  &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Iout,
//                  &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid.Dout);


//    RTT_PrintWave(2,
//                  &kalman_test,
//                  &gimbal_control.gimbal_yaw_motor.current_set);

//    RTT_PrintWave_np(1,
//                     (float) rc_ctrl.rc.ch[2]);
}

//for_test
//static void Print_RTT_ReadBuffer(void) {
//    unsigned NumBytes = sizeof(read_buf);
//    NumBytes = SEGGER_RTT_Read(0, &read_buf[0], NumBytes);
//    if (NumBytes) {
//        int i;
//        for (i = 0; i < NumBytes; i++)
//            printf("%c", read_buf[i]);
//        printf("\n");
//    }
//}

//not_use Now place in PC_receive_task
//static void Use_RTT_SetConfig(void *const variable) {
//    unsigned NumBytes = sizeof(read_buf);
//    NumBytes = SEGGER_RTT_Read(0, &read_buf[0], NumBytes);
//    if (NumBytes) {
//        char *rest;
//        int32_t var = strtol(&read_buf[0], &rest, 10);
//        memcpy(variable, &var, sizeof(var));
//    }
//}