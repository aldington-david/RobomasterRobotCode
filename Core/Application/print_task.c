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
#include "shoot.h"
#include "servo_task.h"
#include "vision_task.h"


#if PRINTF_MODE == RTT_MODE
#define LOG(format, args...)  SEGGER_RTT_printf(0, "[%s:%d] "format, __FILE__, __LINE__, ##args)
#define printf(format, args...)  SEGGER_RTT_printf(0, format, ##args)
#endif

#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"

static uint8_t print_buf[256];
static uint8_t read_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_print_local;
fp32 bias_angle_test = 0.0;
fp32 add_angle_test = 0.0;
int8_t imu_temp = 0;
fp32 sp_err = 0.0;
fp32 re_err = 0.0;
fp32 kalman_test = 0.0;
char switch_test = 0;
uint32_t id_test = 0;

fp32 Dout1_test = 0;
fp32 Dout2_test = 0;
fp32 KF_Dout1_test = 0;
fp32 KF_Dout2_test = 0;

fp32 tracer1 = 0;

fp32 vision_pitch_probe = 0;
fp32 vision_yaw_probe = 0;
//char vision_or_probe[128]={0};

void print_task(void const *argument) {
    if (PRINTF_MODE == USB_MODE) {
        MX_USB_DEVICE_Init();
        error_list_print_local = get_error_list_point();
        vTaskDelay(pdMS_TO_TICKS(500));
        TickType_t LoopStartTime;
        while (1) {
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
        MX_USB_DEVICE_Init();
        error_list_print_local = get_error_list_point();
        vTaskDelay(pdMS_TO_TICKS(500));
        TickType_t LoopStartTime;
        while (1) {
            LoopStartTime = xTaskGetTickCount();
//            SEGGER_RTT_printf(0,"testfunction\n");
            /***********************打印数据 Start *****************************/
            //视觉
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

//            SEGGER_RTT_SetTerminal(4);
//            sprintf(print_buf,
//                    "vision_info=%s\r\n",
//                    vision_or_probe);
//            SEGGER_RTT_WriteString(0, print_buf);
//
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
//            cali
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "Yaw_now_ecd=%d,Yaw_total_ecd=%d,Yaw_turncount=%d,Yaw_offset_ecd=%d,Yaw_max_ecd=%d,Yaw_min_ecd=%d,Pitch_now_ecd=%d,Pitch_total_ecd=%d,Pitch_turncount=%d,Pitch_offset_ecd=%d,Pitch_max_ecd=%d,Pitch_min_ecd=%d\r\n",
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->turnCount,
//                    gimbal_control.gimbal_yaw_motor.offset_ecd,
//                    gimbal_control.gimbal_cali.max_yaw_ecd,
//                    gimbal_control.gimbal_cali.min_yaw_ecd,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->turnCount,
//                    gimbal_control.gimbal_pitch_motor.offset_ecd,
//                    gimbal_control.gimbal_cali.max_pitch_ecd,
//                    gimbal_control.gimbal_cali.max_pitch_ecd);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //裁判系统射速
//            SEGGER_RTT_SetTerminal(1);
//            sprintf(print_buf,
//                    "power=%fW,bullet_speed=%f\r\n",
//                    global_judge_info.PowerHeatData.chassis_power,
//                    global_judge_info.ShootData.bullet_speed);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //拨盘pid
            //拨盘数据
//            SEGGER_RTT_SetTerminal(8);
//            sprintf(print_buf,
//                    "shoot_mode=%d,pwm1=%d,pwm2=%d,ecd=%d,ecd_count=%d,angle=%f,t_sp_set=%f,mv_flag=%d,switch_test=%c\r\n",
//                    shoot_control.shoot_mode,
//                    shoot_control.fric_pwm1,
//                    shoot_control.fric_pwm2,
//                    shoot_control.shoot_motor_measure->ecd,
//                    shoot_control.ecd_count,
//                    shoot_control.angle,
//                    shoot_control.trigger_speed_set,
//                    shoot_control.move_flag,
//                    switch_test);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //拨盘pid
//            SEGGER_RTT_SetTerminal(9);
//            sprintf(print_buf,
//                    "p=%f,i=%f,d=%f\r\n",
//                    shoot_control.trigger_motor_pid.Kp,
//                    shoot_control.trigger_motor_pid.Ki,
//                    shoot_control.trigger_motor_pid.Kd);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //摩擦轮pid
//            SEGGER_RTT_SetTerminal(2);
//            sprintf(print_buf,
//                    "tr_p=%f,tr_i=%f,tr_d=%f,fr1_p=%f,fr1_i=%f,fr1_d=%f,fr1_IS=%f,fr2_p=%f,fr2_i=%f,fr2_d=%f,fr2_IS=%f,err=%f\r\n",
//                    shoot_control.trigger_motor_pid.Kp,
//                    shoot_control.trigger_motor_pid.Ki,
//                    shoot_control.trigger_motor_pid.Kd,
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
//                    "fric1_current=%d,fric2_current=%d\r\n",
//                    gimbal_control.fric1_give_current,
//                    gimbal_control.fric2_give_current);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //拨盘波形显示
//            RTT_PrintWave(4,
//                          &shoot_control.speed_set,
//                          &shoot_control.speed,
//                          &shoot_control.set_angle,
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
//                    gimbal_control.gimbal_pitch_motor.Cloud_MotorAngle_Error_Kalman.R,
//                    gimbal_control.gimbal_pitch_motor.Cloud_MotorAngle_Error_Kalman.Q,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.Cloud_OCKalman.R,
//                    gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid.Cloud_OCKalman.Q);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //rad角度数据
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
//            SEGGER_RTT_SetTerminal(2);
//            //pid
//            sprintf(print_buf,
//                    "LpfFactor=%f,rekp=%f,reki=%f,rekd=%f,spkp=%f,spki=%f,spkd=%f,spIS=%f,maxiout=%f,maxout=%f\r\n",
//                    gimbal_control.gimbal_yaw_motor.LpfFactor,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Kp,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Ki,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Kd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Kp,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Ki,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Kd,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Integral_Separation,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.max_iout,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.max_out);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //PID功能控制
//            SEGGER_RTT_SetTerminal(3);
//            sprintf(print_buf,
//                    "re_D_First_on=%d,re_D_First_Ratio=%f,sp_D_First_on=%d,sp_D_First_Ratio=%f,re_NF_D_on=%d,re_D_Alpha=%f,sp_NF_D_on=%d,sp_D_Alpha=%f\r\n",
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.D_First,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.D_Filter_Ratio,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_First,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_Filter_Ratio,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.NF_D,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.D_Alpha,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.NF_D,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.D_Alpha);
//            SEGGER_RTT_WriteString(0, print_buf);
//            SEGGER_RTT_SetTerminal(4);
//            sprintf(print_buf,
//                    "sp_err=%f,sp_Var_I_on=%d,sp_I_ratio=%f,sp_I_Down=%f,sp_I_Up=%f,re_err=%f,re_Var_I_on=%d,re_I_ratio=%f,re_I_Down=%f,I_Up=%f\r\n",
//                    sp_err,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.I_ratio,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I_Down,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid.Variable_I_UP,
//                    re_err,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Variable_I,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.I_ratio,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Variable_I_Down,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.Variable_I_UP);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //卡尔曼系数
//            SEGGER_RTT_SetTerminal(5);
//            sprintf(print_buf,
//                    "err_kalman_MR=%f,err_kalman_SQ=%f\r\n",
//                    gimbal_control.gimbal_yaw_motor.Cloud_MotorAngle_Error_Kalman.R,
//                    gimbal_control.gimbal_yaw_motor.Cloud_MotorAngle_Error_Kalman.Q);
//            SEGGER_RTT_WriteString(0, print_buf);
//////            rad角度数据
////            SEGGER_RTT_SetTerminal(6);
////            sprintf(print_buf, "setangle=%f,maxangle=%f,nowangle=%f,minangle=%f\r\n",
////                    gimbal_control.gimbal_yaw_motor.relative_angle_set,
////                    gimbal_control.gimbal_yaw_motor.max_relative_angle,
////                    gimbal_control.gimbal_yaw_motor.relative_angle,
////                    gimbal_control.gimbal_yaw_motor.min_relative_angle);
//            SEGGER_RTT_WriteString(0, print_buf);
//            //电流数据
//            SEGGER_RTT_SetTerminal(7);
//            sprintf(print_buf, "current=%f,motor_gyro_set=%f,lms=%f,given_current=%d\r\n",
//                    gimbal_control.gimbal_yaw_motor.current_set,
//                    gimbal_control.gimbal_yaw_motor.motor_gyro_set,
//                    gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.PID_lms.outputF32,
//                    gimbal_control.gimbal_yaw_motor.given_current);
//            SEGGER_RTT_WriteString(0, print_buf);
////            //遥控器数据
////            SEGGER_RTT_SetTerminal(8);
////            sprintf(print_buf, "ch1=%d,ch2=%d,ch3=%d,ch4=%d,ch5=%d\r\n",
////                    rc_ctrl.rc.ch[0],
////                    rc_ctrl.rc.ch[1],
////                    rc_ctrl.rc.ch[2],
////                    rc_ctrl.rc.ch[3],
////                    rc_ctrl.rc.ch[4]);
////            SEGGER_RTT_WriteString(0, print_buf);
////            通道值监控
//            SEGGER_RTT_SetTerminal(9);
//            sprintf(print_buf, "ch1=%d,ch2=%d,ch3=%d,ch4=%d,ch5=%d\r\n",
//                    rc_ctrl.rc.ch[0],
//                    rc_ctrl.rc.ch[1],
//                    rc_ctrl.rc.ch[2],
//                    rc_ctrl.rc.ch[3],
//                    rc_ctrl.rc.ch[4]);
//            SEGGER_RTT_WriteString(0, print_buf);
//
////            波形显示
//            RTT_PrintWave(6,
//                          &gimbal_control.gimbal_yaw_motor.relative_angle_set,
//                          &gimbal_control.gimbal_yaw_motor.relative_angle,
//                          &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid_temp.PID_lms.outputF32,
//                          &gimbal_control.gimbal_yaw_motor.motor_gyro_set,
//                          &gimbal_control.gimbal_yaw_motor.motor_gyro,
//                          &gimbal_control.gimbal_yaw_motor.motor_speed);
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
//            //rad角度数据
//            SEGGER_RTT_SetTerminal(3);
//            sprintf(print_buf, "ab_setangle=%f,now_ab_angle=%f,maxangle=%f,now_rel_angle=%f,minangle=%f\r\n",
//                    gimbal_control.gimbal_pitch_motor.absolute_angle_set,
//                    gimbal_control.gimbal_pitch_motor.absolute_angle,
//                    gimbal_control.gimbal_pitch_motor.max_relative_angle,
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
//            //IMU数据
//            SEGGER_RTT_SetTerminal(5);
//            sprintf(print_buf, "imu_tmp=%f,YAW=%f,PITCH=%f,ROLL=%f\r\n",
//                    bmi088_real_data.temp,
//                    INS_angle[0],
//                    INS_angle[1],
//                    INS_angle[2]);
//            SEGGER_RTT_WriteString(0, print_buf);
            //波形显示
//            RTT_PrintWave(4,
//                          &gimbal_control.gimbal_pitch_motor.absolute_angle_set,
//                          &gimbal_control.gimbal_pitch_motor.absolute_angle,
//                          &gimbal_control.gimbal_pitch_motor.motor_gyro_set,
//                          &gimbal_control.gimbal_pitch_motor.motor_gyro);
            /***********************打印数据 End *****************************/


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
            vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(50));
        }
    }

}

static void usb_printf(const char *fmt, ...) {
    static va_list ap;
    static uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *) print_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(print_buf, len);
}

static void Print_RTT_ReadBuffer(void) {
    unsigned NumBytes = sizeof(read_buf);
    NumBytes = SEGGER_RTT_Read(0, &read_buf[0], NumBytes);
    if (NumBytes) {
        int i;
        for (i = 0; i < NumBytes; i++)
            printf("%c", read_buf[i]);
        printf("\n");
    }
}

static void Use_RTT_SetConfig(void *const variable) {
    unsigned NumBytes = sizeof(read_buf);
    NumBytes = SEGGER_RTT_Read(0, &read_buf[0], NumBytes);
    if (NumBytes) {
        char *rest;
        int32_t var = strtol(&read_buf[0], &rest, 10);
        memcpy(variable, &var, sizeof(var));
    }
}


//void RTT_PrintWave(fp32 *param1, fp32 *param2, fp32 *param3, fp32 *param4, fp32 *param5, fp32 *param6) {
//    SEGGER_RTT_SetTerminal(0);
//    char buf[256];
//    sprintf(buf, "%f,%f,%f,%f,%f,%f\r\n", *(float *) param1, *(float *) param2,
//            *(float *) param3, *(float *) param4, *(float *) param5, *(float *) param6);
//    SEGGER_RTT_WriteString(0, buf);
//}

void RTT_PrintWave(int num_args, ...) {
    fp32 *param_point[num_args];
    int i;
    char buf[256];
    va_list arg;
    va_start(arg, num_args);
    int len = 0;
    for (i = 0; i < num_args; i++) {
        param_point[i] = va_arg(arg, fp32 *);

        if (i == (num_args - 1)) {
            len += sprintf((buf + len), "%f\r\n", *(float *) param_point[i]);
        } else {
            len += sprintf((buf + len), "%f,", *(float *) param_point[i]);
        }
    }
    va_end(arg);
    SEGGER_RTT_SetTerminal(0);
    SEGGER_RTT_WriteString(0, buf);
}