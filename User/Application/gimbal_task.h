/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "remote_control.h"
#include "kalman_filter.h"
#include "pid.h"
#include "vision_task.h"
#include "PID_AutoTune.h"

#define MAX_6020_MOTOR_CAN_CURRENT 30000.0f

//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_MAX_OUT   MAX_6020_MOTOR_CAN_CURRENT
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_MAX_OUT   MAX_6020_MOTOR_CAN_CURRENT
#define YAW_SPEED_PID_MAX_IOUT  15000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 1.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        1.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.0f

#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 30.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 15.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   30.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  15.0f


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define RADIO_CONTROL_SWITCH_R       0
#define RADIO_CONTROL_SWITCH_L       1


//turn 180°
//掉头180 按键
//#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
//#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10

//遥控器摇杆灵敏度
#define YAW_RC_SEN    0.000161f
#define PITCH_RC_SEN  0.000115f

#define YAW_MOUSE_SEN   0.00084f
#define PITCH_MOUSE_SEN 0.0008f

//Original_vision_sen_define
//#define YAW_VISION_SEN   0.35f
//#define PITCH_VISION_SEN 0.15f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.0074532f
#define GIMBAL_INIT_PITCH_ANGLE_LIMIT     0.8726646f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            5000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.0523598f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.002f
#define GIMBAL_INIT_YAW_SPEED       0.004f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//ist校准
#define IST_CALI_YAW_MOTOR_SET   2000

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   4000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

#define IST_CALI_FORWARD_STEP  1
#define IST_CALI_BACKWARD_STEP  2

#define IST_CALI_START_STEP  IST_CALI_FORWARD_STEP
#define IST_CALI_END_STEP    3
//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.0007669903939208984375f //      2*  PI  /8192
#endif

typedef enum {
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
    GIMBAL_MOTOR_PID_AUTO_TUNE, //电机PID自动调参控制
} gimbal_motor_mode_e;
//not_use
typedef struct {
    float32_t kp;
    float32_t ki;
    float32_t kd;

    float32_t set;
    float32_t get;
    float32_t err;

    float32_t max_out;
    float32_t max_iout;

    float32_t Pout;
    float32_t Iout;
    float32_t Dout;

    float32_t out;

    float32_t error_last;
    float32_t Integral_Separation;


    bool D_First;
    float32_t D_Filter_Ratio;
    float32_t last_get;
    float32_t D3;
    float32_t D2;
    float32_t D1;

} gimbal_PID_t;

typedef struct {
    const motor_measure_t *gimbal_motor_measure;

    pid_type_def gimbal_motor_absolute_angle_pid;
    pid_type_def gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;

    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    int32_t offset_ecd;
    float32_t max_relative_angle; //rad
    float32_t min_relative_angle; //rad

    float32_t relative_angle;     //rad
    float32_t relative_angle_set; //rad
    float32_t absolute_angle;     //rad
    float32_t absolute_angle_set; //rad
    float32_t motor_gyro;         //rad/s
    float32_t motor_gyro_set;
    float32_t motor_speed;
    float32_t raw_cmd_current;
    float32_t current_set;
    int16_t given_current;

    float LpfFactor;

    extKalman_t MotorSpeed_Kalman;
    extKalman_t Cloud_Motor_Current_Kalman_Filter;

} gimbal_motor_t;

typedef struct {
    float32_t max_yaw;
    float32_t min_yaw;
    float32_t max_pitch;
    float32_t min_pitch;
    int32_t max_yaw_ecd;
    int32_t min_yaw_ecd;
    int32_t max_pitch_ecd;
    int32_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct {
    uint8_t step;
} ist_step_cali_t;

typedef struct {
    const volatile vision_control_t *gimbal_vision_ctrl;
    const volatile RC_ctrl_t *gimbal_rc_ctrl;
    const volatile pid_auto_tune_t *pid_auto_tune_data_point;
    const float32_t *gimbal_INT_angle_point;
    const float32_t *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
    ist_step_cali_t ist_cali;

    uint8_t add_angle_point_num;


    float32_t fric1_current_set;
    int16_t fric1_give_current;
    float32_t fric2_current_set;
    int16_t fric2_give_current;

} gimbal_control_t;

/**
  * @brief          获取gimbal_task栈大小
  * @param[in]      none
  * @retval         gimbal_task_stack:任务堆栈大小
  */
extern uint32_t get_stack_of_gimbal_task(void);

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */

extern void gimbal_task(void const *pvParameters) __attribute__((noreturn));

/**
  * @brief          gimbal cali calculate, return motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:pitch max relative angle
  * @param[out]     min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern bool_t
cmd_cali_gimbal_hook(int32_t *yaw_offset, int32_t *pitch_offset, float32_t *max_yaw, float32_t *min_yaw, float32_t *max_pitch,
                     float32_t *min_pitch);

/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @param[in]      yaw_offse:yaw 中值
  * @param[in]      pitch_offset:pitch 中值
  * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
  * @param[in]      min_yaw:yaw 最小相对角度
  * @param[in]      max_yaw:pitch 最大相对角度
  * @param[in]      min_yaw:pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern void
set_cali_gimbal_hook(const int32_t yaw_offset, const int32_t pitch_offset, const float32_t max_yaw, const float32_t min_yaw,
                     const float32_t max_pitch, const float32_t min_pitch);

/**
  * @brief          offset_ecd 偏移,以total_ecd为基准需要校正turnCount值
  * @param[in]      gimbal_cali: 校准数据
  * @retval         none
  */
extern void gimbal_offset_ecd_cali(gimbal_control_t *init);
/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          计算ecd与offset_ecd之间的相对角度
  * @param[in]      ecd: 电机当前编码
  * @param[in]      offset_ecd: 电机中值编码
  * @retval         相对角度，单位rad
  */
extern float32_t motor_ecd_to_yaw_angle_change(uint16_t ecd, uint16_t offset_ecd);
extern float32_t motor_ecd_to_pitch_angle_change(uint16_t ecd, uint16_t offset_ecd);

extern gimbal_control_t gimbal_control;
#endif
