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



#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "print_task.h"
#include "SEGGER_RTT.h"
#include "USER_Filter.h"
#include "gimbal_task.h"
#include "kalman_filter.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "global_control_define.h"
#include "DWT.h"

//motor enconde value format, range[0-8191]
//电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);          \
                                                                                               \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid);        \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_task_stack;
#endif


/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          limit angle set in GIMBAL_MOTOR_GYRO mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add);

static void gimbal_absolute_angle_yaw_unlimit(gimbal_motor_t *gimbal_motor, float32_t add);
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCONDE mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add);


/**
  * @brief          在GIMBAL_MOTOR_ENCONDE模式，yaw无限制限制角度设定
  * @param[out]     gimbal_motor:yaw电机
  * @retval         none
  */
static void gimbal_relative_angle_yaw_unlimit(gimbal_motor_t *gimbal_motor, float32_t add);

/**
  * @brief          gimbal angle pid init, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      maxout: pid max out
  * @param[in]      intergral_limit: pid max iout
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
/**
  * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      maxout: pid最大输出
  * @param[in]      intergral_limit: pid最大积分输出
  * @param[in]      kp: pid kp
  * @param[in]      ki: pid ki
  * @param[in]      kd: pid kd
  * @retval         none
  */
static void
gimbal_PID_init(gimbal_PID_t *pid, float32_t maxout, float32_t max_iout, float32_t kp, float32_t ki, float32_t kd);

/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
/**
  * @brief          gimbal angle pid calc, because angle is in range(-pi,pi),can't use PID in pid.c
  * @param[out]     pid: pid data pointer stucture
  * @param[in]      get: angle feeback
  * @param[in]      set: angle set-point
  * @param[in]      error_delta: rotation speed
  * @retval         pid out
  */
/**
  * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
  * @param[out]     pid:云台PID指针
  * @param[in]      get: 角度反馈
  * @param[in]      set: 角度设定
  * @param[in]      error_delta: 角速度
  * @retval         pid 输出
  */
static float32_t gimbal_PID_calc(gimbal_PID_t *pid, float32_t get, float32_t set, float32_t error_delta);

/**
  * @brief          gimbal calibration calculate
  * @param[in]      gimbal_cali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     pitch_offset:pitch motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_pitch: pitch motor max machine angle
  * @param[out]     min_pitch: pitch motor min machine angle
  * @retval         none
  */
/**
  * @brief          云台校准计算
  * @param[in]      gimbal_cali: 校准数据
  * @param[out]     yaw_offset:yaw电机云台中值
  * @param[out]     pitch_offset:pitch 电机云台中值
  * @param[out]     max_yaw:yaw 电机最大机械角度
  * @param[out]     min_yaw: yaw 电机最小机械角度
  * @param[out]     max_pitch: pitch 电机最大机械角度
  * @param[out]     min_pitch: pitch 电机最小机械角度
  * @retval         none
  */
static void
calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, int32_t *yaw_offset, int32_t *pitch_offset,
                 float32_t *max_yaw,
                 float32_t *min_yaw, float32_t *max_pitch, float32_t *min_pitch);


//gimbal control data
//云台控制所有相关数据
gimbal_control_t gimbal_control;

//motor current 
//发送的电机电流
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, shoot_can_set_current = 0;

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

void gimbal_task(void const *pvParameters) {
    //等待陀螺仪任务更新陀螺仪数据
    //wait a time
    vTaskDelay(pdMS_TO_TICKS(GIMBAL_TASK_INIT_TIME));
    //gimbal init
    //云台初始化
    gimbal_init(&gimbal_control);
    //shoot init
    //射击初始化
    shoot_init();
    //wait for all motor online
    //判断电机是否都上线
    while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) {
        vTaskDelay(pdMS_TO_TICKS(GIMBAL_CONTROL_TIME));
        gimbal_feedback_update(&gimbal_control);             //云台数据反馈
    }

    TickType_t LoopStartTime;
//for_test
//设置当前云台朝向为前
//    gimbal_control.gimbal_yaw_motor.offset_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd;

    while (1) {
        DWT_update_task_time_us(&global_task_time.tim_gimbal_task);
        LoopStartTime = xTaskGetTickCount();
        gimbal_set_mode(&gimbal_control);                    //设置云台控制模式
        gimbal_mode_change_control_transit(&gimbal_control); //控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control);             //云台数据反馈
        gimbal_set_control(&gimbal_control);                 //设置云台控制量
        gimbal_control_loop(&gimbal_control);                //云台控制PID计算
        shoot_can_set_current = shoot_control_loop();        //射击任务控制循环
#if YAW_TURN
        yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
//        SEGGER_RTT_printf(0, "yaw_current:%d\r\n",yaw_can_set_current);
#else
        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
//        SEGGER_RTT_printf(0, "yaw_current:%d\r\n",yaw_can_set_current);
#endif

#if PITCH_TURN
        pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

        if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) &&
              toe_is_error(TRIGGER_MOTOR_TOE))) {
            if (toe_is_error(DBUS_TOE)) {
                CAN2_cmd_0x1ff(0, 0, 0, 0);
                CAN1_cmd_0x1ff(0, 0, 0, 0);
                CAN2_cmd_0x200(0, 0, 0, 0);
//                gimbal_offset_ecd_cali(&gimbal_control);
                gimbal_control.gimbal_yaw_motor.relative_angle_set = gimbal_control.gimbal_yaw_motor.relative_angle;
                gimbal_control.gimbal_pitch_motor.relative_angle_set = gimbal_control.gimbal_pitch_motor.relative_angle;
//                shoot_control.angle_set = shoot_control.angle;
//                gimbal_control.gimbal_yaw_motor.offset_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd;

            } else {
                CAN2_cmd_0x1ff(pitch_can_set_current, 0, shoot_can_set_current, 0);
                CAN1_cmd_0x1ff(yaw_can_set_current, 0, 0, 0);
                CAN2_cmd_0x200(gimbal_control.fric1_give_current, 0, 0, gimbal_control.fric2_give_current);
            }
        }

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(GIMBAL_CONTROL_TIME));
    }
}

/**
  * @brief          获取gimbal_task栈大小
  * @param[in]      none
  * @retval         gimbal_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_gimbal_task(void) {
    return gimbal_task_stack;
}

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
void
set_cali_gimbal_hook(const int32_t yaw_offset, const int32_t pitch_offset, const float32_t max_yaw,
                     const float32_t min_yaw, const float32_t max_pitch, const float32_t min_pitch) {
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}


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
//bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, float32_t *max_yaw, float32_t *min_yaw,
//                            float32_t *max_pitch, float32_t *min_pitch) {
//    if (gimbal_control.gimbal_cali.step == 0) {
//        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
//        //保存进入时候的数据，作为起始数据，来判断最大，最小值
//        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
//        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->total_ecd;
//        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
//        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd;
//        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
//        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->total_ecd;
//        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
//        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd;
//        return 0;
//    } else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP) {
//        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
//        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
//        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
//        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
//        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
//        gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
//        gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
//        gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
//        gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
//        gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
//        gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;
//        gimbal_control.gimbal_cali.step = 0;
//        return 1;
//    } else {
//        return 0;
//    }
//}
bool_t cmd_cali_gimbal_hook(int32_t *yaw_offset, int32_t *pitch_offset, float32_t *max_yaw, float32_t *min_yaw,
                            float32_t *max_pitch, float32_t *min_pitch) {
    if (gimbal_control.gimbal_cali.step == 0) {
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        //保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->total_ecd;
        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd;
        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->total_ecd;
        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->total_ecd;
        return 0;
    } else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP) {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;
        return 1;
    } else {
        return 0;
    }
}

/**
  * @brief          calc motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:pitch max relative angle
  * @param[out]     min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          云台校准计算，将校准记录的中值,最大 最小值
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         none
  */
//static void
//calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset,
//                 float32_t *max_yaw, float32_t *min_yaw, float32_t *max_pitch, float32_t *min_pitch) {
//    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL ||
//        max_pitch == NULL || min_pitch == NULL) {
//        return;
//    }
//
//    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;
//
//#if YAW_TURN
//    if (YAW_LIMIT == YAW_HAVE_LIMIT) {
//        temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;
//
//        if (temp_ecd < 0) {
//            temp_ecd += ECD_RANGE;
//        }
//        temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);
//
//        ecd_format(temp_ecd);
//        *yaw_offset = temp_ecd;
//        *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
//        *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
//    } else if (YAW_LIMIT == YAW_NO_LIMIT) {
//        *yaw_offset = gimbal_cali->max_yaw_ecd;
//        *max_yaw = 0;
//        *min_yaw = 0;
//    }
//#else
//    if (YAW_LIMIT == YAW_HAVE_LIMIT) {
//    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;
//
//    if (temp_ecd < 0) {
//        temp_ecd += ECD_RANGE;
//    }
//    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);
//
//    ecd_format(temp_ecd);
//    *yaw_offset = temp_ecd;
//    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
//    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
//}   else if (YAW_LIMIT == YAW_NO_LIMIT) {
//        *yaw_offset = gimbal_cali->max_yaw_ecd;
//        *max_yaw = 0;
//        *min_yaw = 0;
//    }
//#endif
//
//#if PITCH_TURN
//
//    temp_ecd = (int16_t) (gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
//    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
//    temp_ecd = (int16_t) (gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
//    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;
//
//    ecd_format(temp_max_ecd);
//    ecd_format(temp_min_ecd);
//
//    temp_ecd = temp_max_ecd - temp_min_ecd;
//
//    if (temp_ecd > HALF_ECD_RANGE) {
//        temp_ecd -= ECD_RANGE;
//    } else if (temp_ecd < -HALF_ECD_RANGE) {
//        temp_ecd += ECD_RANGE;
//    }
//
//    if (temp_max_ecd > temp_min_ecd) {
//        temp_min_ecd += ECD_RANGE;
//    }
//
//    temp_ecd = temp_max_ecd - temp_ecd / 2;
//
//    ecd_format(temp_ecd);
//
//    *pitch_offset = temp_ecd;
//
//    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
//    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
//
//#else
//    temp_ecd = (int16_t) (gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
//    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
//    temp_ecd = (int16_t) (gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
//    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;
//
//    ecd_format(temp_max_ecd);
//    ecd_format(temp_min_ecd);
//
//    temp_ecd = temp_max_ecd - temp_min_ecd;
//
//    if (temp_ecd > HALF_ECD_RANGE) {
//        temp_ecd -= ECD_RANGE;
//    } else if (temp_ecd < -HALF_ECD_RANGE) {
//        temp_ecd += ECD_RANGE;
//    }
//
//    temp_ecd = temp_max_ecd - temp_ecd / 2;
//
//    ecd_format(temp_ecd);
//
//    *pitch_offset = temp_ecd;
//
//    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
//    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
//#endif
//}

static void
calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, int32_t *yaw_offset, int32_t *pitch_offset,
                 float32_t *max_yaw, float32_t *min_yaw, float32_t *max_pitch, float32_t *min_pitch) {
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL ||
        max_pitch == NULL || min_pitch == NULL) {
        return;
    }

    int32_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    if (YAW_LIMIT == YAW_HAVE_LIMIT) {
        temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

        if (temp_ecd < 0) {
            temp_ecd += ECD_RANGE;
        }
        temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

        ecd_format(temp_ecd);
        *yaw_offset = temp_ecd;
        *max_yaw = motor_ecd_to_yaw_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
        *min_yaw = motor_ecd_to_yaw_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
    } else if (YAW_LIMIT == YAW_NO_LIMIT) {
        *yaw_offset = (gimbal_cali->max_yaw_ecd + gimbal_cali->min_yaw_ecd) / 2;
        *max_yaw = 0;
        *min_yaw = 0;
    }
#else
    if (YAW_LIMIT == YAW_HAVE_LIMIT) {
        temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

        if (temp_ecd < 0) {
            temp_ecd += ECD_RANGE;
        }
        temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

        ecd_format(temp_ecd);
        *yaw_offset = temp_ecd;
        *max_yaw = motor_ecd_to_yaw_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
        *min_yaw = motor_ecd_to_yaw_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
    } else if (YAW_LIMIT == YAW_NO_LIMIT) {
        *yaw_offset = (gimbal_cali->max_yaw_ecd + gimbal_cali->min_yaw_ecd) / 2;
        *max_yaw = 0;
        *min_yaw = 0;
    }
#endif

#if PITCH_TURN
    temp_ecd = (int32_t) (fabsf(gimbal_cali->max_pitch) / MOTOR_ECD_TO_RAD * PITCH_MOTOR_REDUCTION);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int32_t) (fabsf(gimbal_cali->min_pitch) / MOTOR_ECD_TO_RAD * PITCH_MOTOR_REDUCTION);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    temp_ecd = temp_max_ecd - temp_min_ecd;
    temp_ecd = temp_max_ecd - temp_ecd / 2;

    *pitch_offset = temp_ecd;

    *max_pitch = labs(gimbal_cali->max_pitch_ecd - *pitch_offset) * MOTOR_ECD_TO_RAD / PITCH_MOTOR_REDUCTION;
    *min_pitch = -labs(gimbal_cali->min_pitch_ecd - *pitch_offset) * MOTOR_ECD_TO_RAD / PITCH_MOTOR_REDUCTION;

//    temp_ecd = (int16_t) (gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
//    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
//    temp_ecd = (int16_t) (gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
//    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;
//
//    ecd_format(temp_max_ecd);
//    ecd_format(temp_min_ecd);
//
//    temp_ecd = temp_max_ecd - temp_min_ecd;
//
//    if (temp_ecd > HALF_ECD_RANGE) {
//        temp_ecd -= ECD_RANGE;
//    } else if (temp_ecd < -HALF_ECD_RANGE) {
//        temp_ecd += ECD_RANGE;
//    }
//
//    if (temp_max_ecd > temp_min_ecd) {
//        temp_min_ecd += ECD_RANGE;
//    }
//
//    temp_ecd = temp_max_ecd - temp_ecd / 2;
//
//    ecd_format(temp_ecd);
//
//    *pitch_offset = temp_ecd;
//
//    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
//    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int32_t) (fabsf(gimbal_cali->max_pitch) / MOTOR_ECD_TO_RAD * PITCH_MOTOR_REDUCTION);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int32_t) (fabsf(gimbal_cali->min_pitch) / MOTOR_ECD_TO_RAD * PITCH_MOTOR_REDUCTION);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    temp_ecd = temp_max_ecd - temp_min_ecd;
    temp_ecd = temp_max_ecd - temp_ecd / 2;

    *pitch_offset = temp_ecd;

    *max_pitch = labs(gimbal_cali->max_pitch_ecd - *pitch_offset) * MOTOR_ECD_TO_RAD / PITCH_MOTOR_REDUCTION;
    *min_pitch = -labs(gimbal_cali->min_pitch_ecd - *pitch_offset) * MOTOR_ECD_TO_RAD / PITCH_MOTOR_REDUCTION;
#endif
}

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
const gimbal_motor_t *get_yaw_motor_point(void) {
    return &gimbal_control.gimbal_yaw_motor;
}

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
const gimbal_motor_t *get_pitch_motor_point(void) {
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     init:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init) {


    static const float32_t Pitch_speed_pid[3] = {2000.0f, 3.5f, 0.0f};
    static const float32_t Yaw_speed_pid[3] = {6000.0f, 9.5f, 0.0f};

    static const float32_t Yaw_absolute_angle_pid[3] = {6.89f, 0.0f, 200.0f};
    static const float32_t Yaw_relative_angle_pid[3] = {10.59f, 0.0f, 1.0f};
    static const float32_t Pitch_absolute_angle_pid[3] = {15.89f, 0.0f, 200.0f};
    static const float32_t Pitch_relative_angle_pid[3] = {29.2f, 0.0f, 500.0f};
    //电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    //遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    //视觉数据指针获取
    init->gimbal_vision_ctrl = get_vision_control_point();
    //初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, PID_POSITION, Yaw_relative_angle_pid,
             YAW_ENCODE_RELATIVE_PID_MAX_OUT,
             YAW_ENCODE_RELATIVE_PID_MAX_IOUT, 1000, 1, 0.01f, 2.0f, 0, 0.0f, 0, 0.0f, 0, 0, 0, 1, 0.9f);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, PID_POSITION, Yaw_absolute_angle_pid,
             YAW_ENCODE_RELATIVE_PID_MAX_OUT,
             YAW_ENCODE_RELATIVE_PID_MAX_IOUT, 1000, 1, 0.01f, 2.0f, 0, 0.0f, 0, 0.0f, 0, 0, 0, 0, 0);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT,
             YAW_SPEED_PID_MAX_IOUT, 1000, 1, 0.15f, 3.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0);


    //初始化pitch电机pid
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PID_POSITION, Pitch_relative_angle_pid,
             PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
             PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, 1000, 1, 0.01f, 2.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PID_POSITION, Pitch_absolute_angle_pid,
             PITCH_ENCODE_RELATIVE_PID_MAX_OUT,
             PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, 1000, 1, 0.01f, 2.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT,
             PITCH_SPEED_PID_MAX_IOUT, 0.8f, 1, 0.8f, 2.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    KalmanCreate(&init->gimbal_pitch_motor.Cloud_Motor_Current_Kalman_Filter, 0.001f, 0.05f);
    KalmanCreate(&init->gimbal_yaw_motor.Cloud_Motor_Current_Kalman_Filter, 0.001f, 1.0f);
    KalmanCreate(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.D_Kalman, 1.0f, 1.0f);
    KalmanCreate(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.D_Kalman, 1.0f, 1.0f);
    KalmanCreate(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid.D_Kalman, 1.0f, 1.0f);
    KalmanCreate(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid.D_Kalman, 1.0f, 1.0f);
    KalmanCreate(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.D_Kalman, 1.0f, 1.0f);
    KalmanCreate(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid.D_Kalman, 1.0f, 1.0f);
    KalmanCreate(&init->gimbal_yaw_motor.MotorSpeed_Kalman, 0.001f, 0.4f);
    KalmanCreate(&init->gimbal_pitch_motor.MotorSpeed_Kalman, 0.001f, 0.4f);


    init->gimbal_pitch_motor.LpfFactor = 0.9f;
    init->gimbal_yaw_motor.LpfFactor = 0.5f;

//    gimbal_offset_ecd_cali(init);
    //清除所有PID
    gimbal_total_pid_clear(init);

    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;


    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;


}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
  * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode) {
    if (set_mode == NULL) {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}
/**
  * @brief          gimbal some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update) {
    if (feedback_update == NULL) {
        return;
    }
    static float32_t yaw_motor_speed_last = 0;
    static float32_t yaw_relative_angle_last = 0;
    //云台数据更新
//#if PITCH_TURN
//    feedback_update->gimbal_pitch_motor.relative_angle = -(
//            (feedback_update->gimbal_pitch_motor.gimbal_motor_measure->total_ecd -
//             feedback_update->gimbal_pitch_motor.offset_ecd) * MOTOR_ECD_TO_RAD / PITCH_MOTOR_REDUCTION);
////    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(
////            feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
////            feedback_update->gimbal_pitch_motor.offset_ecd);
//#else
//
//    //    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(
//    //            feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
//    //            feedback_update->gimbal_pitch_motor.offset_ecd);
//    feedback_update->gimbal_pitch_motor.relative_angle = (
//            (feedback_update->gimbal_pitch_motor.gimbal_motor_measure->total_ecd -
//             feedback_update->gimbal_pitch_motor.offset_ecd) * MOTOR_ECD_TO_RAD / PITCH_MOTOR_REDUCTION);
//#endif

//#if YAW_TURN
////    yaw_relative_angle_last = feedback_update->gimbal_yaw_motor.relative_angle;
////    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(
////            feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
////            feedback_update->gimbal_yaw_motor.offset_ecd);
////    feedback_update->gimbal_yaw_motor.relative_angle = -( //offset_ecd加了有概率封车
////            (feedback_update->gimbal_yaw_motor.gimbal_motor_measure->total_ecd -
////             feedback_update->gimbal_yaw_motor.offset_ecd) * MOTOR_ECD_TO_RAD);
//
////    feedback_update->gimbal_yaw_motor.relative_angle = -(
////            (feedback_update->gimbal_yaw_motor.gimbal_motor_measure->total_ecd -
////             feedback_update->gimbal_yaw_motor.offset_ecd) * MOTOR_ECD_TO_RAD / YAW_MOTOR_REDUCTION);
//
//#else
//    //    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
//    //                                                                                 feedback_update->gimbal_yaw_motor.offset_ecd);
//        feedback_update->gimbal_yaw_motor.relative_angle = (
//                (feedback_update->gimbal_yaw_motor.gimbal_motor_measure->total_ecd -
//                 feedback_update->gimbal_yaw_motor.offset_ecd) * MOTOR_ECD_TO_RAD / YAW_MOTOR_REDUCTION);
//#endif

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_pitch_angle_change(
            gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd,
            gimbal_control.gimbal_pitch_motor.offset_ecd);

    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_yaw_angle_change(
            gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd,
            gimbal_control.gimbal_yaw_motor.offset_ecd);

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point +
                                                       INS_GYRO_Y_ADDRESS_OFFSET);
    feedback_update->gimbal_pitch_motor.motor_speed = (
            feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm * PI / 30.0f / PITCH_MOTOR_REDUCTION);

    feedback_update->gimbal_yaw_motor.motor_speed = (feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm *
                                                     PI / 30.0f / YAW_MOTOR_REDUCTION);

    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point +
                                                         INS_YAW_ADDRESS_OFFSET);
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point +
                                                           INS_PITCH_ADDRESS_OFFSET);
//    SEGGER_RTT_printf(0, "%f,%f\r\n", feedback_update->gimbal_yaw_motor.absolute_angle,
//                      feedback_update->gimbal_pitch_motor.absolute_angle);
//for_test
//    kalman_test = feedback_update->gimbal_yaw_motor.motor_speed;
//    kalman_test = KalmanFilter(&MotorSpeed_Kalman,
//                               kalman_test);
    feedback_update->gimbal_yaw_motor.motor_speed = KalmanFilter(&feedback_update->gimbal_yaw_motor.MotorSpeed_Kalman,
                                                                 feedback_update->gimbal_yaw_motor.motor_speed);
    feedback_update->gimbal_pitch_motor.motor_speed = KalmanFilter(
            &feedback_update->gimbal_pitch_motor.MotorSpeed_Kalman,
            feedback_update->gimbal_pitch_motor.motor_speed);
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) *
                                                   (*(feedback_update->gimbal_INT_gyro_point +
                                                      INS_GYRO_Z_ADDRESS_OFFSET))
                                                   - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) *
                                                     (*(feedback_update->gimbal_INT_gyro_point +
                                                        INS_GYRO_X_ADDRESS_OFFSET));
}

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
float32_t motor_ecd_to_pitch_angle_change(uint16_t ecd, uint16_t offset_ecd) {
#if PITCH_TURN
    int32_t relative_ecd = offset_ecd - ecd;
    if (relative_ecd > HALF_ECD_RANGE) {
        relative_ecd -= ECD_RANGE;
    } else if (relative_ecd < -HALF_ECD_RANGE) {
        relative_ecd += ECD_RANGE;
    }
#else
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE) {
        relative_ecd -= ECD_RANGE;
    } else if (relative_ecd < -HALF_ECD_RANGE) {
        relative_ecd += ECD_RANGE;
    }
#endif
    return relative_ecd * MOTOR_ECD_TO_RAD / PITCH_MOTOR_REDUCTION;
}

float32_t motor_ecd_to_yaw_angle_change(uint16_t ecd, uint16_t offset_ecd) {
#if YAW_TURN
    int32_t relative_ecd = offset_ecd - ecd;
    if (relative_ecd < 0) {
        relative_ecd += ECD_RANGE;
    }
    if ((relative_ecd > ECD_RANGE) || (relative_ecd < -ECD_RANGE)) {
        loop_uint16_constrain(relative_ecd, 0, ECD_RANGE);
    }
#else
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd < 0) {
        relative_ecd += ECD_RANGE;
    }
    if ((relative_ecd > ECD_RANGE) || (relative_ecd < -ECD_RANGE)) {
        loop_uint16_constrain(relative_ecd, -ECD_RANGE, ECD_RANGE);
    }
#endif

    return relative_ecd * MOTOR_ECD_TO_RAD / YAW_MOTOR_REDUCTION;
}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
  * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change) {
    if (gimbal_mode_change == NULL) {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW &&
        gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    } else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO &&
               gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    } else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE &&
               gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW &&
        gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    } else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO &&
               gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    } else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE &&
               gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
  * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control) {
    if (set_control == NULL) {
        return;
    }

    float32_t add_yaw_angle = 0.0f;
    float32_t add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
//    tracer1=add_yaw_angle;//for_test
    //yaw电机模式控制
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        //raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    } else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) {
        //gyro模式下，陀螺仪角度控制
        if (YAW_LIMIT == YAW_HAVE_LIMIT) {
            gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
        } else if (YAW_LIMIT == YAW_NO_LIMIT) {
            gimbal_absolute_angle_yaw_unlimit(&set_control->gimbal_yaw_motor, add_yaw_angle);
        }
    } else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) {
        //enconde模式下，电机编码角度控制
        if (YAW_LIMIT == YAW_HAVE_LIMIT) {
            gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
        } else if (YAW_LIMIT == YAW_NO_LIMIT) {
            gimbal_relative_angle_yaw_unlimit(&set_control->gimbal_yaw_motor, add_yaw_angle);
        }
    }

    //pitch电机模式控制
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        //raw模式下，直接发送控制值
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    } else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) {
        //gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    } else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) {
        //enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add) {
//    static float32_t bias_angle;
//    static float32_t angle_set;
//    if (gimbal_motor == NULL) {
//        return;
//    }
//    //now angle error
//    //当前控制误差角度
//    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
//    bias_angle_test = bias_angle; //for_test
//    //relative angle + angle error + add_angle > max_relative angle
//    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
//    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle) {
//        //如果是往最大机械角度控制方向
//        if (add > 0.0f) {
//            //calculate max add_angle
//            //计算出一个最大的添加角度，
//            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
//        }
//    } else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle) {
//        if (add < 0.0f) {
//            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
//        }
//    }
//    angle_set = gimbal_motor->absolute_angle_set;
//    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
//    add_angle_test = add;
    if (gimbal_motor == NULL) {
        return;
    }
    gimbal_motor->absolute_angle_set += add;
//    float32_t relative_angle_set_tmp = gimbal_motor->relative_angle_set + add;
//    Filter_IIRLPF(relative_angle_set_tmp, &gimbal_motor->relative_angle_set, gimbal_motor->LpfFactor);
    //是否超过最大 最小值
    if (gimbal_motor->absolute_angle_set > gimbal_motor->max_relative_angle) {
        gimbal_motor->absolute_angle_set = gimbal_motor->max_relative_angle;
    } else if (gimbal_motor->absolute_angle_set < gimbal_motor->min_relative_angle) {
        gimbal_motor->absolute_angle_set = gimbal_motor->min_relative_angle;
    }
}

static void gimbal_absolute_angle_yaw_unlimit(gimbal_motor_t *gimbal_motor, float32_t add) {
    if (gimbal_motor == NULL) {
        return;
    }
    gimbal_motor->absolute_angle_set += add;
//static uint32_t i = 0;
//    i++;
//    if((i%27) >0){
//    float32_t relative_angle_set_tmp = gimbal_motor->relative_angle_set + add;
//    Filter_IIRLPF(relative_angle_set_tmp, &gimbal_motor->relative_angle_set, gimbal_motor->LpfFactor);
    gimbal_motor->absolute_angle_set = loop_fp32_constrain(gimbal_motor->absolute_angle_set, 0, 2 * PI);
//        i = 0;
//    }

//    //是否超过最大 最小值
//    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle) {
//        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
//    } else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle) {
//        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
//    }
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add) {
    if (gimbal_motor == NULL) {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle) {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    } else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle) {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }

//    if (gimbal_motor->relative_angle_set < gimbal_motor->max_relative_angle) {
//        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
//    } else if (gimbal_motor->relative_angle_set > gimbal_motor->min_relative_angle) {
//        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
//    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_relative_angle_yaw_unlimit(gimbal_motor_t *gimbal_motor, float32_t add) {
    if (gimbal_motor == NULL) {
        return;
    }
    gimbal_motor->relative_angle_set += add;
//    gimbal_motor->relative_angle_set += add;
//static uint32_t i = 0;
//    i++;
//    if((i%27) >0){
//    float32_t relative_angle_set_tmp = gimbal_motor->relative_angle_set + add;
//    Filter_IIRLPF(relative_angle_set_tmp, &gimbal_motor->relative_angle_set, gimbal_motor->LpfFactor);
    gimbal_motor->relative_angle_set = loop_fp32_constrain(gimbal_motor->relative_angle_set, 0, 2 * PI);
//        i = 0;
//    }

//    //是否超过最大 最小值
//    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle) {
//        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
//    } else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle) {
//        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
//    }
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop) {
    if (control_loop == NULL) {
        return;
    }

    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    } else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    } else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW) {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    } else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    } else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE) {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor) {
    if (gimbal_motor == NULL) {
        return;
    }
    gimbal_motor->motor_gyro_set = ALL_PID(&gimbal_motor->gimbal_motor_absolute_angle_pid,
                                           gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set);

    gimbal_motor->current_set = ALL_PID(&gimbal_motor->gimbal_motor_gyro_pid, -gimbal_motor->motor_speed,
                                        gimbal_motor->motor_gyro_set);

    if (gimbal_motor == &gimbal_control.gimbal_pitch_motor) {
//        SEGGER_RTT_printf(0,"%f\r\n",gimbal_motor->motor_gyro_set);
//        SEGGER_RTT_printf(0,"speed_set=%f,speed=%f,current=%f\r\n",gimbal_motor->motor_gyro_set,gimbal_motor->motor_speed,gimbal_motor->current_set);

    }

//    if (gimbal_motor == &gimbal_control.gimbal_pitch_motor) {
//        kalman_test = gimbal_motor->current_set;
//    }
    gimbal_motor->current_set = KalmanFilter(&gimbal_motor->Cloud_Motor_Current_Kalman_Filter,
                                             gimbal_motor->current_set);
//    for_test
//    if (gimbal_motor == &gimbal_control.gimbal_pitch_motor) {
//        RTT_PrintWave_np(2,
//                         kalman_test,
//                         gimbal_motor->current_set);
//    }

//for_test
//    kalman_test = KalmanFilter(&gimbal_motor->Cloud_Motor_Current_Kalman_Filter,
//                               gimbal_motor->current_set);
//    kalman_test = gimbal_motor->current_set;
    //控制值赋值
    gimbal_motor->given_current = (int16_t) (gimbal_motor->current_set);
//    //角度环，速度环串级pid调试
//    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid,
//                                                   gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set,
//                                                   gimbal_motor->motor_gyro);
//    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro,
//                                         gimbal_motor->motor_gyro_set);
//    //控制值赋值
//    gimbal_motor->given_current = (int16_t) (gimbal_motor->current_set);
////    RTT_PrintWave(&gimbal_motor->absolute_angle_set,&gimbal_motor->absolute_angle,&gimbal_motor->motor_gyro_set,&gimbal_motor->motor_gyro);
}

/**
  * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
  * @param[out]     gimbal_motor:pitch电机
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor) {
    if (gimbal_motor == NULL) {
        return;
    }
    gimbal_motor->motor_gyro_set = ALL_PID(&gimbal_motor->gimbal_motor_relative_angle_pid,
                                           gimbal_motor->relative_angle, gimbal_motor->relative_angle_set);

    gimbal_motor->current_set = ALL_PID(&gimbal_motor->gimbal_motor_gyro_pid, -gimbal_motor->motor_speed,
                                        gimbal_motor->motor_gyro_set);

    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor) {
//        SEGGER_RTT_printf(0,"%f\r\n",gimbal_motor->motor_gyro_set);
//        SEGGER_RTT_printf(0,"speed_set=%f,speed=%f,current=%f\r\n",gimbal_motor->motor_gyro_set,gimbal_motor->motor_speed,gimbal_motor->current_set);

    }

//    if (gimbal_motor == &gimbal_control.gimbal_pitch_motor) {
//        kalman_test = gimbal_motor->current_set;
//    }
    gimbal_motor->current_set = KalmanFilter(&gimbal_motor->Cloud_Motor_Current_Kalman_Filter,
                                             gimbal_motor->current_set);
//    for_test
//    if (gimbal_motor == &gimbal_control.gimbal_pitch_motor) {
//        RTT_PrintWave_np(2,
//                         kalman_test,
//                         gimbal_motor->current_set);
//    }

//for_test
//    kalman_test = KalmanFilter(&gimbal_motor->Cloud_Motor_Current_Kalman_Filter,
//                               gimbal_motor->current_set);
//    kalman_test = gimbal_motor->current_set;
    //控制值赋值
    gimbal_motor->given_current = (int16_t) (gimbal_motor->current_set);
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
/**
  * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
  * @param[out]     gimbal_motor:yaw电机或者pitch电机
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor) {
    if (gimbal_motor == NULL) {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t) (gimbal_motor->current_set);
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     gimbal_init: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     gimbal_init:"gimbal_control"变量指针.
  * @retval         none
  */
static void
gimbal_PID_init(gimbal_PID_t *pid, float32_t maxout, float32_t max_iout, float32_t kp, float32_t ki, float32_t kd) {
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

//not_use
static float32_t gimbal_PID_calc(gimbal_PID_t *pid, float32_t get, float32_t set, float32_t error_delta) {
    float32_t err;
    if (pid == NULL) {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}
//not_use
/**
  * @brief          gimbal PID clear, clear pid.out, iout.
  * @param[out]     gimbal_pid_clear: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          云台PID清除，清除pid的out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear) {
    if (gimbal_pid_clear == NULL) {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

/**
  * @brief          offset_ecd 偏移,以total_ecd为基准需要校正offset_ecd值
  * @param[in]      gimbal_cali: 校准数据
  * @retval         none
  */
void gimbal_offset_ecd_cali(gimbal_control_t *init) {
    if (labs(init->gimbal_yaw_motor.offset_ecd - init->gimbal_yaw_motor.gimbal_motor_measure->total_ecd) >
        HALF_ECD_RANGE) {
        if (init->gimbal_yaw_motor.offset_ecd < init->gimbal_yaw_motor.gimbal_motor_measure->total_ecd) {
            init->gimbal_yaw_motor.offset_ecd +=
                    ((labs(init->gimbal_yaw_motor.gimbal_motor_measure->total_ecd - init->gimbal_yaw_motor.offset_ecd) /
                      8192) + 1) * 8192;
        } else if (init->gimbal_yaw_motor.offset_ecd > init->gimbal_yaw_motor.gimbal_motor_measure->total_ecd) {
            init->gimbal_yaw_motor.offset_ecd -=
                    ((labs(init->gimbal_yaw_motor.gimbal_motor_measure->total_ecd - init->gimbal_yaw_motor.offset_ecd) /
                      8192) + 1) * 8192;
        }
    }
    if (labs(init->gimbal_pitch_motor.offset_ecd - init->gimbal_pitch_motor.gimbal_motor_measure->total_ecd) >
        HALF_ECD_RANGE) {
        if (init->gimbal_pitch_motor.offset_ecd < init->gimbal_pitch_motor.gimbal_motor_measure->total_ecd) {
            init->gimbal_pitch_motor.offset_ecd += ((labs(init->gimbal_pitch_motor.gimbal_motor_measure->total_ecd -
                                                          init->gimbal_pitch_motor.offset_ecd) / 8192) + 1) * 8192;
        } else if (init->gimbal_pitch_motor.offset_ecd > init->gimbal_pitch_motor.gimbal_motor_measure->total_ecd) {
            init->gimbal_pitch_motor.offset_ecd -= ((labs(init->gimbal_pitch_motor.gimbal_motor_measure->total_ecd -
                                                          init->gimbal_pitch_motor.offset_ecd) / 8192) + 1) * 8192;
        }
    }
}