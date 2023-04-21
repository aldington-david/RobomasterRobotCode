/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "DWT.h"
#include "SEGGER_RTT.h"
#include "pid_auto_tune_task.h"
#include "global_control_define.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *                 
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_task_stack;
#endif

float32_t vx_rc_Interpolation[14] = {0};
float32_t vy_rc_Interpolation[14] = {0};
float32_t wz_rc_Interpolation[14] = {0};

//底盘运动数据
chassis_move_t chassis_move;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters) {
    //wait a time 
    //空闲一段时间
    vTaskDelay(pdMS_TO_TICKS(CHASSIS_TASK_INIT_TIME));
    //chassis init
    //底盘初始化
    chassis_init(&chassis_move);
    //make sure all chassis motor is online,
    //判断底盘电机是否都在线
    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) ||
           toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE)) {
        vTaskDelay(pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
    }

    TickType_t LoopStartTime;

    while (1) {
        DWT_get_time_interval_us(&global_task_time.tim_chassis_task);
        LoopStartTime = xTaskGetTickCount();
        //set chassis control mode
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //模式切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //set chassis control set-point 
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

        //make sure  one motor is online at least, so that the control CAN message can be received
        //确保至少一个电机在线， 这样CAN控制包可以被接收到
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) &&
              toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE))) {
            //when remote control is offline, chassis motor should receive zero current. 
            //当遥控器掉线的时候，发送给底盘电机零电流.
            if (toe_is_error(DBUS_TOE)) {
                CAN1_cmd_0x200(0, 0, 0, 0);
            } else {
                //send control current
                //发送控制电流
                CAN1_cmd_0x200(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                               chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            }
        }
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
        vTaskDelayUntil(&LoopStartTime, pdMS_TO_TICKS(CHASSIS_CONTROL_TIME_MS));
    }
}

/**
  * @brief          获取chassis_task栈大小
  * @param[in]      none
  * @retval         chassis_task_stack:任务堆栈大小
  */
uint32_t get_stack_of_chassis_task(void) {
    return chassis_task_stack;
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init) {
    if (chassis_move_init == NULL) {
        return;
    }

    //chassis motor speed PID
    //底盘速度环pid值
    const static float32_t motor_speed_pid[3] = {1848.1342f, 1848.1342f, 462.0335f};
    const static float32_t chassis_vx_speed_pid[3] = {1.3893f, 1.1114f, 0.4341f};
    const static float32_t chassis_vy_speed_pid[3] = {1.3527f, 2.7054f, 0.1690f};
    const static float32_t chassis_wz_speed_pid[3] = {1.1265f, 0.9012f, 0.3520f};
    //chassis angle PID
    //底盘角度pid值
    const static float32_t chassis_yaw_follow_pid[3] = {1.8f, 10.143f, 300.4152f};

//    const static float32_t chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
//    const static float32_t chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning， chassis mode is raw 
    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //get gimbal motor data point
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    //pid调谐数据指针获取
    chassis_move_init->pid_auto_tune_data_point = get_pid_auto_tune_data_point();
    //超级电容数据指针获取
    chassis_move_init->super_capacitance_measure_point = get_super_capacitance_measure_point();
    //get chassis motor data point,  initialize motor speed PID
    //获取底盘电机数据指针，初始化PID 
    for (i = 0; i < 4; i++) {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT,
                 M3505_MOTOR_SPEED_PID_MAX_IOUT, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
    //initialize angle PID
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_follow_pid,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PID_init(&chassis_move_init->chassis_vx_speed_pid, PID_POSITION, chassis_vx_speed_pid,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, 0.1f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PID_init(&chassis_move_init->chassis_vy_speed_pid, PID_POSITION, chassis_vy_speed_pid,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, 0.1f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PID_init(&chassis_move_init->chassis_wz_speed_pid, PID_POSITION, chassis_wz_speed_pid,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT,
             CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, 0.1f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    //first order low-pass filter  replace ramp function
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, CHASSIS_ACCEL_X_NUM);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, CHASSIS_ACCEL_Y_NUM);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, CHASSIS_ACCEL_WZ_NUM);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_yaw_follow, CHASSIS_CONTROL_TIME, 0.0f);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_spin, CHASSIS_CONTROL_TIME, 0.7f);
    //max and min speed
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;


    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    chassis_move_init->soft_power_limit = chassis_move_init->power_limit = POWER_LIMIT;
    //update data
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode) {
    if (chassis_move_mode == NULL) {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit) {
    if (chassis_move_transit == NULL) {
        return;
    }

    if ((last_chassis_behaviour_mode == CHASSIS_FORWARD_FOLLOW_GIMBAL_YAW) &&
        (chassis_behaviour_mode == CHASSIS_SPIN)) {
        first_order_filter_clear(&chassis_move_transit->chassis_cmd_slow_spin);
        chassis_mode_change_flag = 1;
        spin_pid_change_flag = 1;
    }

    if ((last_chassis_behaviour_mode == CHASSIS_SPIN) &&
        (chassis_behaviour_mode == CHASSIS_FORWARD_FOLLOW_GIMBAL_YAW)) {
        if ((chassis_move_transit->chassis_yaw_motor->relative_angle > HALF_PI &&
             chassis_move_transit->chassis_yaw_motor->relative_angle < PI) ||
            (chassis_move_transit->chassis_yaw_motor->relative_angle > THREE_HALF_PI &&
             chassis_move_transit->chassis_yaw_motor->relative_angle < 2 * PI)) {
            chassis_mode_change_flag = 1;
            chassis_follow_change_flag = 1;
        } else {
            chassis_behaviour_mode = CHASSIS_SPIN;
            chassis_move_transit->chassis_mode = CHASSIS_VECTOR_TO_GIMBAL_YAW;
        }
    }


    last_chassis_behaviour_mode = chassis_behaviour_mode;

//    SEGGER_RTT_printf(0,"%f\r\n",chassis_move_transit->chassis_yaw_motor->relative_angle);


    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode) {
        return;
    }//change to follow gimbal angle mode
    //切入云台前向模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_TO_GIMBAL_YAW) &&
        chassis_move_transit->chassis_mode == CHASSIS_VECTOR_TO_GIMBAL_YAW) {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    } else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) &&
               chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) {
        //change to follow chassis yaw angle
        //切入底盘跟随云台角度模式
        if (chassis_follow_change_flag) {
            if (chassis_move_transit->chassis_yaw_motor->relative_angle > 0 &&
                chassis_move_transit->chassis_yaw_motor->relative_angle < PI) {
                chassis_move_transit->chassis_follow_reverse_flag = 1;
            } else {
                chassis_move_transit->chassis_follow_reverse_flag = 0;
            }
            chassis_follow_change_flag = 0;
        } else {
            if (chassis_move_transit->chassis_yaw_motor->relative_angle > HALF_PI &&
                chassis_move_transit->chassis_yaw_motor->relative_angle < THREE_HALF_PI) {
                chassis_move_transit->chassis_follow_reverse_flag = 1;
            } else {
                chassis_move_transit->chassis_follow_reverse_flag = 0;
            }
        }

    } else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) &&
               chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW) {
        //change to no follow angle
        //切入不跟随云台模式
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update) {
    if (chassis_move_update == NULL) {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++) {
        //update motor speed, accel is differential of speed PID
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN *
                                                      chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel =
                chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx =
            (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed +
             chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) *
            MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy =
            (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed +
             chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) *
            MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz =
            (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed -
             chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) *
            MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    chassis_move_update->chassis_yaw = chassis_move_update->chassis_yaw_motor->relative_angle;
    chassis_move_update->chassis_pitch = rad_format(
            *(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) -
            chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void chassis_rc_to_control_vector(float32_t *vx_set, float32_t *vy_set, float32_t *wz_set,
                                  chassis_move_t *chassis_move_rc_to_vector) {
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL) {
        return;
    }
    static uint8_t interpolation_num = 100;
    static uint8_t move_point = 0;
    static bool_t no_bias_flag = 0;
    float32_t vx_bias = 0;
    float32_t vy_bias = 0;
    float32_t wz_bias = 0;
    int16_t vx_channel, vy_channel, wz_channel;
    float32_t vx_set_channel, vy_set_channel, wz_set_channel;
    float32_t vx_raw, vy_raw;
    vx_set_channel = vy_set_channel = wz_set_channel = vx_channel = vy_channel = wz_channel = 0;
    if (!switch_is_up(chassis_move_rc_to_vector->chassis_RC->rc.s[RADIO_CONTROL_SWITCH_L])) {


        //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
        //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
        rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel,
                          CHASSIS_RC_DEADLINE);
        rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel,
                          CHASSIS_RC_DEADLINE);
        rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel,
                          CHASSIS_RC_DEADLINE);
        vx_bias = chassis_move_rc_to_vector->vx_set - chassis_move_rc_to_vector->vx;
        vy_bias = chassis_move_rc_to_vector->vy_set - chassis_move_rc_to_vector->vy;
        wz_bias = chassis_move_rc_to_vector->wz_set - chassis_move_rc_to_vector->wz;

        if ((fabs(vx_bias) < 1.0) || (fabs(vx_bias)) > 8.0 || vx_channel == 0) {
            vx_bias = 0;
            no_bias_flag = 1;
        }
        if ((fabs(vy_bias) < 1.0) || (fabs(vy_bias)) > 8.0 || vy_channel == 0) {
            vy_bias = 0;
            no_bias_flag = 1;
        }
        if ((fabs(wz_bias) < 1.0) || (fabs(wz_bias)) > 8.0 || wz_channel == 0) {
            wz_bias = 0;
            no_bias_flag = 1;
        }

        if (chassis_move_rc_to_vector->chassis_RC->chassis_update_flag ||
            (chassis_move_rc_to_vector->chassis_RC->chassis_update_flag && no_bias_flag)) {
            clear_chassis_rc_update_flag();
            sigmoidInterpolation(0, (vx_channel * CHASSIS_VX_RC_SEN + vx_bias / interpolation_num) * 1000, 14,
                                 vx_rc_Interpolation);
            sigmoidInterpolation(0, (vy_channel * -CHASSIS_VY_RC_SEN + vy_bias / interpolation_num) * 1000, 14,
                                 vy_rc_Interpolation);
            sigmoidInterpolation(0, (wz_channel * CHASSIS_WZ_RC_SEN + wz_bias / interpolation_num) * 1000, 14,
                                 wz_rc_Interpolation);

            move_point = 0;
            no_bias_flag = 0;
        }
        vx_set_channel = vx_rc_Interpolation[move_point] / 1000;
        vy_set_channel = vy_rc_Interpolation[move_point] / 1000;
        wz_set_channel = wz_rc_Interpolation[move_point] / 1000;
        move_point++;
        if (move_point > 13) {
            move_point = 0;
        }
//        SEGGER_RTT_printf(0,"vx=%f,vy=%f,wz=%f\r\n",vx_set_channel,vy_set_channel,wz_set_channel);
    } else {
        //keyboard set speed set-point
        //键盘控制
        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY) {
            vx_raw = 50.0f;
        } else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY) {
            vx_raw = -50.0f;
        }

        if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY) {
            vy_raw = 50.0f;
        } else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY) {
            vy_raw = -50.0f;
        }


        vx_bias = chassis_move_rc_to_vector->vx_set - chassis_move_rc_to_vector->vx;
        vy_bias = chassis_move_rc_to_vector->vy_set - chassis_move_rc_to_vector->vy;
        wz_bias = chassis_move_rc_to_vector->wz_set - chassis_move_rc_to_vector->wz;

        if ((fabs(vx_bias) < 0.5) || (fabs(vx_bias)) > 8.0 || vx_channel == 0) {
            vx_bias = 0;
            no_bias_flag = 1;
        }
        if ((fabs(vy_bias) < 0.5) || (fabs(vy_bias)) > 8.0 || vy_channel == 0) {
            vy_bias = 0;
            no_bias_flag = 1;
        }
        if ((fabs(wz_bias) < 0.5) || (fabs(wz_bias)) > 8.0) {
            wz_bias = 0;
            no_bias_flag = 1;
        }

        if (chassis_move_rc_to_vector->chassis_RC->chassis_update_flag ||
            (chassis_move_rc_to_vector->chassis_RC->chassis_update_flag && no_bias_flag)) {
            clear_chassis_rc_update_flag();
            sigmoidInterpolation(0, (vx_raw + vx_bias / interpolation_num) * 1000, 14,
                                 vx_rc_Interpolation);
            sigmoidInterpolation(0, (vy_raw + vy_bias / interpolation_num) * 1000, 14,
                                 vy_rc_Interpolation);
            sigmoidInterpolation(0, (wz_bias / interpolation_num) * 1000, 14,
                                 wz_rc_Interpolation);

            move_point = 0;
            no_bias_flag = 0;
        }
        vx_set_channel = vx_rc_Interpolation[move_point] / 1000;
        vy_set_channel = vy_rc_Interpolation[move_point] / 1000;
        wz_set_channel = wz_rc_Interpolation[move_point] / 1000;
        move_point++;
        if (move_point > 13) {
            move_point = 0;
        }
//        SEGGER_RTT_printf(0,"vx=%f,vy=%f,wz=%f\r\n",vx_set_channel,vy_set_channel,wz_set_channel);
    }
    *vx_set = vx_set_channel;
    *vy_set = vy_set_channel;
    *wz_set = wz_set_channel;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control) {

    if (chassis_move_control == NULL) {
        return;
    }


    float32_t vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f, vx_raw = 0.0f, vy_raw = 0.0f, wz_raw = 0.0f;
    //get three control set-point, 获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //follow gimbal mode
    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_TO_GIMBAL_YAW) {
        float32_t sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = sinf(chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = cosf(chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        chassis_move_control->wz_set = angle_set;
        //set control relative angle  set-point
        //设置控制相对云台角度
//        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
//        chassis_move_control->chassis_relative_angle_set = 0.0f;
        //calculate ratation speed
        //计算旋转PID角速度
//        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //速度限幅
        vx_raw = ALL_PID(&chassis_move_control->chassis_vx_speed_pid,
                         chassis_move_control->vx,
                         chassis_move_control->vx_set);
        vy_raw = ALL_PID(&chassis_move_control->chassis_vy_speed_pid,
                         chassis_move_control->vy,
                         chassis_move_control->vy_set);
        wz_raw = ALL_PID(&chassis_move_control->chassis_wz_speed_pid,
                         chassis_move_control->wz,
                         chassis_move_control->wz_set);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_vx, vx_raw);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_vy, vy_raw);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_wz, wz_raw);
//        chassis_move_control->vx_set = chassis_move_control->chassis_cmd_slow_set_vx.out;
//        chassis_move_control->vy_set = chassis_move_control->chassis_cmd_slow_set_vy.out;
//        chassis_move_control->wz_set = chassis_move_control->chassis_cmd_slow_set_wz.out;
//        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed,
//                                                      chassis_move_control->vx_max_speed);
//        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed,
//                                                      chassis_move_control->vy_max_speed);
//        SEGGER_RTT_printf(0, "%f,%f,%f,%f\r\n", chassis_move_control->vx,
//                          chassis_move_control->vy, chassis_move_control->vx_set, chassis_move_control->vy_set);
    } else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) {
        float32_t sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = sinf(chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = cosf(chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
//        chassis_move_control->vx_set = vx_set;
//        chassis_move_control->vy_set = vy_set;
        //set chassis yaw angle set-point
        //设置底盘控制的角度
        if (chassis_move_control->chassis_follow_reverse_flag) {
            chassis_move_control->chassis_yaw_set = PI;
            chassis_move_control->vx_set = -vx_set;
            chassis_move_control->vy_set = -vy_set;
        } else {
            chassis_move_control->chassis_yaw_set = 0;
            chassis_move_control->vx_set = vx_set;
            chassis_move_control->vy_set = vy_set;
        }
        //calculate rotation speed
        //计算旋转的角速度
        wz_raw = ALL_PID(&chassis_move_control->chassis_angle_pid,
                         chassis_move_control->chassis_yaw,
                         chassis_move_control->chassis_yaw_set);
//        SEGGER_RTT_printf(0, "chassis_move_control->wz_set =

        //speed limit
        //速度限幅
        vx_raw = ALL_PID(&chassis_move_control->chassis_vx_speed_pid,
                         chassis_move_control->vx,
                         chassis_move_control->vx_set);
        vy_raw = ALL_PID(&chassis_move_control->chassis_vy_speed_pid,
                         chassis_move_control->vy,
                         chassis_move_control->vy_set);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_vx, vx_raw);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_vy, vy_raw);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_yaw_follow, wz_raw);
//        chassis_move_control->vx_set = chassis_move_control->chassis_cmd_slow_set_vx.out;
//        chassis_move_control->vy_set = chassis_move_control->chassis_cmd_slow_set_vy.out;
//        chassis_move_control->wz_set = chassis_move_control->chassis_cmd_slow_yaw_follow.out;
//        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed,
//                                                      chassis_move_control->vx_max_speed);
//        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed,
//                                                      chassis_move_control->vy_max_speed);
        chassis_move_control->wz_set = wz_raw;
//        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed,
//                                                      chassis_move_control->vx_max_speed);
//        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed,
//                                                      chassis_move_control->vy_max_speed);
    } else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW) {
        //"angle_set" is rotation speed set-point
        //“angle_set” 是旋转速度控制
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        vx_raw = ALL_PID(&chassis_move_control->chassis_vx_speed_pid,
                         chassis_move_control->vx,
                         chassis_move_control->vx_set);
        vy_raw = ALL_PID(&chassis_move_control->chassis_vy_speed_pid,
                         chassis_move_control->vy,
                         chassis_move_control->vy_set);
        wz_raw = ALL_PID(&chassis_move_control->chassis_wz_speed_pid,
                         chassis_move_control->wz,
                         chassis_move_control->wz_set);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_vx, vx_raw);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_vy, vy_raw);
//        first_order_filter_cali(&chassis_move_control->chassis_cmd_slow_set_wz, wz_raw);
//        chassis_move_control->vx_set = chassis_move_control->chassis_cmd_slow_set_vx.out;
//        chassis_move_control->vy_set = chassis_move_control->chassis_cmd_slow_set_vy.out;
//        chassis_move_control->wz_set = chassis_move_control->chassis_cmd_slow_set_wz.out;
//        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed,
//                                                      chassis_move_control->vx_max_speed);
//        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed,
//                                                      chassis_move_control->vy_max_speed);
    } else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW) {
        //in raw mode, set-point is sent to CAN bus
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    } else if (chassis_move_control->chassis_mode == CHASSIS_NOT_MOVE) {
        //底盘无力
        chassis_move_control->vx_set = 0;
        chassis_move_control->vy_set = 0;
        chassis_move_control->wz_set = 0;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    } else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_PID_AUTO_TUNE) {
        if (chassis_move_control->pid_auto_tune_data_point->tune_type == SPEED_TO_SPEED) {
            chassis_move_control->vx_set = pid_auto_tune_data.control_list.vx_control_value;
            chassis_move_control->vy_set = pid_auto_tune_data.control_list.vy_control_value;
            chassis_move_control->wz_set = pid_auto_tune_data.control_list.wz_control_value;
        } else if (chassis_move_control->pid_auto_tune_data_point->tune_type == ANGLE_TO_SPEED) {
            chassis_move_control->wz_set = pid_auto_tune_data.control_list.wz_control_value;
        }
    }
}

/**
  * @brief          four mecanum wheels speed is calculated by three param. 
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度
  * @retval         none
  */
static void
chassis_vector_to_mecanum_wheel_speed(const float32_t vx_set, const float32_t vy_set, const float32_t wz_set,
                                      float32_t wheel_speed[4]) {
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
//    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
//    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[0] = -vx_set - vy_set + -MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + -MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + -MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + -MOTOR_DISTANCE_TO_CENTER * wz_set;
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop) {
    float32_t max_vector = 0.0f, vector_rate = 0.0f;
    float32_t temp = 0.0f;
    float32_t wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //mecanum wheel speed calculation
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set,
                                          wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW) {

        for (i = 0; i < 4; i++) {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t) (wheel_speed[i]);
        }
        //in raw mode, derectly return
        //raw控制直接返回
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++) {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabsf(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp) {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED) {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++) {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }
    if (PID_AUTO_TUNE) {
        if (chassis_move_control_loop->pid_auto_tune_data_point->tune_type == SPEED_TO_CURRENT) {
            chassis_move_control_loop->motor_chassis[0].give_current = (int16_t) (chassis_move_control_loop->pid_auto_tune_data_point->control_list.wheel_control_value);
            chassis_move_control_loop->motor_chassis[1].give_current = (int16_t) (chassis_move_control_loop->pid_auto_tune_data_point->control_list.wheel_control_value);
            chassis_move_control_loop->motor_chassis[2].give_current = (int16_t) (chassis_move_control_loop->pid_auto_tune_data_point->control_list.wheel_control_value);
            chassis_move_control_loop->motor_chassis[3].give_current = (int16_t) (chassis_move_control_loop->pid_auto_tune_data_point->control_list.wheel_control_value);
        } else if (chassis_move_control_loop->pid_auto_tune_data_point->tune_type == ANGLE_TO_SPEED ||
                   chassis_move_control_loop->pid_auto_tune_data_point->tune_type == SPEED_TO_SPEED) {
            for (i = 0; i < 4; i++) {
                chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
            }
            //calculate pid
            //计算pid
            for (i = 0; i < 4; i++) {
                ALL_PID(&chassis_move_control_loop->motor_speed_pid[i],
                        chassis_move_control_loop->motor_chassis[i].speed,
                        chassis_move_control_loop->motor_chassis[i].speed_set);
            }
            //赋值电流值
            for (i = 0; i < 4; i++) {
                chassis_move_control_loop->motor_chassis[i].give_current = (int16_t) (chassis_move_control_loop->motor_speed_pid[i].out);
            }
        }


    } else {
        //calculate pid
        //计算pid
        for (i = 0; i < 4; i++) {
            ALL_PID(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed,
                    chassis_move_control_loop->motor_chassis[i].speed_set);
        }

//        SEGGER_RTT_printf(0,"%d",chassis_move_control_loop->soft_power_limit);
        //功率控制
        chassis_power_control(chassis_move_control_loop);


        //赋值电流值
        for (i = 0; i < 4; i++) {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t) (chassis_move_control_loop->motor_speed_pid[i].out);
        }
    }

}
