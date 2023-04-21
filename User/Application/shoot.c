/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee_task.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "print_task.h"
#include "SEGGER_RTT.h"
#include "global_control_define.h"
#include "pid_auto_tune_task.h"


#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)


/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);

/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);


shoot_control_t shoot_control;          //射击数据


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void) {

    static const float32_t Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    static const float32_t Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const float32_t fric1_speed_pid[3] = {500, 0, 0};
    static const float32_t fric2_speed_pid[3] = {500, 0, 0};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //pid自动调谐指针
    shoot_control.pid_auto_tune_data_point = get_pid_auto_tune_data_point();

    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric1_motor_measure = get_trigger_motor1_measure_point();
    shoot_control.fric2_motor_measure = get_trigger_motor2_measure_point();

    //初始化PID
    PID_init(&shoot_control.trigger_motor_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT,
             TRIGGER_READY_PID_MAX_IOUT, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PID_init(&shoot_control.trigger_motor_angle_pid, PID_POSITION, Trigger_angle_pid, CONTINUE_TRIGGER_SPEED_MAX,
             CONTINUE_TRIGGER_SPEED_MAX / 2, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    PID_init(&shoot_control.fric1_motor_pid, PID_POSITION, fric1_speed_pid, TRIGGER_READY_PID_MAX_OUT,
             TRIGGER_READY_PID_MAX_IOUT, 1000, 0, 0, 0, 0, 0, 1, 0.003f, 0, 0, 0, 0, 0);
    PID_init(&shoot_control.fric2_motor_pid, PID_POSITION, fric2_speed_pid, TRIGGER_READY_PID_MAX_OUT,
             TRIGGER_READY_PID_MAX_IOUT, 1000, 0, 0, 0, 0, 0, 1, 0.003f, 0, 0, 0, 0, 0);
//    shoot_control.pwm = SHOOT_FRIC_PWM_ADD_VALUE;
    KalmanCreate(&shoot_control.Trigger_Motor_Current_Kalman_Filter, 10.0f, 0.5f);
    //更新数据
    shoot_feedback_update();
//    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN_PWM, FRIC_OFF_PWM);
//    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN_PWM, FRIC_OFF_PWM);
//    shoot_control.fric_pwm1 = FRIC_OFF_PWM;
//    shoot_control.fric_pwm2 = FRIC_OFF_PWM;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.angle_set = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
    shoot_control.fric1_speed = 0.0f;
    shoot_control.fric2_speed = 0.0f;
    shoot_control.shoot_speed_referee_set = SHOOT_SPEED_15_MS;
    shoot_control.fric_all_speed = SHOOT_SPEED_15_MS_TO_FRIC;
    shoot_control.fric1_speed_set = shoot_control.fric_all_speed;
    shoot_control.fric2_speed_set = -shoot_control.fric_all_speed;
    shoot_control.shoot_fric_state = FRIC_OFF;
#if SHOOT_TRIGGER_TURN
    shoot_control.trigger_speed_set = -CONTINUE_TRIGGER_SPEED;
#else
    shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
#endif
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void) {

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据


    if (shoot_control.shoot_mode == SHOOT_STOP) {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    } else if (shoot_control.shoot_mode == SHOOT_START) {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    } else if (shoot_control.shoot_mode == SHOOT_READY) {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    } else if (shoot_control.shoot_mode == SHOOT_BULLET) {
//        shoot_control.angle_set = shoot_control.angle;
        shoot_control.trigger_motor_speed_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_speed_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    } else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET) {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
//        trigger_motor_turn_back();
        shoot_control.speed_set = shoot_control.trigger_speed_set;
//        shoot_control.angle_set = shoot_control.angle;
    } else if (shoot_control.shoot_mode == SHOOT_DONE) {
        shoot_control.speed_set = 0.0f;
        shoot_control.angle_set = shoot_control.angle;
    }
    if (PID_AUTO_TUNE) {
        if (shoot_control.pid_auto_tune_data_point->tune_type == ANGLE_TO_SPEED) {
            shoot_control.speed_set = shoot_control.pid_auto_tune_data_point->control_list.trigger_control_value;
            shoot_control.current_set = ALL_PID(&shoot_control.trigger_motor_speed_pid, shoot_control.speed,
                                                shoot_control.speed_set);
            shoot_control.given_current = (int16_t) KalmanFilter(&shoot_control.Trigger_Motor_Current_Kalman_Filter,
                                                                 shoot_control.current_set);
        } else if (shoot_control.pid_auto_tune_data_point->tune_type == SPEED_TO_CURRENT) {
            shoot_control.current_set = shoot_control.pid_auto_tune_data_point->control_list.trigger_control_value;
            shoot_control.given_current = (int16_t) KalmanFilter(&shoot_control.Trigger_Motor_Current_Kalman_Filter,
                                                                 shoot_control.current_set);
            gimbal_control.fric1_current_set = shoot_control.pid_auto_tune_data_point->control_list.fric1_control_value;
            gimbal_control.fric2_current_set = shoot_control.pid_auto_tune_data_point->control_list.fric2_control_value;
            gimbal_control.fric1_give_current = (int16_t) (gimbal_control.fric1_current_set);
            gimbal_control.fric2_give_current = (int16_t) (gimbal_control.fric2_current_set);
        }
    } else {

        if (shoot_control.shoot_mode == SHOOT_STOP || toe_is_error(DBUS_TOE)) {
            shoot_laser_off();
            shoot_control.current_set = 0;
            shoot_control.given_current = 0;
            shoot_control.angle_set = shoot_control.angle;
//        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
//        ramp_calc(&shoot_control.fric1_ramp, -shoot_control.pwm);
//        ramp_calc(&shoot_control.fric2_ramp, -shoot_control.pwm);
            gimbal_control.fric1_current_set = 0;
            gimbal_control.fric2_current_set = 0;
            gimbal_control.fric1_give_current = 0;
            gimbal_control.fric2_give_current = 0;
            shoot_control.fric1_speed_set = 0;
            shoot_control.fric2_speed_set = 0;
            shoot_control.block_time = 0;
            shoot_control.reverse_time = 0;


        } else {
            shoot_control.fric1_speed_set = shoot_control.fric_all_speed;
            shoot_control.fric2_speed_set = -shoot_control.fric_all_speed;
            shoot_laser_on(); //激光开启
            //计算拨弹轮电机PID
            if (shoot_control.shoot_mode != SHOOT_CONTINUE_BULLET) {
                shoot_control.speed_set = ALL_PID(&shoot_control.trigger_motor_angle_pid, shoot_control.angle,
                                                  shoot_control.angle_set);
            } else {
                shoot_control.angle_set = shoot_control.angle;
            }
            shoot_control.current_set = ALL_PID(&shoot_control.trigger_motor_speed_pid, shoot_control.speed,
                                                shoot_control.speed_set);
            shoot_control.given_current = (int16_t) KalmanFilter(&shoot_control.Trigger_Motor_Current_Kalman_Filter,
                                                                 shoot_control.current_set);
            if (shoot_control.shoot_mode < SHOOT_READY) {
                shoot_control.given_current = 0;
                shoot_control.angle_set = shoot_control.angle;
            }
//        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
//        ramp_calc(&shoot_control.fric1_ramp, shoot_control.pwm);
//        ramp_calc(&shoot_control.fric2_ramp, shoot_control.pwm);
//        gimbal_control.fric1_current_set = Cloud_IPID(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed,
//                                                      shoot_control.fric1_speed_set);
            gimbal_control.fric1_current_set = ALL_PID(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed,
                                                       shoot_control.fric1_speed_set);
            gimbal_control.fric2_current_set = ALL_PID(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed,
                                                       shoot_control.fric2_speed_set);

        }
    }

//    shoot_control.fric_pwm1 = (uint16_t) (shoot_control.fric1_ramp.out);
//    shoot_control.fric_pwm2 = (uint16_t) (shoot_control.fric2_ramp.out);
//    shoot_fric1_on(shoot_control.fric_pwm1);
//    shoot_fric2_on(shoot_control.fric_pwm2);
    gimbal_control.fric1_give_current = (int16_t) (gimbal_control.fric1_current_set);
    gimbal_control.fric2_give_current = (int16_t) (gimbal_control.fric2_current_set);
    return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void) {
    static char last_s = RC_SW_MID;
    static char last_s_switch = RC_SW_MID;
    static uint16_t key_count = 0;
//    SEGGER_RTT_WriteString(0, "inloop\r\n");
    if (PID_AUTO_TUNE) {
        shoot_control.shoot_mode = SHOOT_PID_AUTO_TUNE;
        return;
    }
    if (shoot_control.shoot_speed_referee_set == SHOOT_SPEED_30_MS) {
        shoot_control.fric_all_speed = SHOOT_SPEED_18_MS_TO_FRIC;
    } else if (shoot_control.shoot_speed_referee_set == SHOOT_SPEED_18_MS) {
        shoot_control.fric_all_speed = SHOOT_SPEED_18_MS_TO_FRIC;
    } else {
        shoot_control.fric_all_speed = SHOOT_SPEED_15_MS_TO_FRIC;
    }

#if SHOOT_TRIGGER_TURN
    if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_10_MS) {
        shoot_control.trigger_speed_set = -TRIGGER_SPEED_10;
    } else if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_15_MS) {
        shoot_control.trigger_speed_set = -TRIGGER_SPEED_15;
    } else if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_25_MS) {
        shoot_control.trigger_speed_set = -TRIGGER_SPEED_25;
    } else if(shoot_control.shoot_cooling_rate_referee_set==SHOOT_HEAT_COOLING_35_MS){
        shoot_control.trigger_speed_set = -TRIGGER_SPEED_35;
    }else if(shoot_control.shoot_cooling_rate_referee_set==SHOOT_HEAT_COOLING_40_MS){
        shoot_control.trigger_speed_set = -TRIGGER_SPEED_40;
    }else if(shoot_control.shoot_cooling_rate_referee_set==SHOOT_HEAT_COOLING_60_MS){
        shoot_control.trigger_speed_set = -TRIGGER_SPEED_60;
    }else if(shoot_control.shoot_cooling_rate_referee_set==SHOOT_HEAT_COOLING_80_MS){
        shoot_control.trigger_speed_set = -TRIGGER_SPEED_80;
    }
#else
    if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_10_MS) {
        shoot_control.trigger_speed_set = TRIGGER_SPEED_10;
    } else if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_15_MS) {
        shoot_control.trigger_speed_set = TRIGGER_SPEED_15;
    } else if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_25_MS) {
        shoot_control.trigger_speed_set = TRIGGER_SPEED_25;
    } else if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_35_MS) {
        shoot_control.trigger_speed_set = TRIGGER_SPEED_35;
    } else if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_40_MS) {
        shoot_control.trigger_speed_set = TRIGGER_SPEED_40;
    } else if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_60_MS) {
        shoot_control.trigger_speed_set = TRIGGER_SPEED_60;
    } else if (shoot_control.shoot_cooling_rate_referee_set == SHOOT_HEAT_COOLING_80_MS) {
        shoot_control.trigger_speed_set = TRIGGER_SPEED_80;
    }

#endif


    //上拨判断， 一次开启，再次关闭
    if ((switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
         switch_is_up(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_R]) && !switch_is_up(last_s) &&
         !switch_is_up(last_s_switch) &&
         shoot_control.shoot_mode == SHOOT_STOP)) {
//            SEGGER_RTT_WriteString(0, "inloop\r\n");
        shoot_control.shoot_mode = SHOOT_START;
    } else if ((switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
                switch_is_up(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_R]) && !switch_is_up(last_s) &&
                !switch_is_up(last_s_switch) &&
                shoot_control.shoot_mode != SHOOT_STOP)) {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //处于左拨杆上档， 可以使用键盘开启/关闭摩擦轮
    if (switch_is_up(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
        (shoot_control.shoot_rc->key.v & SHOOT_KEYBOARD) && shoot_control.shoot_fric_state == FRIC_OFF &&
        key_count == 0) {
        key_count = 250;
        shoot_control.shoot_mode = SHOOT_START;
        shoot_control.shoot_fric_state = FRIC_ON;
    } else if (switch_is_up(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
               (shoot_control.shoot_rc->key.v & SHOOT_KEYBOARD) && shoot_control.shoot_fric_state != FRIC_OFF &&
               key_count == 0) {
        key_count = 250;
        shoot_control.shoot_mode = SHOOT_STOP;
        shoot_control.shoot_fric_state = FRIC_OFF;
    }
    if (key_count > 0) {
        key_count--;
    }
    SEGGER_RTT_printf(0, "shoot_mode:%d,shoot_fric_state:%d\r\n", shoot_control.shoot_mode,
                      shoot_control.shoot_fric_state);
//状态检测
    if (shoot_control.shoot_mode == SHOOT_START &&
        (fabsf(100 * shoot_control.fric1_speed_set - 100 * shoot_control.fric1_speed) < 0.5f * 100) &&
        (fabsf(100 * shoot_control.fric2_speed_set - 100 * shoot_control.fric2_speed) < 0.5f * 100)) {
        shoot_control.shoot_mode = SHOOT_READY;
        shoot_control.shoot_fric_state = FRIC_READY;
    } else if (shoot_control.shoot_mode == SHOOT_START) {
        shoot_control.shoot_fric_state = FRIC_ON;
    } else if (shoot_control.shoot_mode == SHOOT_STOP) {
        shoot_control.shoot_fric_state = FRIC_OFF;
    }

    if (shoot_control.shoot_mode == SHOOT_READY) {
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
             switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_R]) && !switch_is_down(last_s)) ||
            (shoot_control.press_l && shoot_control.last_press_l == 0)) {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    } else if (shoot_control.shoot_mode == SHOOT_DONE) {
        shoot_control.shoot_mode = SHOOT_READY;
    }
//    else if (shoot_control.shoot_mode == SHOOT_DONE) {
//        if (shoot_control.key == SWITCH_TRIGGER_OFF) {
//            shoot_control.key_time++;
//            if (shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME) {
//                shoot_control.key_time = 0;
//                shoot_control.shoot_mode = SHOOT_READY_BULLET;
//            }
//        } else {
//            shoot_control.key_time = 0;
//            shoot_control.shoot_mode = SHOOT_BULLET;
//        }
//    }


    if (shoot_control.shoot_mode > SHOOT_START) {
        //鼠标长按一直进入射击状态 保持连发
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) ||
            (shoot_control.rc_s_time == RC_S_LONG_TIME)) {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        } else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET) {
            shoot_control.shoot_mode = SHOOT_READY;
        }
    }
//for_test 枪口热量 服务器 屏蔽
//    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
//    if (!toe_is_error(REFEREE_RX_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit)) {
//        if (shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET) {
//            shoot_control.shoot_mode = SHOOT_READY;
//        }
//    }
    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop()) {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
    if (switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_L])) {
        last_s = shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_R];
    }
    last_s_switch = shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_R];
    switch_test = last_s_switch;//for_test
}

/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void) {
    static char last_s = RC_SW_UP;
    static float32_t speed_fliter_1 = 0.0f;
    static float32_t speed_fliter_2 = 0.0f;
    static float32_t speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const float32_t fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] +
                     (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    shoot_control.fric1_speed = (shoot_control.fric1_motor_measure->speed_rpm * PI / 30.0f);//47.123889802
    shoot_control.fric2_speed = (shoot_control.fric2_motor_measure->speed_rpm * PI / 30.0f);

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
//    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE) {
//        shoot_control.ecd_count--;
//    } else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE) {
//        shoot_control.ecd_count++;
//    }
//
//    if (shoot_control.ecd_count == FULL_COUNT) {
//        shoot_control.ecd_count = -(FULL_COUNT - 1);
//    } else if (shoot_control.ecd_count == -FULL_COUNT) {
//        shoot_control.ecd_count = FULL_COUNT - 1;
//    }

    //计算输出轴角度
    shoot_control.angle = (shoot_control.shoot_motor_measure->total_ecd) * MOTOR_ECD_TO_ANGLE;
    //微动开关
//    shoot_control.key = BUTTEN_TRIG_PIN;
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
//    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
//    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
    if (shoot_control.press_l) {
        if (shoot_control.press_l_time < PRESS_LONG_TIME) {
            shoot_control.press_l_time++;
        }
    } else {
        shoot_control.press_l_time = 0;
    }

//    if (shoot_control.press_r) {
//        if (shoot_control.press_r_time < PRESS_LONG_TIME) {
//            shoot_control.press_r_time++;
//        }
//    } else {
//        shoot_control.press_r_time = 0;
//    }

    //射击开关下档时间计时
    if (switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
        shoot_control.shoot_mode != SHOOT_STOP &&
        switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_R])) {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME) {
            shoot_control.rc_s_time++;
        }
    } else if (switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_L]) &&
               !switch_is_down(shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_R]) && switch_is_down(last_s) ||
               shoot_control.shoot_mode == SHOOT_STOP) {
        shoot_control.rc_s_time = 0;
    }
//need_to_fixed
    //鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
//    static uint16_t up_time = 0;
//    if (shoot_control.press_r) {
//        up_time = UP_ADD_TIME;
//    }
//
//    if (up_time > 0) {
//        shoot_control.fric1_ramp.max_value = FRIC_UP_PWM;
//        shoot_control.fric2_ramp.max_value = FRIC_UP_PWM;
//        up_time--;
//    } else {
//        shoot_control.fric1_ramp.max_value = FRIC_DOWN_PWM;
//        shoot_control.fric2_ramp.max_value = FRIC_DOWN_PWM;
//    }

    last_s = shoot_control.shoot_rc->rc.s[RADIO_CONTROL_SWITCH_R];

}

static void trigger_motor_turn_back(void) {
    if (shoot_control.block_time < BLOCK_TIME) {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    } else {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if (fabsf(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME) {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    } else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME) {
        shoot_control.reverse_time++;
    } else {
        shoot_control.block_time = 0;
        shoot_control.reverse_time = 0;
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void) {

    //每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0) {
//        shoot_control.angle_set = rad_format(shoot_control.angle + PI_TEN);
#if SHOOT_TRIGGER_TURN
        shoot_control.angle_set = shoot_control.angle - PI_FOUR;
#else
        shoot_control.angle_set = shoot_control.angle + PI_FOUR;
#endif
        shoot_control.move_flag = 1;
    }
//    if (shoot_control.key == SWITCH_TRIGGER_OFF) {
//
//        shoot_control.shoot_mode = SHOOT_DONE;
//    }
    //到达角度判断
//    if (fabsf(rad_format(shoot_control.angle_set - shoot_control.angle)) > 0.05f) {
    if (fabsf(shoot_control.angle_set - shoot_control.angle) > 0.2f) {
//        //没到达一直设置旋转速度
        trigger_motor_turn_back();
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    } else {
        shoot_control.shoot_mode = SHOOT_DONE;
        shoot_control.move_flag = 0;
    }
}

