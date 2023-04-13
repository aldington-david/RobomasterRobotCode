/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
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

#include "pid.h"
#include "main.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "print_task.h"
#include "shoot.h"
#include "USER_Filter.h"
#include "SEGGER_RTT.h"

//#define abs(x) ((x) > 0 ? (x) : (-x))
#define LimitMax(input, max)   \
    {                          \
        if ((input) > (max))       \
        {                      \
            (input) = (max);       \
        }                      \
        else if ((input) < -(max)) \
        {                      \
            (input) = -(max);      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @param[in]      Integral: 积分分离参数，若误差大于此值则Iout置零，不启用请设一个较大值
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const float32_t PID[3], float32_t max_out, float32_t max_iout,
              float32_t Integral, bool Variable_I_Switch, float32_t Variable_I_Down, float32_t Variable_I_UP,
              bool D_First, float32_t D_Filter_Ratio, bool D_Low_Pass, float32_t D_Low_Pass_Factor, bool NF_D,
              float32_t D_Alpha, bool D_KF, bool P_On_M, float32_t P_On_M_Ratio) {
    if (pid == NULL || PID == NULL) {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Integral_Separation = Integral;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->last_get = pid->D1 = pid->D2 = pid->D3 = pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

    pid->Variable_I = Variable_I_Switch;
    pid->Variable_I_Down = Variable_I_Down;
    pid->Variable_I_UP = Variable_I_UP;

    pid->D_First = D_First;
    pid->D_Filter_Ratio = D_Filter_Ratio;

    pid->D_Low_Pass = D_Low_Pass;
    pid->D_Low_Pass_Factor = D_Low_Pass_Factor;

    pid->NF_D = NF_D;
    pid->D_Alpha = D_Alpha;

    pid->D_KF = D_KF;

    pid->hal_tick = HAL_GetTick();

}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
float32_t PID_calc(pid_type_def *pid, float32_t ref, float32_t set) {
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->get = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION) {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    } else if (pid->mode == PID_DELTA) {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid) {
    if (pid == NULL) {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->last_get = pid->get = pid->set = 0.0f;
    pid->I_ratio = 0.0f;
}


/******************************以下是专属PID*********************************/

/**********************底盘跟随PID**********************/
int AngleTs[5] = {100, 90, 60, 30, 23};                   //角度分段阈值
float FollowKp[6] = {2500, 2700, 3500, 4800, 7000, 9000}; //跟随Kp变化值
float FollowFactor[4] = {0.75f, 1.0f, 0.7f, 0.65f};       //跟随Kp变化因子
//底盘跟随PID
float ClassisFollow_PID(positionpid_t *pid_t, float target, float measured) {
    if (fabsf(target) > AngleTs[0]) {
        pid_t->Kp = FollowKp[0];
    } else if (fabsf(target) > AngleTs[1]) {
        pid_t->Kp = FollowKp[1];
    } else if (fabsf(target) > AngleTs[2]) {
        pid_t->Kp = FollowKp[2];
    } else if (fabsf(target) > AngleTs[3]) {
        pid_t->Kp = FollowKp[3];
    } else if (fabsf(target) > AngleTs[4]) {
        pid_t->Kp = FollowKp[4];
    } else {
        pid_t->Kp = FollowKp[5];
    }

    if (measured >= 5100) {
        pid_t->Kp *= FollowFactor[0];
    } else if (measured >= 4900) {
        pid_t->Kp *= FollowFactor[1];
    } else if (measured >= 3800) {
        pid_t->Kp *= FollowFactor[2];
    } else if (measured >= 3000) {
        pid_t->Kp *= FollowFactor[3];
    } else {
        pid_t->Kp *= pid_t->Integral_Separation;
    }

    target = target * (PI / 180);

    pid_t->Target = (float) target;
    pid_t->Measured = (float) measured;
    pid_t->err = pid_t->Target;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**************************************************/

float ALL_PID(pid_type_def *pid, float32_t ref, float32_t set) {
    float32_t Dout_temp;
    if (pid == NULL) {
        return 0.0f;
    }
    if ((HAL_GetTick() - pid->hal_tick) > 90 && (HAL_GetTick() - pid->hal_tick) < 294967295) {
        pid->last_get = ref;
        pid->Iout = pid->out;
        LimitMax(pid->Iout, pid->max_iout);
    }
    pid->set = set;
    pid->get = ref;
    if ((pid == &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid) ||
        (pid == &gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid) ||
        (pid == &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid) ||
        (pid == &gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid) ||
        (pid == &chassis_move.chassis_angle_pid)) {
//        pid->error[0] = loop_fp32_constrain(set-ref,0,2*PI);
        pid->error[0] = jump_error(pid->set - pid->get, 2 * PI);
//        SEGGER_RTT_printf(0,"set=%f,ref=%f,err_fun=%f,err=%f\r\n",set,ref,pid->error[0],set-ref);
    } else {
        pid->error[0] = pid->set - pid->get;
    }
//探针
//    if (pid == &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid) {//for_test
//        sp_err = pid->error[0];
//    }
//    if (pid == &gimbal_control.gimbal_pitch_motor.gimbal_motor_gyro_pid) {//for_test
//        re_err = pid->error[0];
//        SEGGER_RTT_SetTerminal(1);
//        SEGGER_RTT_printf(0,"err=%f\r\n",pid->error[0]);
//    }
    if (pid->Variable_I) {
        if ((pid->Variable_I_Down != 0.0 || pid->Variable_I_UP != 0.0) &&
            (pid->Variable_I_UP - pid->Variable_I_Down > 0)) {
            if (fabsf(1000.0f * pid->error[0]) < (1000.0f * pid->Variable_I_Down)) {
                pid->I_ratio = 1.0f;
            } else if (fabsf(pid->error[0]) > pid->Variable_I_UP) {
                pid->I_ratio = 0.0f;
            } else {
                pid->I_ratio = ((1000.0f * pid->Variable_I_UP) - fabsf(1000.0f * pid->error[0])) /
                               ((1000.0f * pid->Variable_I_UP) - (1000.0f * pid->Variable_I_Down));
            }
        } else {
            pid->Iout += pid->Ki * (pid->error[0] + pid->error[1]) / 2.0f;
        }
        pid->Iout += pid->I_ratio * pid->Ki * (pid->error[0] + pid->error[1]) / 2.0f;
    } else {
        pid->Iout += pid->Ki * (pid->error[0] + pid->error[1]) / 2.0f; //pid->Iout += pid->Ki * pid->error[0];
    }

    if ((pid == &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid) ||
        (pid == &gimbal_control.gimbal_yaw_motor.gimbal_motor_absolute_angle_pid) ||
        (pid == &gimbal_control.gimbal_pitch_motor.gimbal_motor_relative_angle_pid) ||
        (pid == &gimbal_control.gimbal_pitch_motor.gimbal_motor_absolute_angle_pid) ||
        (pid == &chassis_move.chassis_angle_pid)) {
//        pid->error[0] = loop_fp32_constrain(set-ref,0,2*PI);
        pid->Dbuf[0] = jump_error(pid->get - pid->last_get, 2 * PI);
//        SEGGER_RTT_printf(0,"set=%f,ref=%f,err_fun=%f,err=%f\r\n",set,ref,pid->error[0],set-ref);
    } else {
        pid->Dbuf[0] = pid->get - pid->last_get;
    }
//    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];


    if (pid->D_First && !pid->NF_D) {
        float32_t D_operator = pid->D_Filter_Ratio * pid->Kd + pid->Kp;
        pid->D3 = pid->Kd / D_operator;
        pid->D2 = (pid->Kd + pid->Kp) / D_operator;
        pid->D1 = pid->D_Filter_Ratio * pid->D3;
        Dout_temp = pid->D1 * pid->Dout_Last + pid->D2 * pid->get + pid->D3 * pid->last_get;
    } else if (!pid->D_First && pid->NF_D) {
        Dout_temp = pid->Kd * (1 - pid->D_Alpha) * pid->Dbuf[0] + pid->D_Alpha * pid->Dout_Last;
    } else {
        Dout_temp = pid->Kd * pid->Dbuf[0];
    }

    if (pid->D_Low_Pass) {
        Filter_IIRLPF(Dout_temp, &pid->Dout, pid->D_Filter_Ratio);
    } else {
        Filter_IIRLPF(Dout_temp, &pid->Dout, 1.0f);
    }

    float32_t KF_temp;
    if (pid->D_Kalman.A == 1) {
        KF_temp = KalmanFilter(&pid->D_Kalman, pid->Dout);
    }
    if (pid->D_Kalman.A == 1 && pid->D_KF) {
        pid->Dout = KF_temp;
    }

    if (pid->P_On_M) {
        pid->P_On_M_out -= pid->P_On_M_Ratio * pid->Kp * pid->Dbuf[0];
        pid->Pout = (1-pid->P_On_M_Ratio)*pid->Kp * pid->error[0] + pid->P_On_M_out;
    } else {
        pid->Pout = pid->Kp * pid->error[0];
    }


//for_test
//    pid_out_probe += pid->out;
//    pid_pout_probe += pid->Pout;
//    pid_iout_probe += pid->Iout;
//    pid_dout_probe += pid->Dout;

    //积分限幅
    LimitMax(pid->Iout, pid->max_iout); //积分输出的限幅。

    if (fabsf(pid->error[0]) <= pid->Integral_Separation) {
        pid->Iout = 0;
        pid->out = (pid->Pout - pid->Dout);
    } else {
        pid->out = (pid->Pout + pid->Iout - pid->Dout);
    }

    //输出限幅
    LimitMax(pid->out, pid->max_out);

    ////for_test
//    if (pid == &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid) {
//        Dout1_test = pid->Dout;
//        KF_Dout1_test = KalmanFilter(&pid->D_Kalman, pid->Dout);
//    }
//    if (pid == &gimbal_control.gimbal_yaw_motor.gimbal_motor_relative_angle_pid) {
//        Dout2_test = pid->Dout;
//        KF_Dout2_test = KalmanFilter(&pid->D_Kalman, pid->Dout);
//        RTT_PrintWave_np(2,
//                         Dout2_test,
//                         KF_Dout2_test);
//    }


    pid->last_get = pid->get;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    pid->Dout_Last = pid->Dout;
    pid->hal_tick = HAL_GetTick();
    return pid->out;
}
/***********************************************************************/

