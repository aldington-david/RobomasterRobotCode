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
#include "print_task.h"
#include "shoot.h"

#define abs(x) ((x) > 0 ? (x) : (-x))
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout, fp32 Integral,
              bool Variable_I_Switch, fp32 Variable_I_Down, fp32 Variable_I_UP, bool D_First, fp32 D_Filter_Ratio,
              bool D_Low_Pass, bool NF_D, fp32 D_Alpha) {
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
    pid->last_fdb = pid->D1 = pid->D2 = pid->D3 = pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

    pid->Variable_I = Variable_I_Switch;
    pid->Variable_I_Down = Variable_I_Down;
    pid->Variable_I_UP = Variable_I_UP;

    pid->D_First = D_First;
    pid->D_Filter_Ratio = D_Filter_Ratio;

    pid->D_Low_Pass = D_Low_Pass;
    pid->NF_D = NF_D;
    pid->D_Alpha = D_Alpha;

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
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set) {
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
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
    pid->last_fdb = pid->fdb = pid->set = 0.0f;
    pid->I_ratio = pid->D_Low_Pass_Filter.out = 0.0f;
}

/************ Move pid *************/


float Incremental_PID(incrementalpid_t *pid_t, float target, float measured) {

    pid_t->Target = target;
    pid_t->Measured = measured;
    pid_t->err = pid_t->Target - pid_t->Measured;

    //	if(abs(pid_t->err)<0.1f)
    //		pid_t->err = 0.0f;
    //return 0;

    pid_t->p_out = pid_t->Kp * (pid_t->err - pid_t->err_last);
    pid_t->i_out = pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - 2.0f * pid_t->err_last + pid_t->err_beforeLast);

    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit);

    pid_t->pwm += (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_beforeLast = pid_t->err_last;
    pid_t->err_last = pid_t->err;

    return pid_t->pwm;
}

void
Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Ki, float Kd, uint32_t MaxOutput, uint32_t IntegralLimit) {
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    pid_t->MaxOutput = MaxOutput;
    pid_t->IntegralLimit = IntegralLimit;
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->pwm = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
}

void Clear_IncrementalPIDData(incrementalpid_t *pid_t) {
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_beforeLast = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}

//位置式PID算法，对偏差值进行累加积分。
float Position_PID(positionpid_t *pid_t, float target, float measured) {

    pid_t->Target = (float) target;
    pid_t->Measured = (float) measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
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

void Position_PIDInit(positionpid_t *pid_t, float Kp, float Ki, float Kd, float MaxOutput, float Integral_Separation,
                      float IntegralLimit) {
    pid_t->Kp = Kp;
    pid_t->Ki = Ki;
    pid_t->Kd = Kd;
    pid_t->MaxOutput = MaxOutput;
    pid_t->Integral_Separation = Integral_Separation;
    pid_t->IntegralLimit = IntegralLimit;
    pid_t->p_out = 0;
    pid_t->d_out = 0;
    pid_t->i_out = 0;
    pid_t->err = 0;
    pid_t->err_last = 0;
    pid_t->err_change = 0;
    pid_t->pwm = 0;
    pid_t->Measured = 0;
    pid_t->Target = 0;
}

void Clear_PositionPIDData(positionpid_t *pid_t) {
    pid_t->Target = 0;
    pid_t->Measured = 0;
    pid_t->err = 0;
    pid_t->err_change = 0;
    pid_t->err_last = 0;
    pid_t->p_out = 0;
    pid_t->i_out = 0;
    pid_t->d_out = 0;
    pid_t->pwm = 0;
}

/******************************以下是专属PID*********************************/

/**********************底盘跟随PID**********************/
int AngleTs[5] = {100, 90, 60, 30, 23};                   //角度分段阈值
float FollowKp[6] = {2500, 2700, 3500, 4800, 7000, 9000}; //跟随Kp变化值
float FollowFactor[4] = {0.75f, 1.0f, 0.7f, 0.65f};       //跟随Kp变化因子
//底盘跟随PID
float ClassisFollow_PID(positionpid_t *pid_t, float target, float measured) {
    if (fabs(target) > AngleTs[0]) {
        pid_t->Kp = FollowKp[0];
    } else if (fabs(target) > AngleTs[1]) {
        pid_t->Kp = FollowKp[1];
    } else if (fabs(target) > AngleTs[2]) {
        pid_t->Kp = FollowKp[2];
    } else if (fabs(target) > AngleTs[3]) {
        pid_t->Kp = FollowKp[3];
    } else if (fabs(target) > AngleTs[4]) {
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

//自旋+扭腰
//float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured)
//{
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//
//    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}
/******************************************************/

/**********************视觉PID**********************/

//float Vision_AutoTracPID(positionpid_t *pid_t, float target, float measured)
//{
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//
//    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}
//
///**
// * @brief 视觉YAW轴外环PID
// *
// * @param pid_t
// * @param target
// * @param measured
// */
//float Vision_YAWOPID(positionpid_t *pid_t, float target, float measured)
//{
//    if (pid_t == NULL)
//    {
//        return 0;
//    }
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//
//    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}
//
///**
// * @brief 视觉YAW轴内环PID
// *
// * @param pid_t
// * @param target
// * @param measured
// */
//float Vision_YAWIPID(positionpid_t *pid_t, float target, float measured)
//{
//    if (pid_t == NULL)
//    {
//        return 0;
//    }
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//
//    if (abs(pid_t->err) >= pid_t->Integral_Separation)
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
//    }
//    else
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//    }
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}
//
///**
// * @brief 视觉PITCH轴外环PID
// *
// * @param pid_t
// * @param target
// * @param measured
// */
//float Vision_PITCHOPID(positionpid_t *pid_t, float target, float measured)
//{
//    if (pid_t == NULL)
//    {
//        return 0;
//    }
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//
//    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}
//
///**
// * @brief 视觉PITCH轴内环PID
// *
// * @param pid_t
// * @param target
// * @param measured
// */
//float Vision_PITCHIPID(positionpid_t *pid_t, float target, float measured)
//{
//    if (pid_t == NULL)
//    {
//        return 0;
//    }
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//
//    if (abs(pid_t->err) >= pid_t->Integral_Separation)
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
//    }
//    else
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//    }
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}
/**************************************************/
/********************云台PID***********************/
/**
 * @brief 云台YAW轴外环PID
 *
 * @param pid_t
 * @param target
 * @param measured
 */
float Cloud_YAWOPID(positionpid_t *pid_t, float target, float measured) {
    if (pid_t == NULL) {
        return 0;
    }
    pid_t->Target = (float) target;
    pid_t->Measured = (float) measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->d_out = KalmanFilter(&Cloud_YAWODKalman, pid_t->d_out);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台YAW轴内环PID
 *
 * @param pid_t
 * @param target
 * @param measured
 */
float Cloud_YAWIPID(positionpid_t *pid_t, float target, float measured) {
    if (pid_t == NULL) {
        return 0;
    }
    pid_t->Target = (float) target;
    pid_t->Measured = (float) measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation) {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    } else {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台外环PID
 *
 * @param pid_t
 * @param target
 * @param measured
 */
fp32 Cloud_OPID(gimbal_PID_t *pid, fp32 set, fp32 get) {
    fp32 err;
    if (pid == NULL) {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;
    err = set - get;

    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    if (pid->D_First) {
        fp32 D_temp = pid->D_Filter_Ratio * pid->kd + pid->kp;
        pid->D3 = pid->kd / D_temp;
        pid->D2 = (pid->kd + pid->kp) / D_temp;
        pid->D1 = pid->D_Filter_Ratio * pid->D3;
        pid->Dout = pid->D1 * pid->Dout + pid->D2 * pid->get + pid->D3 * pid->last_get;
    } else {
        pid->Dout = pid->kd * (pid->err - pid->error_last);
    }
//    pid->Dout = KalmanFilter(&pid->Cloud_OCKalman, pid->Dout);
    //积分限幅
    abs_limit(&pid->Iout, pid->max_iout); //取消积分输出的限幅。

    pid->out = pid->Pout + pid->Iout + pid->Dout;

    //输出限幅
    abs_limit(&pid->out, pid->max_out);
    pid->last_get = pid->get;
    pid->error_last = err;
    return pid->out;


}

/**
 * @brief 云台内环PID
 *
 * @param pid_t
 * @param target
 * @param measured
 */
float Cloud_IPID(pid_type_def *pid, fp32 ref, fp32 set) {
    if (pid == NULL) {
        return 0.0f;
    }
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

//    if (pid == &shoot_control.fric1_motor_pid) {//for_test
//        err_test = pid->error[0];
//    }

    pid->Pout = pid->Kp * pid->error[0];
    if (pid->Variable_I) {
        if ((pid->Variable_I_Down != 0.0 || pid->Variable_I_UP != 0.0) &&
            (pid->Variable_I_UP - pid->Variable_I_Down > 0)) {
            if (fabs(1000.0 * pid->error[0]) < (1000.0 * pid->Variable_I_Down)) {
                pid->I_ratio = 1.0;
            } else if (fabs(pid->error[0]) > pid->Variable_I_UP) {
                pid->I_ratio = 0.0;
            } else {
                pid->I_ratio = ((1000.0 * pid->Variable_I_UP) - fabs(1000.0 * pid->error[0])) /
                               ((1000.0 * pid->Variable_I_UP) - (1000.0 * pid->Variable_I_Down));
            }
        } else {
            pid->Iout += pid->Ki * pid->error[0];
        }
        pid->Iout += pid->I_ratio * pid->Ki * pid->error[0];
    } else {
        pid->Iout += pid->Ki * pid->error[0];
    }
    fp32 D_temp_in;
    if (pid->D_First) {
        if (pid->D_Low_Pass) {
            first_order_filter_cali(&pid->D_Low_Pass_Filter, pid->fdb);
            D_temp_in = pid->D_Low_Pass_Filter.out;
        } else {
            D_temp_in = pid->fdb;
        }

        fp32 D_temp = pid->D_Filter_Ratio * pid->Kd + pid->Kp;
        pid->D3 = pid->Kd / D_temp;
        pid->D2 = (pid->Kd + pid->Kp) / D_temp;
        pid->D1 = pid->D_Filter_Ratio * pid->D3;
        pid->Dout = pid->D1 * pid->Dout + pid->D2 * D_temp_in + pid->D3 * pid->last_fdb;
    } else {
        if (pid->D_Low_Pass) {
            first_order_filter_cali(&pid->D_Low_Pass_Filter, pid->error[0]);
            D_temp_in = pid->D_Low_Pass_Filter.out;
        } else {
            D_temp_in = pid->error[0];
        }

        pid->Dbuf[0] = (D_temp_in - pid->error[1]);
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dout = pid->Kd * pid->Dbuf[0];
    }
    //积分限幅
    LimitMax(pid->Iout, pid->max_iout); //取消积分输出的限幅。

    if (fabs(pid->error[0]) >= pid->Integral_Separation) {
        pid->Iout = 0;
        pid->out = (pid->Pout + pid->Dout);
    } else {
        pid->out = (pid->Pout + pid->Iout + pid->Dout);
    }


    //输出限幅
    LimitMax(pid->out, pid->max_out);

    pid->last_fdb = D_temp_in;

    pid->error[2] = pid->error[1];
    pid->error[1] = D_temp_in;
    return pid->out;
}

float ALL_PID(pid_type_def *pid, fp32 ref, fp32 set) {
    if (pid == NULL) {
        return 0.0f;
    }
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    if (pid == &gimbal_control.gimbal_yaw_motor.gimbal_motor_gyro_pid) {//for_test
        err_test = pid->error[0];
    }

    pid->Pout = pid->Kp * pid->error[0];
    if (pid->Variable_I) {
        if ((pid->Variable_I_Down != 0.0 || pid->Variable_I_UP != 0.0) &&
            (pid->Variable_I_UP - pid->Variable_I_Down > 0)) {
            if (fabs(1000.0 * pid->error[0]) < (1000.0 * pid->Variable_I_Down)) {
                pid->I_ratio = 1.0;
            } else if (fabs(pid->error[0]) > pid->Variable_I_UP) {
                pid->I_ratio = 0.0;
            } else {
                pid->I_ratio = ((1000.0 * pid->Variable_I_UP) - fabs(1000.0 * pid->error[0])) /
                               ((1000.0 * pid->Variable_I_UP) - (1000.0 * pid->Variable_I_Down));
            }
        } else {
            pid->Iout += pid->Ki * (pid->error[0] + pid->error[1]) / 2.0f;
        }
        pid->Iout += pid->I_ratio * pid->Ki * (pid->error[0] + pid->error[1]) / 2.0f;
    } else {
        pid->Iout += pid->Ki * (pid->error[0] + pid->error[1]) / 2.0f;
    }


    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];


    if (pid->D_First && !pid->NF_D) {
        fp32 D_temp = pid->D_Filter_Ratio * pid->Kd + pid->Kp;
        pid->D3 = pid->Kd / D_temp;
        pid->D2 = (pid->Kd + pid->Kp) / D_temp;
        pid->D1 = pid->D_Filter_Ratio * pid->D3;
        pid->Dout = pid->D1 * pid->Dout_Last + pid->D2 * pid->fdb + pid->D3 * pid->last_fdb;
    } else if (!pid->D_First && pid->NF_D) {
        pid->Dout = pid->Kd * (1 - pid->D_Alpha) * pid->Dbuf[0] + pid->D_Alpha * pid->Dout_Last;
    } else {
        pid->Dout = pid->Kd * pid->Dbuf[0];
    }

    //积分限幅
    LimitMax(pid->Iout, pid->max_iout); //取消积分输出的限幅。

    if (fabs(pid->error[0]) >= pid->Integral_Separation) {
        pid->Iout = 0;
        pid->out = (pid->Pout + pid->Dout);
    } else {
        pid->out = (pid->Pout + pid->Iout + pid->Dout);
    }


    //输出限幅
    LimitMax(pid->out, pid->max_out);

    pid->last_fdb = pid->fdb;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];

    pid->Dout_Last = pid->Dout;
    return pid->out;
}
/***********************************************************************/

/********************云台PID***********************/
/**
 * @brief 云台YAW轴外环PID
 *
 * @param pid_t
 * @param target
 * @param measured
 */
float Cloud_IMUYAWOPID(positionpid_t *pid_t, float target, float measured) {
    if (pid_t == NULL) {
        return 0;
    }
    pid_t->Target = (float) target;
    pid_t->Measured = (float) measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->d_out = KalmanFilter(&Cloud_YAWODKalman, pid_t->d_out);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台YAW轴内环PID
 *
 * @param pid_t
 * @param target
 * @param measured
 */
float Cloud_IMUYAWIPID(positionpid_t *pid_t, float target, float measured) {
    if (pid_t == NULL) {
        return 0;
    }
    pid_t->Target = (float) target;
    pid_t->Measured = (float) measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;
    // float Absolute_Measured = measured;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
    if (abs(pid_t->err) >= pid_t->Integral_Separation) {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    } else {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台PITCH轴外环PID
 *
 * @param pid_t
 * @param target
 * @param measured
 */
float Cloud_IMUPITCHOPID(positionpid_t *pid_t, float target, float measured) {
    if (pid_t == NULL) {
        return 0;
    }
    pid_t->Target = (float) target;
    pid_t->Measured = (float) measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    pid_t->d_out = KalmanFilter(&Cloud_PITCHODKalman, pid_t->d_out);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/**
 * @brief 云台PITCH轴内环PID
 *
 * @param pid_t
 * @param target
 * @param measured
 */
float Cloud_IMUPITCHIPID(positionpid_t *pid_t, float target, float measured) {
    if (pid_t == NULL) {
        return 0;
    }
    pid_t->Target = (float) target;
    pid_t->Measured = (float) measured;
    pid_t->err = pid_t->Target - pid_t->Measured;
    pid_t->err_change = pid_t->err - pid_t->err_last;

    pid_t->p_out = pid_t->Kp * pid_t->err;
    pid_t->i_out += pid_t->Ki * pid_t->err;
    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
    //积分限幅
    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。

    if (abs(pid_t->err) >= pid_t->Integral_Separation) {
        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
    } else {
        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
    }

    //输出限幅
    abs_limit(&pid_t->pwm, pid_t->MaxOutput);

    pid_t->err_last = pid_t->err;
    return pid_t->pwm;
}

/***********************************************************************/

/****************************IMU--Vision********************************/
///**
// * @brief 云台YAW轴外环PID(视觉--IMU)
// *
// * @param pid_t
// * @param target
// * @param measured
// */
//float Cloud_VisionIMUYAWOPID(positionpid_t *pid_t, float target, float measured)
//{
//    if (pid_t == NULL)
//    {
//        return 0;
//    }
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//
//    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}

///**
// * @brief 云台YAW轴内环PID(IMU--Vision)
// *
// * @param pid_t
// * @param target
// * @param measured
// */
//float Cloud_VisionIMUYAWIPID(positionpid_t *pid_t, float target, float measured)
//{
//    if (pid_t == NULL)
//    {
//        return 0;
//    }
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//    // float Absolute_Measured = measured;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//    if (abs(pid_t->err) >= pid_t->Integral_Separation)
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
//    }
//    else
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//    }
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}

///**
// * @brief 云台PITCH轴外环PID(IMU--Vision)
// *
// * @param pid_t
// * @param target
// * @param measured
// */
//float Cloud_VisionIMUPITCHOPID(positionpid_t *pid_t, float target, float measured)
//{
//    if (pid_t == NULL)
//    {
//        return 0;
//    }
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//
//    pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}

///**
// * @brief 云台PITCH轴内环PID(IMU--Vision)
// *
// * @param pid_t
// * @param target
// * @param measured
// */
//float Cloud_VisionIMUPITCHIPID(positionpid_t *pid_t, float target, float measured)
//{
//    if (pid_t == NULL)
//    {
//        return 0;
//    }
//    pid_t->Target = (float)target;
//    pid_t->Measured = (float)measured;
//    pid_t->err = pid_t->Target - pid_t->Measured;
//    pid_t->err_change = pid_t->err - pid_t->err_last;
//
//    pid_t->p_out = pid_t->Kp * pid_t->err;
//    pid_t->i_out += pid_t->Ki * pid_t->err;
//    pid_t->d_out = pid_t->Kd * (pid_t->err - pid_t->err_last);
//    //积分限幅
//    abs_limit(&pid_t->i_out, pid_t->IntegralLimit); //取消积分输出的限幅。
//    if (abs(pid_t->err) >= pid_t->Integral_Separation)
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->d_out);
//    }
//    else
//    {
//        pid_t->pwm = (pid_t->p_out + pid_t->i_out + pid_t->d_out);
//    }
//
//    //输出限幅
//    abs_limit(&pid_t->pwm, pid_t->MaxOutput);
//
//    pid_t->err_last = pid_t->err;
//    return pid_t->pwm;
//}

