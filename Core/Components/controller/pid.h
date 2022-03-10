/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
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
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H

#include "kalman_filter.h"
#include "stdint-gcc.h"
#include <user_lib.h>
#include <math.h>
#include "struct_typedef.h"

enum PID_MODE {
    PID_POSITION = 0,
    PID_DELTA,
};

typedef struct {
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;
    fp32 err;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
    fp32 Integral_Separation;
    fp32 ekDeadZone;

    bool Variable_I;
    fp32 I_ratio;
    fp32 Variable_I_UP;
    fp32 Variable_I_Down;

    bool D_First;
    fp32 D_Filter_Ratio;
    fp32 last_fdb;
    fp32 D3;
    fp32 D2;
    fp32 D1;

    bool D_Low_Pass;//not_use
    first_order_filter_type_t D_Low_Pass_Filter;//not_use

    bool NF_D;
    fp32 Dout_Last;
    fp32 D_Alpha;//0-1

    lms_filter_type_t PID_lms;
    lms_filter_type_t PID_lms_2;

} pid_type_def;

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
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout, fp32 Integral,
                     bool Variable_I_Switch, fp32 Variable_I_Down, fp32 Variable_I_UP, bool D_First,
                     fp32 D_Filter_Ratio, bool D_Low_Pass, bool NF_D, fp32 D_Alpha);

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
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

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
extern void PID_clear(pid_type_def *pid);
/************ Move pid *************/

/**********PID对外数据接口************/
typedef struct incrementalpid_t {
    float Target;         //设定目标值
    float Measured;       //测量值
    float err;            //本次偏差值
    float err_last;       //上一次偏差
    float err_beforeLast; //上上次偏差
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
    float p_out;
    float i_out;
    float d_out;            //各部分输出值
    float pwm;              //pwm输出
    uint32_t MaxOutput;     //输出限幅
    uint32_t IntegralLimit; //积分限幅
    float (*Incremental_PID)(struct incrementalpid_t *pid_t, float target, float measured);
} incrementalpid_t;

typedef struct positionpid_t {
    float Target;     //设定目标值
    float Measured;   //测量值
    float err;        //本次偏差值
    float err_last;   //上一次偏差
    float err_change; //误差变化率
    float Kp;
    float Ki;
    float Kd; //Kp, Ki, Kd控制系数
    float p_out;
    float i_out;
    float d_out;               //各部分输出值
    float pwm;                 //pwm输出
    float MaxOutput;           //输出限幅
    float Integral_Separation; //积分分离阈值
    float IntegralLimit;       //积分限幅
} positionpid_t;

extern float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);

extern float Position_PID(positionpid_t *pid_t, float target, float measured);

//extern float ClassisTwister_PID(positionpid_t *pid_t, float target, float measured);
extern void
Incremental_PIDInit(incrementalpid_t *pid_t, float Kp, float Kd, float Ki, uint32_t MaxOutput, uint32_t IntegralLimit);

extern void Position_PIDInit(positionpid_t *pid_t, float Kp, float Kd, float Ki, float MaxOutput, float IntegralLimit,
                             float Integral_Separation);

//extern float Vision_AutoTracPID(positionpid_t *pid_t, float target, float measured);
extern float Cloud_IMUYAWOPID(positionpid_t *pid_t, float target, float measured);

extern float Cloud_IMUYAWIPID(positionpid_t *pid_t, float target, float measured);

extern float Cloud_IMUPITCHOPID(positionpid_t *pid_t, float target, float measured);

extern float Cloud_IMUPITCHIPID(positionpid_t *pid_t, float target, float measured);

extern float ClassisFollow_PID(positionpid_t *pid_t, float target, float measured);

//extern float Cloud_VisionIMUYAWOPID(positionpid_t *pid_t, float target, float measured);
//extern float Cloud_VisionIMUYAWIPID(positionpid_t *pid_t, float target, float measured);
//extern float Cloud_VisionIMUPITCHOPID(positionpid_t *pid_t, float target, float measured);
//extern float Cloud_VisionIMUPITCHIPID(positionpid_t *pid_t, float target, float measured);
extern void Clear_PositionPIDData(positionpid_t *pid_t);

extern void Clear_IncrementalPIDData(incrementalpid_t *pid_t);

extern float Cloud_YAWOPID(positionpid_t *pid_t, float target, float measured);

extern float Cloud_YAWIPID(positionpid_t *pid_t, float target, float measured);

extern float ALL_PID(pid_type_def *pid, fp32 ref, fp32 set);
//extern float Vision_YAWIPID(positionpid_t *pid_t, float target, float measured);
//extern float Vision_YAWOPID(positionpid_t *pid_t, float target, float measured);
//extern float Vision_PITCHOPID(positionpid_t *pid_t, float target, float measured);
//extern float Vision_PITCHIPID(positionpid_t *pid_t, float target, float measured);
extern extKalman_t Cloud_YAWODKalman;
extern extKalman_t Cloud_PITCHODKalman;


#endif
