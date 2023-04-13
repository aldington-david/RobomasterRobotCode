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
#include <stdint.h>
#include "user_lib.h"
#include <math.h>
#include "struct_typedef.h"

enum PID_MODE {
    PID_POSITION = 0,
    PID_DELTA,
};

typedef struct {
    uint8_t mode;
    //PID 三参数
    float32_t Kp;
    float32_t Ki;
    float32_t Kd;

    float32_t max_out;  //最大输出
    float32_t max_iout; //最大积分输出

    float32_t set;
    float32_t get;

    float32_t out;
    float32_t Pout;
    float32_t P_On_M_out;
    float32_t Iout;
    float32_t Dout;
    float32_t Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float32_t error[3]; //误差项 0最新 1上一次 2上上次
    float32_t Integral_Separation;

    bool Variable_I;
    float32_t I_ratio;
    float32_t Variable_I_UP;
    float32_t Variable_I_Down;

    bool D_First; //微分先行
    float32_t D_Filter_Ratio;
    float32_t last_get;
    float32_t D3;
    float32_t D2;
    float32_t D1;

    bool NF_D; //不完全微分
    float32_t Dout_Last;
    float32_t D_Alpha;//0-1

    bool D_KF; //微分卡尔曼滤波
    extKalman_t D_Kalman;

    bool D_Low_Pass; //微分低通滤波
    float32_t D_Low_Pass_Factor;

    bool P_On_M; //Proportional on Measurement
    float32_t P_On_M_Ratio; //0-1

    uint32_t hal_tick
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
extern void PID_init(pid_type_def *pid, uint8_t mode, const float32_t PID[3], float32_t max_out, float32_t max_iout,
                     float32_t Integral, bool Variable_I_Switch, float32_t Variable_I_Down, float32_t Variable_I_UP,
                     bool D_First, float32_t D_Filter_Ratio, bool D_Low_Pass, float32_t D_Low_Pass_Factor, bool NF_D,
                     float32_t D_Alpha, bool D_KF, bool P_On_M, float32_t P_On_M_Ratio);

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
extern float32_t PID_calc(pid_type_def *pid, float32_t ref, float32_t set);

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

extern float ALL_PID(pid_type_def *pid, float32_t ref, float32_t set);


#endif
