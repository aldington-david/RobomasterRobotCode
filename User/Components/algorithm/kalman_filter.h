#ifndef _KALMAN_FILTER_H
#define _KALMAN_FILTER_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "arm_math.h"
#include "matrix.h"

/*************一阶卡尔曼**************/
typedef struct {
    float32_t X_last; //上一时刻的最优结果  X(k-|k-1)
    float32_t X_mid;  //当前时刻的预测结果  X(k|k-1)
    float32_t X_now;  //当前时刻的最优结果  X(k|k)
    float32_t P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float32_t P_now;  //当前时刻最优结果的协方差  P(k|k)
    float32_t P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float32_t kg;     //kalman增益
    float32_t A;      //系统参数
    float32_t B;
    float32_t Q;
    float32_t R;
    float32_t H;

    float32_t V; //残差中间变量
    float32_t r;
    float32_t d;
    float32_t b;
    uint16_t k;
} extKalman_t;

void KalmanCreate(extKalman_t *p, float32_t T_Q, float32_t T_R);

float32_t KalmanFilter(extKalman_t *p, float32_t dat);

float32_t KalmanFilter_test(extKalman_t *p, float32_t dat);
/*************一阶卡尔曼 END**************/

/*************二阶卡尔曼**************/
#define mat arm_matrix_instance_f32    //float32_t
#define mat_64 arm_matrix_instance_f64 //double
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32 //浮点矩阵转置
#define mat_inv arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

#define Angle_limit 200         //角度小于50开启预测
#define PredictAngle_limit 250 //预测值限幅

#define Kf_Angle 0
#define Kf_Speed 1

typedef struct {
    float32_t raw_value;
    float32_t filtered_value[2];
    matrix_f32_t xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct {
    float32_t raw_value;
    float32_t filtered_value[2];
    float32_t xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
    float32_t P_data[4];
    float32_t AT_data[4], HT_data[4];
    float32_t A_data[4];
    float32_t H_data[4];
    float32_t Q_data[4];
    float32_t R_data[4];
} kalman_filter_init_t;

typedef struct {
    float32_t Vision_Angle; //视觉--角度
    float32_t Vision_Speed; //视觉--速度
    float32_t *Kf_result;   //卡尔曼输出值
    uint16_t Kf_Delay;  //卡尔曼延时计时

    struct {
        float32_t Predicted_Factor;   //预测比例因子
        float32_t Predicted_SpeedMin; //预测值最小速度
        float32_t Predicted_SpeedMax; //预测值最大速度
        float32_t kf_delay_open;      //卡尔曼延时开启时间
    } Parameter;
} Kalman_Data_t;

void second_order_kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);

float32_t *second_order_kalman_filter_calc(kalman_filter_t *F, float32_t signal1, float32_t signal2);
/*************二阶卡尔曼 END**************/

#endif
