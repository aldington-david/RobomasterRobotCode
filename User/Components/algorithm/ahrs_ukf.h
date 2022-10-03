//
// Created by Ken_n on 2022/10/1.
//

#ifndef ROBOMASTERROBOTCODE_AHRS_UKF_H
#define ROBOMASTERROBOTCODE_AHRS_UKF_H

#include "matrix.h"
//New_AHRS_Define
/* ================================================= UKF Variables/function declaration ================================================= */
/* UKF initialization constant */
#define P_INIT       (10.0f)
#define Rv_INIT      (1e-6f)
#define Rn_INIT_ACC  (0.0015f/10.0f)
#define Rn_INIT_MAG  (0.0015f/10.0f)

//will_move_to_other_place
#define IMU_ACC_Z0          (1)
#define COS(x)   1.0f
#define SIN(x)   0.0f
/* State machine for hard-iron bias identification or UKF running */
typedef enum {
    STATE_UKF_RUNNING = 0,
    STATE_MAGNETO_BIAS_IDENTIFICATION,
    STATE_NORTH_VECTOR_IDENTIFICATION
} STATE_AHRS;
//will_move_to_other_place
/**********************************************************************************************************************
 *  Class for Discrete Unscented Kalman Filter
 *  Ref: Van der. Merwe, .. (2004). Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic
 *      State-Space Models (Ph.D. thesis). Oregon Health & Science University.
 *
 *  The system to be estimated is defined as a discrete nonlinear dynamic dystem:
 *              x(k+1) = f[x(k), u(k)] + v(k)           ; x = Nx1,    u = Mx1
 *              y(k)   = h[x(k), u(k)] + n(k)           ; y = Zx1
 *
 *        Where:
 *          x(k) : State Variable at time-k                          : Nx1
 *          y(k) : Measured output at time-k                         : Zx1
 *          u(k) : System input at time-k                            : Mx1
 *          v(k) : Process noise, AWGN assumed, w/ covariance  Rv    : Nx1
 *          n(k) : Measurement noise, AWGN assumed, w/ covariance Rn : Nx1
 *
 *          f(..), h(..) is a nonlinear transformation of the system to be estimated.
 *
 **********************************************************************************************************************
 *      Unscented Kalman Filter algorithm:
 *          Initialization:
 *              P (k=0|k=0) = Identitas * covariant(P(k=0)), typically initialized with some big number.
 *              x(k=0|k=0)  = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
 *              Rv, Rn      = Covariance matrices of process & measurement. As this implementation
 *                              the noise as AWGN (and same value for every variable), this is set
 *                              to Rv=diag(RvInit,...,RvInit) and Rn=diag(RnInit,...,RnInit).
 *              Wc, Wm      = First order & second order weight, respectively.
 *              alpha, beta, kappa, gamma = scalar constants.
 *
 *              lambda = (alpha^2)*(N+kappa)-N,         gamma = sqrt(N+alpha)           ...{UKF_1}
 *              Wm = [lambda/(N+lambda)         1/(2(N+lambda)) ... 1/(2(N+lambda))]    ...{UKF_2}
 *              Wc = [Wm(0)+(1-alpha(^2)+beta)  1/(2(N+lambda)) ... 1/(2(N+lambda))]    ...{UKF_3}
 *
 *
 *          UKF Calculation (every sampling time):
 *              Calculate the Sigma Point:
 *                  Xs(k-1) = [x(k-1) ... x(k-1)]            ; Xs(k-1) = NxN
 *                  GPsq = gamma * sqrt(P(k-1))
 *                  XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                    ...{UKF_4}
 *
 *
 *              Unscented Transform XSigma [f,XSigma,u,Rv] -> [x,XSigma,P,DX]:
 *                  XSigma(k) = f(XSigma(k-1), u(k-1))                                  ...{UKF_5a}
 *
 *                  x(k|k-1) = sum(Wm(i) * XSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6a}
 *
 *                  DX = XSigma(k)(i) - Xs(k)   ; Xs(k) = [x(k|k-1) ... x(k|k-1)]
 *                                              ; Xs(k) = Nx(2N+1)                      ...{UKF_7a}
 *
 *                  P(k|k-1) = sum(Wc(i)*DX*DX') + Rv       ; i = 1 ... (2N+1)          ...{UKF_8a}
 *
 *
 *              Unscented Transform YSigma [h,XSigma,u,Rn] -> [y_est,YSigma,Py,DY]:
 *                  YSigma(k) = h(XSigma(k), u(k|k-1))      ; u(k|k-1) = u(k)           ...{UKF_5b}
 *
 *                  y_est(k) = sum(Wm(i) * YSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6b}
 *
 *                  DY = YSigma(k)(i) - Ys(k)   ; Ys(k) = [y_est(k) ... y_est(k)]
 *                                              ; Ys(k) = Zx(2N+1)                      ...{UKF_7b}
 *
 *                  Py(k) = sum(Wc(i)*DY*DY') + Rn          ; i = 1 ... (2N+1)          ...{UKF_8b}
 *
 *
 *              Calculate Cross-Covariance Matrix:
 *                  Pxy(k) = sum(Wc(i)*DX*DY(i))            ; i = 1 ... (2N+1)          ...{UKF_9}
 *
 *
 *              Calculate the Kalman Gain:
 *                  K           = Pxy(k) * (Py(k)^-1)                                   ...{UKF_10}
 *
 *
 *              Update the Estimated State Variable:
 *                  x(k|k)      = x(k|k-1) + K * (y(k) - y_est(k))                      ...{UKF_11}
 *
 *
 *              Update the Covariance Matrix:
 *                  P(k|k)      = P(k|k-1) - K*Py(k)*K'                                 ...{UKF_12}
 *
 *
 *        *Additional Information:
 *              - Dengan asumsi masukan plant ZOH, u(k) = u(k|k-1),
 *                  Dengan asumsi tambahan observer dijalankan sebelum pengendali, u(k|k-1) = u(k-1),
 *                  sehingga u(k) [untuk perhitungan kalman] adalah nilai u(k-1) [dari pengendali].
 *              - Notasi yang benar adalah u(k|k-1), tapi disini menggunakan notasi u(k) untuk
 *                  menyederhanakan penulisan rumus.
 *              - Pada contoh di atas X~(k=0|k=0) = [0]. Untuk mempercepat konvergensi bisa digunakan
 *                  informasi plant-spesific. Misal pada implementasi Kalman Filter untuk sensor
 *                  IMU (Inertial measurement unit) dengan X = [quaternion], dengan asumsi IMU
 *                  awalnya menghadap ke atas tanpa rotasi, X~(k=0|k=0) = [1, 0, 0, 0]'
 *
 * See https://github.com/pronenewbits for more!
 **********************************************************************************************************************/

#if (MATRIX_MAXIMUM_SIZE < (2 * SS_X_LEN + 1))
#error("MATRIX_MAXIMUM_SIZE is not big enough for UKF (need at least (2*SS_X_LEN + 1))");
#endif
#if ((MATRIX_MAXIMUM_SIZE < SS_U_LEN) || (MATRIX_MAXIMUM_SIZE < SS_X_LEN) || (MATRIX_MAXIMUM_SIZE < SS_Z_LEN))
#error("MATRIX_MAXIMUM_SIZE is not big enough for UKF (need at least SS_U_LEN / SS_X_LEN / SS_Z_LEN)");
#endif
typedef struct {
    matrix_f32_t IMU_MAG_B0;
    matrix_f32_t HARD_IRON_BIAS;
} AHRS_t;

typedef struct {
    matrix_f32_t X_Est;
    matrix_f32_t X_Sigma;

    matrix_f32_t Y_Est;
    matrix_f32_t Y_Sigma;

    matrix_f32_t P;
    matrix_f32_t P_Chol;

    matrix_f32_t DX;
    matrix_f32_t DY;

    matrix_f32_t Py;
    matrix_f32_t Pxy;

    matrix_f32_t Wm;
    matrix_f32_t Wc;

    matrix_f32_t Rv;
    matrix_f32_t Rn;

    matrix_f32_t Err;
    matrix_f32_t Gain;
    float32_t Gamma;

    bool (*AHRS_bUpdateNonlinearX)(matrix_f32_t *X_Next, matrix_f32_t *X, matrix_f32_t *U, AHRS_t *AHRS_op);

    bool (*AHRS_bUpdateNonlinearY)(matrix_f32_t *Y, matrix_f32_t *X, matrix_f32_t *U, AHRS_t *AHRS_op);
} UKF_t;


extern float32_t RLS_lambda;
extern matrix_f32_t RLS_theta;
extern matrix_f32_t RLS_P;
extern matrix_f32_t RLS_in;
extern matrix_f32_t RLS_out;
extern matrix_f32_t RLS_gain;
extern uint32_t RLS_u32iterData;
extern matrix_f32_t UKF_PINIT;
extern matrix_f32_t UKF_Rv;
extern matrix_f32_t UKF_Rn;
extern matrix_f32_t quaternionData;
extern matrix_f32_t Y;
extern matrix_f32_t U;
extern UKF_t UKF_IMU;

extern void UKF_init(UKF_t *UKF_op, matrix_f32_t *XInit, matrix_f32_t *P, matrix_f32_t *Rv, matrix_f32_t *Rn,
                     bool (*bNonlinearUpdateX)(matrix_f32_t *, matrix_f32_t *, matrix_f32_t *, AHRS_t *),
                     bool (*bNonlinearUpdateY)(matrix_f32_t *, matrix_f32_t *, matrix_f32_t *, AHRS_t *));

extern void UKF_vReset(UKF_t *UKF_op, matrix_f32_t *XInit, matrix_f32_t *P, matrix_f32_t *Rv, matrix_f32_t *Rn);

extern bool UKF_bUpdate(UKF_t *UKF_op, matrix_f32_t *Y_matrix, matrix_f32_t *U_matrix, AHRS_t *AHRS_op);

extern bool UKF_bCalculateSigmaPoint(UKF_t *UKF_op);

extern bool
UKF_bUnscentedTransform(UKF_t *UKF_op, matrix_f32_t *Out, matrix_f32_t *OutSigma, matrix_f32_t *P, matrix_f32_t *DSig,
                        bool (*_vFuncNonLinear)(matrix_f32_t *xOut, matrix_f32_t *xInp, matrix_f32_t *U,
                                                AHRS_t *AHRS_op),
                        matrix_f32_t *InpSigma, matrix_f32_t *InpVector,
                        matrix_f32_t *_CovNoise, AHRS_t *AHRS_op);

extern void NEWAHRS_init(AHRS_t *AHRS_op);

extern bool AHRS_bUpdateNonlinearX(matrix_f32_t *X_Next, matrix_f32_t *X_matrix, matrix_f32_t *U_matrix, AHRS_t *AHRS_op);

extern bool AHRS_bUpdateNonlinearY(matrix_f32_t *Y_matrix, matrix_f32_t *X_matrix, matrix_f32_t *U_matrix, AHRS_t *AHRS_op);
#endif //ROBOMASTERROBOTCODE_AHRS_UKF_H
