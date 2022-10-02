//
// Created by Ken_n on 2022/10/1.
//

#include "ahrs_ukf.h"
#include "matrix.h"
/* =============================================== Sharing Variables/function declaration =============================================== */
/* Gravity vector constant (align with global Z-axis) */
/* Magnetic vector constant (align with local magnetic vector) */
float32_t IMU_MAG_B0_data[3] = {COS(0), SIN(0), 0.000000f};
/* The hard-magnet bias */
float32_t HARD_IRON_BIAS_data[3] = {8.832973f, 7.243323f, 23.95714f};





void UKF_init(UKF_t *UKF_op, matrix_f32_t *XInit, matrix_f32_t *P, matrix_f32_t *Rv, matrix_f32_t *Rn,
              bool (*bNonlinearUpdateX)(matrix_f32_t *, matrix_f32_t *, matrix_f32_t *),
              bool (*bNonlinearUpdateY)(matrix_f32_t *, matrix_f32_t *, matrix_f32_t *)) {
    /* Initialization:
 *  P (k=0|k=0) = Identitas * covariant(P(k=0)), typically initialized with some big number.
 *  x(k=0|k=0)  = Expected value of x at time-0 (i.e. x(k=0)), typically set to zero.
 *  Rv, Rn      = Covariance matrices of process & measurement. As this implementation
 *                the noise as AWGN (and same value for every variable), this is set
 *                to Rv=diag(RvInit,...,RvInit) and Rn=diag(RnInit,...,RnInit).
 */
    UKF_op->X_Est = *XInit;
    Matrix_vCopy(&UKF_op->P, P);
    Matrix_vCopy(&UKF_op->Rv, Rv);
    Matrix_vCopy(&UKF_op->Rn, Rn);
    UKF_op->AHRS_bUpdateNonlinearX = AHRS_bUpdateNonlinearX;
    UKF_op->AHRS_bUpdateNonlinearY = AHRS_bUpdateNonlinearY;
    Matrix_nodata_creat(&UKF_op->X_Est, SS_X_LEN, 1, InitMatWithZero);
    Matrix_nodata_creat(&UKF_op->X_Sigma, SS_X_LEN, (2 * SS_X_LEN + 1), InitMatWithZero);

    Matrix_nodata_creat(&UKF_op->Y_Est, SS_Z_LEN, 1, InitMatWithZero);
    Matrix_nodata_creat(&UKF_op->Y_Sigma, SS_Z_LEN, (2 * SS_X_LEN + 1), InitMatWithZero);

//    Matrix_nodata_creat(&UKF_op->P, SS_X_LEN, SS_X_LEN, InitMatWithZero);
    Matrix_nodata_creat(&UKF_op->P_Chol, SS_X_LEN, SS_X_LEN, InitMatWithZero);

    Matrix_nodata_creat(&UKF_op->DX, SS_X_LEN, (2 * SS_X_LEN + 1), InitMatWithZero);
    Matrix_nodata_creat(&UKF_op->DY, SS_Z_LEN, (2 * SS_X_LEN + 1), InitMatWithZero);

    Matrix_nodata_creat(&UKF_op->Py, SS_Z_LEN, SS_Z_LEN, InitMatWithZero);
    Matrix_nodata_creat(&UKF_op->Pxy, SS_X_LEN, SS_Z_LEN, InitMatWithZero);

    Matrix_nodata_creat(&UKF_op->Wm, 1, (2 * SS_X_LEN + 1), InitMatWithZero);
//    Matrix_nodata_creat(&UKF_op->Wc, 1, (2 * SS_X_LEN + 1), InitMatWithZero);

//    Matrix_nodata_creat(&UKF_op->Rv, SS_X_LEN, SS_X_LEN, InitMatWithZero);
//    Matrix_nodata_creat(&UKF_op->Rn, SS_Z_LEN, SS_Z_LEN, InitMatWithZero);

    Matrix_nodata_creat(&UKF_op->Err, SS_Z_LEN, 1, InitMatWithZero);
    Matrix_nodata_creat(&UKF_op->Gain, SS_X_LEN, SS_Z_LEN, InitMatWithZero);

    /* Van der. Merwe, .. (2004). Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic State-Space Models
 * (Ph.D. thesis). Oregon Health & Science University. Page 6:
 *
 * where λ = α2(L+κ)−L is a scaling parameter. α determines the spread of the sigma points around ̄x and is usually
 * set to a small positive value (e.g. 1e−2 ≤ α ≤ 1). κ is a secondary scaling parameter which is usually set to either
 * 0 or 3−L (see [45] for details), and β is an extra degree of freedom scalar parameter used to incorporate any extra
 * prior knowledge of the distribution of x (for Gaussian distributions, β = 2 is optimal).
 */
    float32_t _alpha   = 1e-2f;
    float32_t _k       = 0.0f;
    float32_t _beta    = 2.0f;

    /* lambda = (alpha^2)*(N+kappa)-N,         gamma = sqrt(N+alpha)            ...{UKF_1} */
    float32_t _lambda  = (_alpha*_alpha)*(SS_X_LEN+_k) - SS_X_LEN;
    UKF_op->Gamma = sqrtf((SS_X_LEN + _lambda));


    /* Wm = [lambda/(N+lambda)         1/(2(N+lambda)) ... 1/(2(N+lambda))]     ...{UKF_2} */
    UKF_op->Wm.p2Data[0][0] = _lambda/(SS_X_LEN + _lambda);
    for (int16_t _i = 1; _i < UKF_op->Wm.arm_matrix.numCols; _i++) {
        UKF_op->Wm.p2Data[0][_i] = 0.5f/(SS_X_LEN + _lambda);
    }

    /* Wc = [Wm(0)+(1-alpha(^2)+beta)  1/(2(N+lambda)) ... 1/(2(N+lambda))]     ...{UKF_3} */
    Matrix_vCopy(&UKF_op->Wm,&UKF_op->Wc);
    UKF_op->Wc.p2Data[0][0] = UKF_op->Wc.p2Data[0][0] + (1.0f-(_alpha*_alpha)+_beta);
}

bool UKF_bUpdate(UKF_t *UKF_op, matrix_f32_t *Y, matrix_f32_t *U, AHRS_t *AHRS_op) {
    matrix_f32_t _temp_T;
    matrix_f32_t _temp_M;
    matrix_f32_t _temp_M2;
    /* Run once every sampling time */

    /* XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                     ...{UKF_4}  */
    if (!UKF_bCalculateSigmaPoint(UKF_op)) {
        return false;
    }


    /* Unscented Transform XSigma [f,XSigma,u,Rv] -> [x,XSigma,P,DX]:       ...{UKF_5a} - {UKF_8a} */
    if (!UKF_bUnscentedTransform(UKF_op,&UKF_op->X_Est, &UKF_op->X_Sigma, &UKF_op->P, &UKF_op->DX, UKF_op->AHRS_bUpdateNonlinearX, &UKF_op->X_Sigma, U, &UKF_op->Rv,AHRS_op)) {
        return false;
    }

    /* Unscented Transform YSigma [h,XSigma,u,Rn] -> [y_est,YSigma,Py,DY]:  ...{UKF_5b} - {UKF_8b} */
    if (!UKF_bUnscentedTransform(UKF_op,&UKF_op->Y_Est, &UKF_op->Y_Sigma, &UKF_op->Py, &UKF_op->DY, UKF_op->AHRS_bUpdateNonlinearY, &UKF_op->X_Sigma, U, &UKF_op->Rn,AHRS_op)) {
        return false;
    }


    /* Calculate Cross-Covariance Matrix:
     *  Pxy(k) = sum(Wc(i)*DX*DY(i))            ; i = 1 ... (2N+1)          ...{UKF_9}
     */
    for (int16_t _i = 0; _i < UKF_op->DX.arm_matrix.numRows; _i++) {
        for (int16_t _j = 0; _j < UKF_op->DX.arm_matrix.numCols; _j++) {
            UKF_op->DX.p2Data[_i][_j] *= UKF_op->Wc.p2Data[0][_j];
        }
    }
    Matrix_vTranspose_nsame(&UKF_op->DY,&_temp_T);
    Matrix_vmult_nsame(&UKF_op->DX,&_temp_T,&UKF_op->Pxy);


    /* Calculate the Kalman Gain:
     *  K           = Pxy(k) * (Py(k)^-1)                                   ...{UKF_10}
     */
    matrix_f32_t PyInv;
    Matrix_vInverse_nsame(&UKF_op->Py,&PyInv);
    if (!Matrix_bMatrixIsValid(&PyInv)) {
        Matrix_vSetMatrixInvalid(&PyInv);
        return false;
    }
    Matrix_vmult_nsame(&UKF_op->Pxy,&PyInv,&UKF_op->Gain);


    /* Update the Estimated State Variable:
     *  x(k|k)      = x(k|k-1) + K * (y(k) - y_est(k))                      ...{UKF_11}
     */
    Matrix_vsub(Y,&UKF_op->Y_Est,&UKF_op->Err);
    Matrix_vmult_nsame(&UKF_op->Gain,&UKF_op->Err,&_temp_M);
    Matrix_vadd(&UKF_op->X_Est,&_temp_M,&UKF_op->X_Est);


    /* Update the Covariance Matrix:
     *  P(k|k)      = P(k|k-1) - K*Py(k)*K'                                 ...{UKF_12}
     */
    Matrix_vTranspose_nsame(&UKF_op->Gain,&_temp_T);
    Matrix_vmult_nsame(&UKF_op->Gain,&UKF_op->Py,&_temp_M);
    Matrix_vmult_nsame(&_temp_M,&_temp_T,&_temp_M2);
    Matrix_vsub(&UKF_op->P,&_temp_M2,&UKF_op->P);
    Matrix_vSetMatrixInvalid(&PyInv);
    Matrix_vSetMatrixInvalid(&_temp_T);
    Matrix_vSetMatrixInvalid(&_temp_M);
    Matrix_vSetMatrixInvalid(&_temp_M2);


    return true;
}

void UKF_vReset(UKF_t *UKF_op, matrix_f32_t *XInit, matrix_f32_t *P, matrix_f32_t *Rv, matrix_f32_t *Rn) {
    UKF_op->X_Est = *XInit;
    Matrix_vCopy(&UKF_op->P, P);
    Matrix_vCopy(&UKF_op->Rv, Rv);
    Matrix_vCopy(&UKF_op->Rn, Rn);
}

bool UKF_bCalculateSigmaPoint(UKF_t *UKF_op) {
    /* Xs(k-1) = [x(k-1) ... x(k-1)]            ; Xs(k-1) = NxN
     * GPsq = gamma * sqrt(P(k-1))
     * XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq]                     ...{UKF_4}
     */
    /* Use Cholesky Decomposition to compute sqrt(P) */
    matrix_f32_t _temp;
    Matrix_vCholeskyDec(&UKF_op->P, &UKF_op->P_Chol);
    if (!Matrix_bMatrixIsValid(&UKF_op->P_Chol)) {
        /* System Fail */
        return false;
    }
    Matrix_vscale(&UKF_op->P_Chol, UKF_op->Gamma);

    /* Xs(k-1) = [x(k-1) ... x(k-1)]            ; Xs(k-1) = NxN */
    matrix_f32_t Y_;
    Matrix_nodata_creat(&Y_, SS_X_LEN, SS_X_LEN, InitMatWithZero);
    for (int16_t _i = 0; _i < SS_X_LEN; _i++) {
        Matrix_vInsertVector(&Y_, &UKF_op->X_Est, _i, &Y_);
    }

    Matrix_vSetToZero(&UKF_op->X_Sigma);
    /* XSigma(k-1) = [x(k-1) 0 0] */
    Matrix_vInsertVector(&UKF_op->X_Sigma, &UKF_op->X_Est, 0, &UKF_op->X_Sigma);
    /* XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq  0] */
    Matrix_vadd(&Y_, &UKF_op->P_Chol, &_temp);
    Matrix_vInsertAllSubMatrix(&UKF_op->X_Sigma, &_temp, 0, 1, &UKF_op->X_Sigma);
    /* XSigma(k-1) = [x(k-1) Xs(k-1)+GPsq Xs(k-1)-GPsq] */
    Matrix_vsub(&Y_, &UKF_op->P_Chol, &_temp);
    Matrix_vInsertAllSubMatrix(&UKF_op->X_Sigma, &_temp, 0, (1 + SS_X_LEN), &UKF_op->X_Sigma);
    Matrix_vSetMatrixInvalid(&Y_);
    Matrix_vSetMatrixInvalid(&_temp);

    return true;
}

bool
UKF_bUnscentedTransform(UKF_t *UKF_op, matrix_f32_t *Out, matrix_f32_t *OutSigma, matrix_f32_t *P, matrix_f32_t *DSig,
                        bool (*_vFuncNonLinear)(matrix_f32_t *xOut, matrix_f32_t *xInp, matrix_f32_t *U,AHRS_t *AHRS_op),
                        matrix_f32_t *InpSigma, matrix_f32_t *InpVector,
                        matrix_f32_t *_CovNoise, AHRS_t *AHRS_op) {
    matrix_f32_t _temp_T;
    matrix_f32_t _temp_M;
    /* XSigma(k) = f(XSigma(k-1), u(k-1))                                  ...{UKF_5a}  */
    /* x(k|k-1) = sum(Wm(i) * XSigma(k)(i))    ; i = 1 ... (2N+1)          ...{UKF_6a}  */
    Matrix_vSetToZero(Out);
    for (int16_t _j = 0; _j < InpSigma->arm_matrix.numCols; _j++) {
        /* Transform the column submatrix of sigma-points input matrix (InpSigma) */
        matrix_f32_t _AuxSigma1;
        matrix_f32_t _AuxSigma2;
        Matrix_nodata_creat(&_AuxSigma1, InpSigma->arm_matrix.numRows, 1, InitMatWithZero);
        Matrix_nodata_creat(&_AuxSigma2, OutSigma->arm_matrix.numRows, 1, InitMatWithZero);
        for (int16_t _i = 0; _i < InpSigma->arm_matrix.numRows; _i++) {
            _AuxSigma1.p2Data[_i][0] = InpSigma->p2Data[_i][_j];
        }
        if (!_vFuncNonLinear(&_AuxSigma2, &_AuxSigma1, InpVector,AHRS_op)) {
            Matrix_vSetMatrixInvalid(&_AuxSigma1);
            Matrix_vSetMatrixInvalid(&_AuxSigma2);
            /* Somehow the transformation function is failed, propagate the error */
            return false;
        }

        /* Combine the transformed vector to construct sigma-points output matrix (OutSigma) */
        Matrix_vInsertVector(OutSigma, &_AuxSigma2, _j, OutSigma);

        /* Calculate x(k|k-1) as weighted mean of OutSigma */
        Matrix_vscale(&_AuxSigma2, UKF_op->Wm.p2Data[0][_j]);
        Matrix_vadd(Out, &_AuxSigma2, Out);
        Matrix_vSetMatrixInvalid(&_AuxSigma1);
        Matrix_vSetMatrixInvalid(&_AuxSigma2);
    }

    /* DX = XSigma(k)(i) - Xs(k)   ; Xs(k) = [x(k|k-1) ... x(k|k-1)]
     *                             ; Xs(k) = Nx(2N+1)                      ...{UKF_7a}  */
    matrix_f32_t _AuxSigma1;
    Matrix_nodata_creat(&_AuxSigma1, OutSigma->arm_matrix.numRows, OutSigma->arm_matrix.numCols, InitMatWithZero);
    for (int16_t _j = 0; _j < OutSigma->arm_matrix.numCols; _j++) {
        Matrix_vInsertVector(&_AuxSigma1, Out, _j, &_AuxSigma1);
    }
    Matrix_vsub(OutSigma, &_AuxSigma1, DSig);

    /* P(k|k-1) = sum(Wc(i)*DX*DX') + Rv       ; i = 1 ... (2N+1)          ...{UKF_8a}  */
    Matrix_vCopy(DSig, &_AuxSigma1);
    for (int16_t _i = 0; _i < DSig->arm_matrix.numRows; _i++) {
        for (int16_t _j = 0; _j < DSig->arm_matrix.numCols; _j++) {
            _AuxSigma1.p2Data[_i][_j] *= UKF_op->Wc.p2Data[0][_j];
        }
    }
    Matrix_vTranspose_nsame(DSig, &_temp_T);
    Matrix_vmult_nsame(&_AuxSigma1, &_temp_T, &_temp_M);
    Matrix_vadd(&_temp_M, _CovNoise, P);
    Matrix_vSetMatrixInvalid(&_AuxSigma1);
    Matrix_vSetMatrixInvalid(&_temp_T);
    Matrix_vSetMatrixInvalid(&_temp_M);

    return true;
}

bool AHRS_bUpdateNonlinearX(matrix_f32_t *X_Next, matrix_f32_t *X, matrix_f32_t *U, AHRS_t *AHRS_op) {
    /* Insert the nonlinear update transformation here
 *          x(k+1) = f[x(k), u(k)]
 *
 * The quaternion update function:
 *  q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
 *  q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
 *  q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
 *  q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )
 *
 * Euler method for integration:
 *  q0 = q0 + q0_dot * dT;
 *  q1 = q1 + q1_dot * dT;
 *  q2 = q2 + q2_dot * dT;
 *  q3 = q3 + q3_dot * dT;
 */
    float32_t q0, q1, q2, q3;
    float32_t p, q, r;

    q0 = X->p2Data[0][0];
    q1 = X->p2Data[1][0];
    q2 = X->p2Data[2][0];
    q3 = X->p2Data[3][0];

    p = U->p2Data[0][0];
    q = U->p2Data[1][0];
    r = U->p2Data[2][0];

    X_Next->p2Data[0][0] = (0.5f * (+0.00f - p * q1 - q * q2 - r * q3)) * SS_DT + q0;
    X_Next->p2Data[1][0] = (0.5f * (+p * q0 + 0.00f + r * q2 - q * q3)) * SS_DT + q1;
    X_Next->p2Data[2][0] = (0.5f * (+q * q0 - r * q1 + 0.00f + p * q3)) * SS_DT + q2;
    X_Next->p2Data[3][0] = (0.5f * (+r * q0 + q * q1 - p * q2 + 0.00f)) * SS_DT + q3;


    /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
    if (!Matrix_bNormVector(X_Next)) {
        /* System error, return false so we can reset the UKF */
        return false;
    }

    return true;
}

bool AHRS_bUpdateNonlinearY(matrix_f32_t *Y, matrix_f32_t *X, matrix_f32_t *U, AHRS_t *AHRS_op) {
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                        2*(q1*q2+q0*q3),                        2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3),     (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                        2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                        2*(q2*q3-q0*q1),     (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     *
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float32_t q0, q1, q2, q3;
    float32_t q0_2, q1_2, q2_2, q3_2;

    q0 = X->p2Data[0][0];
    q1 = X->p2Data[1][0];
    q2 = X->p2Data[2][0];
    q3 = X->p2Data[3][0];

    q0_2 = q0 * q0;
    q1_2 = q1 * q1;
    q2_2 = q2 * q2;
    q3_2 = q3 * q3;

    Y->p2Data[0][0] = (2 * q1 * q3 - 2 * q0 * q2) * IMU_ACC_Z0;

    Y->p2Data[1][0] = (2 * q2 * q3 + 2 * q0 * q1) * IMU_ACC_Z0;

    Y->p2Data[2][0] = (+(q0_2) - (q1_2) - (q2_2) + (q3_2)) * IMU_ACC_Z0;

    Y->p2Data[3][0] = (+(q0_2) + (q1_2) - (q2_2) - (q3_2)) * AHRS_op->IMU_MAG_B0.p2Data[0][0]
                      + (2 * (q1 * q2 + q0 * q3)) * AHRS_op->IMU_MAG_B0.p2Data[1][0]
                      + (2 * (q1 * q3 - q0 * q2)) * AHRS_op->IMU_MAG_B0.p2Data[2][0];

    Y->p2Data[4][0] = (2 * (q1 * q2 - q0 * q3)) * AHRS_op->IMU_MAG_B0.p2Data[0][0]
                      + (+(q0_2) - (q1_2) + (q2_2) - (q3_2)) * AHRS_op->IMU_MAG_B0.p2Data[1][0]
                      + (2 * (q2 * q3 + q0 * q1)) * AHRS_op->IMU_MAG_B0.p2Data[2][0];

    Y->p2Data[5][0] = (2 * (q1 * q3 + q0 * q2)) * AHRS_op->IMU_MAG_B0.p2Data[0][0]
                      + (2 * (q2 * q3 - q0 * q1)) * AHRS_op->IMU_MAG_B0.p2Data[1][0]
                      + (+(q0_2) - (q1_2) - (q2_2) + (q3_2)) * AHRS_op->IMU_MAG_B0.p2Data[2][0];
    return true;
}

//void AHRS_init(AHRS_t *AHRS_op) {
//    Matrix_data_creat(&AHRS_op->IMU_MAG_B0, 3, 1, IMU_MAG_B0_data, InitMatWithZero);
//    Matrix_data_creat(&AHRS_op->HARD_IRON_BIAS, 3, 1, HARD_IRON_BIAS_data, InitMatWithZero);
//}