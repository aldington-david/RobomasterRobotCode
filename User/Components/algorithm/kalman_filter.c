#include "kalman_filter.h"
#include "user_lib.h"
/**    
  * @author  Liu heng
  * 一阶卡尔曼滤波器来自RoboMaster论坛
  *   一维卡尔曼滤波器
  *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器
  *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
  *          使用示例
  *          extKalman p;                  //定义一个卡尔曼滤波器结构体
  *          float SersorData;             //需要进行滤波的数据
  *          KalmanCreate(&p,20,200);      //初始化该滤波器的Q=20 R=200参数
  *          while(1)
  *          {
  *             SersorData = sersor();                     //获取数据
  *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波
  *          }
  */

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  * @other 	Q：过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R：测量噪声，R增大，动态响应变慢，收敛稳定性变好
  */
void KalmanCreate(extKalman_t *p, float T_Q, float T_R) {
    p->X_last = (float) 0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;

    p->V = 0;
    p->d = 1;
    p->b = 0.98f;
    p->r = 0;
    p->k = 1;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(extKalman_t *p, float dat) {
    p->X_mid = p->A * p->X_last;                    //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A * p->P_last + p->Q;             //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid / (p->P_mid + p->R);           //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid); //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1 - p->kg) * p->P_mid;              //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                           //状态更新
    p->X_last = p->X_now;
    return p->X_now; //输出预测结果x(k|k)
}

float KalmanFilter_test(extKalman_t *p, float dat) {
    if (p->k < 65534) {
        p->k++;
    }

    p->X_mid = p->A * p->X_last;                    //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A * p->P_last + p->Q;             //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid / (p->P_mid + p->R);           //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->V = dat - p->X_mid;
    p->X_now = p->X_mid + p->kg * (p->V); //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1 - p->d) * (1 - p->kg) * p->P_mid +
               p->kg * p->R;              //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                           //状态更新
    p->X_last = p->X_now;
    p->d = (1 - p->b) / (1 - invpow(p->b, p->k));
    p->R = (1 - p->d) * p->R + p->d * (p->V * p->V - p->P_mid);
    return p->X_now; //输出预测结果x(k|k)
}

/**
  * 二阶卡尔曼滤波器
  */

void second_order_kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I) {
    Matrix_data_creat(&F->xhat,2,1,I->xhat_data,NoInitMatZero);
    Matrix_data_creat(&F->xhat, 2, 1, (float *) I->xhat_data,NoInitMatZero);
    Matrix_data_creat(&F->xhatminus, 2, 1, (float *) I->xhatminus_data,NoInitMatZero);
    Matrix_data_creat(&F->z, 2, 1, (float *) I->z_data,NoInitMatZero);
    Matrix_data_creat(&F->A, 2, 2, (float *) I->A_data,NoInitMatZero);
    Matrix_data_creat(&F->H, 2, 2, (float *) I->H_data,NoInitMatZero);
    Matrix_data_creat(&F->Q, 2, 2, (float *) I->Q_data,NoInitMatZero);
    Matrix_data_creat(&F->R, 2, 2, (float *) I->R_data,NoInitMatZero);
    Matrix_data_creat(&F->P, 2, 2, (float *) I->P_data,NoInitMatZero);
    Matrix_data_creat(&F->Pminus, 2, 2, (float *) I->Pminus_data,NoInitMatZero);
    Matrix_data_creat(&F->K, 2, 2, (float *) I->K_data,NoInitMatZero);
    Matrix_vTranspose_nsame(&F->A, &F->AT);
    Matrix_vTranspose_nsame(&F->H, &F->HT);
}

// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param 卡尔曼参数结构体
  *@param 角度
  *@param 速度
*/
float32_t *second_order_kalman_filter_calc(kalman_filter_t *F, float32_t signal1, float32_t signal2) {
    matrix_f32_t _temp_M;
    Matrix_vinit(&_temp_M);

    F->z.arm_matrix.pData[0] = signal1; //z(k)
    F->z.arm_matrix.pData[1] = signal2; //z(k)

    //1. xhat'(k)= A xhat(k-1)
    Matrix_vmult_nsame(&F->A, &F->xhat, &F->xhatminus); //  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

    //2. P'(k) = A P(k-1) AT + Q
    Matrix_vmult_nsame(&F->A, &F->P, &F->Pminus);  //   p(k|k-1) = A*p(k-1|k-1)*A'+Q
    Matrix_vmult_nsame(&F->Pminus, &F->AT, &_temp_M); //  p(k|k-1) = A*p(k-1|k-1)*A'+Q
    Matrix_vadd(&_temp_M, &F->Q, &F->Pminus);   //    p(k|k-1) = A*p(k-1|k-1)*A'+Q

    //3. K(k) = P'(k) HT / (H P'(k) HT + R)
    Matrix_vmult_nsame(&F->H, &F->Pminus, &F->K); //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    Matrix_vmult_nsame(&F->K, &F->HT, &_temp_M);     //      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    Matrix_vadd(&_temp_M, &F->R, &F->K);       //        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

    Matrix_vInverse_nsame(&F->K, &F->P);               //
    Matrix_vmult_nsame(&F->Pminus, &F->HT, &_temp_M); //
    Matrix_vmult_nsame(&_temp_M, &F->P, &F->K);       //

    //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
    Matrix_vmult_nsame(&F->H, &F->xhatminus, &_temp_M);   //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    Matrix_vsub(&F->z, &_temp_M, &F->xhat);         //            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    Matrix_vmult_nsame(&F->K, &F->xhat, &_temp_M);        //           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    Matrix_vadd(&F->xhatminus, &_temp_M, &F->xhat); //    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

    //5. P(k) = (1-K(k)H)P'(k)
    Matrix_vmult_nsame(&F->K, &F->H, &F->P); //            p(k|k) = (I-kg(k)*H)*P(k|k-1)
    Matrix_vsub(&F->Q, &F->P, &_temp_M);  //
    Matrix_vmult_nsame(&_temp_M, &F->Pminus, &F->P);

    F->filtered_value[0] = F->xhat.arm_matrix.pData[0];
    F->filtered_value[1] = F->xhat.arm_matrix.pData[1];
    return F->filtered_value;
}
