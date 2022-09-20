/**
 * @file USER_Filter.c
 * @author Someone
 * @brief 
 * @version 0.1
 * @date 2021-03-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "USER_Filter.h"


/**
  * @brief  IIR低通滤波，输入与输出不能是同一个变量
  * @param[in]  *in 输入数据
  *				 LpfAttFactor 低通滤波衰减因子 Attenuation should be between 0 to 1.
  * @param[out]	*out 数据输出
  * @retval None
  */
void Filter_IIRLPF(float in, float *out, float LpfAttFactor) {
    *out = *out + LpfAttFactor * (in - *out);
}


/**
  * @brief  IIR低通滤波，输入与输出不能是同一个变量
  * @param[in]  in 输入数据
  *				 LpfAttFactor 低通滤波衰减因子 Attenuation should be between 0 to 1.
  * @param[out]	out 数据输出
  * @retval None
  */
int16_t Filter_IIRLPF_np(int16_t in, int16_t out, float LpfAttFactor) {
    out = out + LpfAttFactor * (in - out);
    return out;
}