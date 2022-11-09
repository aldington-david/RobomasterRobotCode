#include "bsp_buzzer.h"
#include "main.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
extern TIM_HandleTypeDef htim4;
void buzzer_on(uint16_t frequency, uint16_t volume)
{
    __HAL_TIM_PRESCALER(&htim4, frequency);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, volume);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_cali_start(void){
    buzzer_on(L1, VOLUME(100));
    osDelay(200);
    buzzer_on(M1, VOLUME(100));
    osDelay(200);
    buzzer_on(H1, VOLUME(100));
    osDelay(200);
    buzzer_off();
}

void buzzer_cali_success(void){
    buzzer_on(L5, VOLUME(100));
    osDelay(200);
    buzzer_on(M6, VOLUME(100));
    osDelay(200);
    buzzer_on(H7, VOLUME(100));
    osDelay(200);
    buzzer_off();
}

void buzzer_cali_fail(void){
    buzzer_on(H7, VOLUME(100));
    osDelay(200);
    buzzer_on(M4, VOLUME(100));
    osDelay(200);
    buzzer_on(L2, VOLUME(100));
    osDelay(200);
    buzzer_off();
}
