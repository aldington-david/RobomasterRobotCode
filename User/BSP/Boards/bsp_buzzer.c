#include "bsp_buzzer.h"
#include "main.h"
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

