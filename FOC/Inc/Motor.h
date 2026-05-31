#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "FocCommon.h"


extern FocParam *param;

FOC_t *motor_init(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc);
void set_pwm(void *p, int16_t timeu, int16_t timev, int16_t timew);
#endif
