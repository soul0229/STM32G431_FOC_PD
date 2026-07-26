#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "FocCommon.h"


extern FocParam_t *param;

#define ADC_AVERAGE_NUM         (1024)

FOC_t *motor_init(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc);
#endif
