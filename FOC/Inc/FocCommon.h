#ifndef __FOC_COMMON_H__
#define __FOC_COMMON_H__
#include "main.h"
#include "stdint.h"
#include "stdbool.h"

#define PWM_CHANNEL1 			TIM_CHANNEL_1
#define PWM_CHANNEL2 			TIM_CHANNEL_2
#define PWM_CHANNEL3 			TIM_CHANNEL_3
#define PWM_CHANNEL4 			TIM_CHANNEL_4

#define MAX_PWM_OUT				20000
#define LIM_PWM_OUT				20000
#define MIN_PWM_OUT				0
#define ADC_DELAY_TIM			50
#define PWM_TIMx				htim1

#define LIMIT(x,min,max)		((x>max)?max:((x<=min)?min:x))

#define PWM_LIMIT(x)			LIMIT(x,MIN_PWM_OUT,LIM_PWM_OUT)

#define FLOAT_SQRT3 			1.732050808f
#define FLOAT_SQRT3_2 			0.866025404f
#define FLOAT_SQRT3_3 			0.577350269f
#define IDIQ_MAX 				FLOAT_SQRT3_3


#define ADC_PORT				hadc1
#define ADC_FILTER_KP 			0.8f
#define RESIST_NUM 				3
#define POWER_ADC_NUM 			1
#define ADC_NUM 				(RESIST_NUM + POWER_ADC_NUM)

#define POPLE_PAIRS				4

#include "TIMxPWM.h"
// #include "sincos.h"
//#include "arm_math.h"

#endif
