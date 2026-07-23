#ifndef __FOC_COMMON_H__
#define __FOC_COMMON_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "main.h"
#include "stm32g4xx_hal_flash.h"
#include "arm_math.h"

#include "Sensor.h"
#include "Svpwm.h"
#include "RsSamp.h"
#include "FOC.h"


// #include "sincos.h"
//#include "arm_math.h"
#define USE_SINCOS_TAB

#define MAX_PWM_OUT				20000
#define LIM_PWM_OUT				20000
#define MIN_PWM_OUT				0
#define ADC_DELAY_TIM			50

#define LIMIT(x,min,max)		((x>max)?max:((x<=min)?min:x))

#define PWM_LIMIT(x)			LIMIT(x,MIN_PWM_OUT,LIM_PWM_OUT)

#define FLOAT_SQRT3 			1.732050808f
#define FLOAT_SQRT3_2 			0.866025404f
#define FLOAT_SQRT3_3 			0.577350269f
#define IDIQ_MAX 				FLOAT_SQRT3_3

#define ADC_FILTER_KP 			0.8f
#define RESIST_NUM 				3
#define POWER_ADC_NUM 			1
#define ADC_NUM 				(RESIST_NUM + POWER_ADC_NUM)

#define POPLE_PAIRS				4

#define PRINTF_BUF_NUM          2
#define PRINTF_SUBBUF_NUM       256
#define PRINTF_SUBBUF_MASK      (PRINTF_SUBBUF_NUM - 1)
#define PRINTF_TOTAL_MASK       (PRINTF_SUBBUF_NUM * PRINTF_BUF_NUM - 1)

bool Sensor_register(FOC_t *pFOC, const Sensor_t *pSensor, void *priv);
bool Sensor_unregister(FOC_t *pFOC);

bool Svpwm_register(FOC_t *pFOC, const PWM_Opt *pPWM_opts, void *priv);
bool Svpwm_unregister(FOC_t *pFOC);

bool RsSamp_register(FOC_t *pFOC, const RsSamp_t *pRsSamp, void *priv);
bool RsSamp_unregister(FOC_t *pFOC);

bool FOC_init(FOC_t *pFOC);
void FocControl(FOC_t *pFOC);
extern FocParam_t *param;


#endif
