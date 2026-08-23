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

#define FLOAT_PI                3.1415926535f
#define FLOAT_SQRT3 			1.732050808f
#define FLOAT_SQRT3_2 			0.866025404f
#define FLOAT_SQRT3_3 			0.577350269f

#define PRINTF_BUF_NUM          8
#define PRINTF_SUBBUF_NUM       32
#define PRINTF_SUBBUF_MASK      (PRINTF_SUBBUF_NUM - 1)
#define PRINTF_TOTAL_MASK       (PRINTF_SUBBUF_NUM * PRINTF_BUF_NUM - 1)

bool Sensor_register(FOC_t *pFOC, const Sensor_t *pSensor, void *priv);
bool Sensor_unregister(FOC_t *pFOC);

bool Svpwm_register(FOC_t *pFOC, const PWM_t *pPWM_opts);
bool Svpwm_unregister(FOC_t *pFOC);

bool RsSamp_register(FOC_t *pFOC, const ADC_t *pADC, const RsLPF_t *pLPF);
bool RsSamp_unregister(FOC_t *pFOC);

bool FOC_init(FOC_t *pFOC, cb_t *cb);
void Foc_Process(FOC_t *pFOC);
extern FocParam_t *param;


#endif
