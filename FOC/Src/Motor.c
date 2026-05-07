#include "FOC.h"
#include "Svpwm.h"
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stdint.h"
#include "stdbool.h"

static const FOC_Info FocDefault = {
	.rNum = 3,
	.idPID.kp = 0.0008f,
	.idPID.ki = 0.0004f,

	.iqPID.kp = 0.0008f,
	.iqPID.ki = 0.0004f,
	.iqPID.out = 0,
};

static void SetTIM1Channel1HighLeaveTime_us(void *p, int16_t time)
{
	TIM_HandleTypeDef *htim = p;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL1, time);
}

static void SetTIM1Channel2HighLeaveTime_us(void *p, int16_t time)
{
	TIM_HandleTypeDef *htim = p;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL2, time);
}

static void SetTIM1Channel3HighLeaveTime_us(void *p, int16_t time)
{
	TIM_HandleTypeDef *htim = p;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL3, time);
}

static void SetTIM1Channel4HighLeaveTime_us(void *p, int16_t time)
{
	TIM_HandleTypeDef *htim = p;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL4, time);
}

static void TimerxChannel4ITEnable(void *p, bool isEnable)
{
	TIM_HandleTypeDef *htim = p;
    if(isEnable) {
			HAL_TIM_PWM_Start(htim,PWM_CHANNEL1);
			HAL_TIM_PWM_Start(htim,PWM_CHANNEL2);
			HAL_TIM_PWM_Start(htim,PWM_CHANNEL3);

			HAL_TIMEx_PWMN_Start(htim,PWM_CHANNEL1);
			HAL_TIMEx_PWMN_Start(htim,PWM_CHANNEL2);
			HAL_TIMEx_PWMN_Start(htim,PWM_CHANNEL3);
			
			HAL_TIM_PWM_Start_IT(htim,PWM_CHANNEL4);
			HAL_TIM_Base_Start_IT(htim);

    } else {
			HAL_TIM_PWM_Stop(htim,PWM_CHANNEL1);
			HAL_TIM_PWM_Stop(htim,PWM_CHANNEL2);
			HAL_TIM_PWM_Stop(htim,PWM_CHANNEL3);

			HAL_TIMEx_PWMN_Stop(htim,PWM_CHANNEL1);
			HAL_TIMEx_PWMN_Stop(htim,PWM_CHANNEL2);
			HAL_TIMEx_PWMN_Stop(htim,PWM_CHANNEL3);
			
			HAL_TIM_PWM_Stop_IT(htim,PWM_CHANNEL4);
			HAL_TIM_Base_Stop_IT(htim);
    }
}

static void Motor_Init(void *p)
{
	TIM_HandleTypeDef *htim = p;
    SetTIM1Channel1HighLeaveTime_us(htim, 0);
    SetTIM1Channel2HighLeaveTime_us(htim, 0);
    SetTIM1Channel3HighLeaveTime_us(htim, 0);
    SetTIM1Channel3HighLeaveTime_us(htim, 4000);
		    
	TimerxChannel4ITEnable(htim, false);
}

static const PWM_Opt pwmOptDefault = 
{
	.enable = TimerxChannel4ITEnable,
	.init = Motor_Init,
	.SetPWM[PHASE_U] = SetTIM1Channel1HighLeaveTime_us,
	.SetPWM[PHASE_V] = SetTIM1Channel2HighLeaveTime_us,
	.SetPWM[PHASE_W] = SetTIM1Channel3HighLeaveTime_us,
	.SetPWM[INT] = SetTIM1Channel4HighLeaveTime_us,
	.private = NULL
};

static uint16_t getADCSampleValueU(void *private)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)private,ADC_INJECTED_RANK_1);
}

static uint16_t getADCSampleValueV(void *private)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)private,ADC_INJECTED_RANK_2);
}

static uint16_t getADCSampleValueW(void *private)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)private,ADC_INJECTED_RANK_3);
}

static void Motor_Adc_Init(void *private)
{
	HAL_ADCEx_Calibration_Start((ADC_HandleTypeDef *)private, ADC_SINGLE_ENDED);
  	HAL_ADCEx_InjectedStart_IT((ADC_HandleTypeDef *)private);
}

static const ADC_Info adcDefaultCfg = 
{
	.adcRefVolt = 3.4,
	.amplifier = 11,
	.mOhm = 50,
	.getADCSample[0] = getADCSampleValueU,
	.getADCSample[1] = getADCSampleValueV,
	.getADCSample[2] = getADCSampleValueW,
	.ADC_Init = Motor_Adc_Init,
};

pFOC_Info motor_init(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc)
{
	PWM_Opt *p_opts = malloc(sizeof(PWM_Opt));
	if(p_opts == NULL)
	{
		goto pwm_malloc_error;
	}
	*p_opts = pwmOptDefault;
	p_opts->private = htim;

	ADC_Info *ADC_opts = malloc(sizeof(ADC_Info));
	if(ADC_opts == NULL)
	{
		goto adc_malloc_error;
	}
	*ADC_opts = adcDefaultCfg;
	ADC_opts->private = hadc;

	pFOC_Info foc = malloc(sizeof(FOC_Info));
	if(foc == NULL)
	{
		goto foc_malloc_error;
	}
	*foc = FocDefault;

	if(!FOC_init(foc, p_opts, ADC_opts))
	{
		goto foc_init_error;
	}
	p_opts->init(p_opts->private);
	ADC_opts->ADC_Init(ADC_opts->private);
	return foc;

foc_init_error:
foc_malloc_error:
	free(ADC_opts);
adc_malloc_error:
	free(p_opts);
pwm_malloc_error:
	return NULL;
}

void get_adc_offset(pFOC_Info Foc)
{
	// HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);

}

void set_pwm(void *p, int16_t timeu, int16_t timev, int16_t timew)
{
	TIM_HandleTypeDef *htim = p;
	SetTIM1Channel1HighLeaveTime_us(htim, timeu);
	SetTIM1Channel2HighLeaveTime_us(htim, timev);
	SetTIM1Channel3HighLeaveTime_us(htim, timew);
}