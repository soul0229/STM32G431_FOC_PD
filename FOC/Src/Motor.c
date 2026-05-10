#include "FOC.h"
#include "Svpwm.h"
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stdint.h"
#include "stdbool.h"

const FocParam default_param = {.header  = {0xaa},.tail = {0xcc}};
FocParam param_buff[PRINTF_BUF_NUM][PRINTF_SUBBUF_NUM];
FocParam *param = param_buff[0];

static const FOC_t FocDefault = {
	.rNum 		= 3,
	.idPID.kp 	= 0.0008f,
	.idPID.ki 	= 0.0004f,

	.iqPID.kp 	= 0.0008f,
	.iqPID.ki 	= 0.0004f,
	.iqPID.out 	= 0,
};

static void SetTIM1Channel1HighLeaveTime_us(void *priv, int16_t time)
{
	TIM_HandleTypeDef *htim = priv;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL1, time);
}

static void SetTIM1Channel2HighLeaveTime_us(void *priv, int16_t time)
{
	TIM_HandleTypeDef *htim = priv;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL2, time);
}

static void SetTIM1Channel3HighLeaveTime_us(void *priv, int16_t time)
{
	TIM_HandleTypeDef *htim = priv;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL3, time);
}

static void SetTIM1Channel4HighLeaveTime_us(void *priv, int16_t time)
{
	TIM_HandleTypeDef *htim = priv;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL4, time);
}


static void TimerxChannel4ITEnable(void *priv, bool isEnable)
{
	TIM_HandleTypeDef *htim = priv;
	
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

static void Motor_Init(void *this)
{
	PWM_Opt *pPWM = &((Svpwm_t*)this)->pwm_opts;

	pPWM->SetPWM[PHASE_U](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_V](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_W](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_INT](pPWM->priv, 2000);
	pPWM->enable(pPWM->priv, false);
}

static const PWM_Opt pwmOptDefault = 
{
	.init 				= Motor_Init,
	.enable 			= TimerxChannel4ITEnable,
	.SetPWM[PHASE_U] 	= SetTIM1Channel1HighLeaveTime_us,
	.SetPWM[PHASE_V] 	= SetTIM1Channel2HighLeaveTime_us,
	.SetPWM[PHASE_W] 	= SetTIM1Channel3HighLeaveTime_us,
	.SetPWM[PHASE_INT] 	= SetTIM1Channel4HighLeaveTime_us,
};

static uint16_t getADCSampleValueU(void *priv)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)priv,ADC_INJECTED_RANK_1);
}

static uint16_t getADCSampleValueV(void *priv)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)priv,ADC_INJECTED_RANK_2);
}

static uint16_t getADCSampleValueW(void *priv)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)priv,ADC_INJECTED_RANK_3);
}

static void Motor_Adc_Init(void *priv)
{
	HAL_ADCEx_Calibration_Start((ADC_HandleTypeDef *)priv, ADC_SINGLE_ENDED);
  	HAL_ADCEx_InjectedStart_IT((ADC_HandleTypeDef *)priv);
}

static const ADC_t adcDefaultCfg = 
{
	.ADC_Init 			= Motor_Adc_Init,
	.getADCSample[0] 	= getADCSampleValueU,
	.getADCSample[1] 	= getADCSampleValueV,
	.getADCSample[2] 	= getADCSampleValueW,
	.adcRefVolt 		= 3.4,
	.amplifier 			= 11,
	.mOhm 				= 50,
};

FOC_t *motor_init(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc)
{
	Svpwm_t *pSvpwm = Svpwm_init((PWM_Opt*)&pwmOptDefault, htim);
    if(!pSvpwm)
    {
        goto Svpwm_init_error;
    }

	ADC_t *pADC = malloc(sizeof(ADC_t));
	if(pADC == NULL)
	{
		goto adc_malloc_error;
	}
	*pADC = adcDefaultCfg;
	pADC->priv = hadc;

	FOC_t *pFOC = malloc(sizeof(FOC_t));
	if(pFOC == NULL)
	{
		goto foc_malloc_error;
	}
	*pFOC = FocDefault;

	if(!FOC_init(pFOC, pSvpwm, pADC))
	{
		goto foc_init_error;
	}

	for(uint8_t dim1 = 0; dim1 < PRINTF_BUF_NUM; dim1++)
    {
        for(uint8_t dim2 = 0; dim2 < PRINTF_BUF_NUM; dim2++)
        {
            param_buff[dim1][dim2] = default_param;
        }
    }
	
	return pFOC;

foc_init_error:
	free(pFOC);
foc_malloc_error:
	free(pADC);
adc_malloc_error:
	free(pSvpwm);
Svpwm_init_error:

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