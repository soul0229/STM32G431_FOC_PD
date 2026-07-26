#include <stdint.h>
#include "FocCommon.h"
#include "Motor.h"

extern const Sensor_t sensor;

/* pwm */
static void SetTIMxUVWIChannelHighLeaveTimePWM(void *priv, uint16_t *time)
{
	TIM_HandleTypeDef *htim = priv;
	htim->Instance->CCR1 = time[PHASE_U];
	htim->Instance->CCR2 = time[PHASE_V];
	htim->Instance->CCR3 = time[PHASE_W];
	htim->Instance->CCR4 = time[PHASE_INT];
    // __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, time[PHASE_U]);
	// __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, time[PHASE_V]);
	// __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, time[PHASE_W]);
	// __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, time[PHASE_INT]);
}

static void TimerxChannel4ITEnable(void *priv, bool isEnable)
{
	TIM_HandleTypeDef *htim = priv;
	
    if(isEnable) {
			HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(htim,TIM_CHANNEL_3);

			HAL_TIMEx_PWMN_Start(htim,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Start(htim,TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Start(htim,TIM_CHANNEL_3);
			
			HAL_TIM_PWM_Start_IT(htim,TIM_CHANNEL_4);
			HAL_TIM_Base_Start_IT(htim);

    } else {
			HAL_TIM_PWM_Stop(htim,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(htim,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(htim,TIM_CHANNEL_3);

			HAL_TIMEx_PWMN_Stop(htim,TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(htim,TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(htim,TIM_CHANNEL_3);
			
			HAL_TIM_PWM_Stop_IT(htim,TIM_CHANNEL_4);
			HAL_TIM_Base_Stop_IT(htim);
    }
}

static const PWM_Opt pwmOptDefault = 
{
	.enable 	= TimerxChannel4ITEnable,
	.SetPWM 	= SetTIMxUVWIChannelHighLeaveTimePWM,
	.udc		= 5.0f,
	.cycle		= 4199
};


/* adc */

static void RsSampleADCInit(void *this)
{
	ADC_HandleTypeDef *phadc = ((ADC_t *)this)->priv;
	HAL_ADCEx_Calibration_Start(phadc, ADC_SINGLE_ENDED);
  	HAL_ADCEx_InjectedStart_IT(phadc);
}

static bool getRsSampleValue(void *this, int16_t *pADCValue)
{
	ADC_t *pADC	= (ADC_t *)this;
	ADC_HandleTypeDef *phadc = pADC->priv;
	pADCValue[RESISTOR_U] = HAL_ADCEx_InjectedGetValue(phadc,ADC_INJECTED_RANK_1);
	pADCValue[RESISTOR_V] = HAL_ADCEx_InjectedGetValue(phadc,ADC_INJECTED_RANK_2);
	pADCValue[RESISTOR_W] = HAL_ADCEx_InjectedGetValue(phadc,ADC_INJECTED_RANK_3);
	if(pADC->Average_cnt < ADC_AVERAGE_NUM)
	{
		if(pADC->Average_cnt != 0)
		{
			pADC->offset[RESISTOR_U] = (pADC->offset[RESISTOR_U] + pADCValue[RESISTOR_U]) / 2;
			pADC->offset[RESISTOR_V] = (pADC->offset[RESISTOR_V] + pADCValue[RESISTOR_V]) / 2;
			pADC->offset[RESISTOR_W] = (pADC->offset[RESISTOR_W] + pADCValue[RESISTOR_W]) / 2;
		}
		else 
		{
			pADC->offset[RESISTOR_U] = pADCValue[RESISTOR_U];
			pADC->offset[RESISTOR_V] = pADCValue[RESISTOR_V];
			pADC->offset[RESISTOR_W] = pADCValue[RESISTOR_W];
		}

		pADC->Average_cnt++;
		return false;
	}

	pADCValue[RESISTOR_U] -= pADC->offset[RESISTOR_U];
	pADCValue[RESISTOR_V] -= pADC->offset[RESISTOR_V];
	pADCValue[RESISTOR_W] -= pADC->offset[RESISTOR_W];
	
	return true;
}

static const ADC_t adcDefaultCfg = 
{
	.init 						= RsSampleADCInit,
	.getSample 					= getRsSampleValue,
	.amplifier 					= 11.0f,
	.RefVolt 					= 3.3f,
	.maxValue					= 4096,
	.mOhm 						= 50,
};

typedef struct
{
	float 		alpha;
	int16_t     preValue[RESISTOR_MAX];
} LPF_data_t;

static bool LPF_Init(void* this) 
{
	RsLPF_t *pFilter = (RsLPF_t *)this;
    float RC = 1.0f / (2.0f * FLOAT_PI * pFilter->cutOffFreq);
    float Ts = 1.0f / pFilter->sampFreq;

	LPF_data_t *pLPF_data = (LPF_data_t*)malloc(sizeof(LPF_data_t));
	if(pLPF_data == NULL)
	{
		return false;
	}
	memset(pLPF_data, 0x00, sizeof(LPF_data_t));
    pLPF_data->alpha = Ts / (Ts + RC);

	return true;
}


static void LPF_Fliter_Update(void* this, int16_t *phase)
{
	RsLPF_t *pFilter 		= (RsLPF_t *)this;
	LPF_data_t *pLPF_data 	= (LPF_data_t *)pFilter->priv_data;
	int16_t output[RESISTOR_MAX];

    output[RESISTOR_U] = pLPF_data->alpha * phase[RESISTOR_U] + (1.0f - pLPF_data->alpha) * pLPF_data->preValue[RESISTOR_U];
	output[RESISTOR_V] = pLPF_data->alpha * phase[RESISTOR_V] + (1.0f - pLPF_data->alpha) * pLPF_data->preValue[RESISTOR_V];
	output[RESISTOR_W] = pLPF_data->alpha * phase[RESISTOR_W] + (1.0f - pLPF_data->alpha) * pLPF_data->preValue[RESISTOR_W];

    pLPF_data->preValue[RESISTOR_U] = phase[RESISTOR_U];
	pLPF_data->preValue[RESISTOR_V] = phase[RESISTOR_V];
	pLPF_data->preValue[RESISTOR_W] = phase[RESISTOR_W];

	phase[RESISTOR_U] = output[RESISTOR_U];
	phase[RESISTOR_V] = output[RESISTOR_V];
	phase[RESISTOR_W] = output[RESISTOR_W];
}

static const RsLPF_t RsLPFcDefaultCfg = 
{
	.init 			= LPF_Init,
	.fliter			= LPF_Fliter_Update,
	.cutOffFreq 	= 100,
	.sampFreq 		= 20000
};

/* init */
FOC_t *motor_init(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc)
{
	
	FOC_t *pFOC = (FOC_t*)malloc(sizeof(FOC_t));
	if(pFOC == NULL)
	{
		goto pFOC_malloc_error;
	}
	
	memset(pFOC, 0x00, sizeof(FOC_t));
	if(!Svpwm_register(pFOC, &pwmOptDefault, htim))
	{
		goto Svpwm_register_error;
	}

	ADC_t ADC = adcDefaultCfg;
	ADC.priv = hadc;
	if(!RsSamp_register(pFOC, &ADC, &RsLPFcDefaultCfg))
	{
		goto RsSamp_register_error;
	}

	if(!Sensor_register(pFOC, &sensor, NULL))
	{
		goto RsSamp_register_error;
	}

	if(!FOC_init(pFOC))
	{
		goto FOC_init_error;
	}

	return pFOC;

FOC_init_error:
	RsSamp_unregister(pFOC);
RsSamp_register_error:
	Svpwm_unregister(pFOC);
Svpwm_register_error:
	free(pFOC);
pFOC_malloc_error:      
	return NULL;
}
