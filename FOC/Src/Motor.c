#include "FocCommon.h"

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
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, time);
}

static void SetTIM1Channel2HighLeaveTime_us(void *priv, int16_t time)
{
	TIM_HandleTypeDef *htim = priv;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, time);
}

static void SetTIM1Channel3HighLeaveTime_us(void *priv, int16_t time)
{
	TIM_HandleTypeDef *htim = priv;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, time);
}

static void SetTIM1Channel4HighLeaveTime_us(void *priv, int16_t time)
{
	TIM_HandleTypeDef *htim = priv;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, time);
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

static void Motor_Init(void *this)
{
	PWM_Opt *pPWM = &((Svpwm_t*)this)->pwm_opts;

	pPWM->SetPWM[PHASE_U](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_V](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_W](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_INT](pPWM->priv, 4000);
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

static uint16_t getRsSampleValueU(void *priv)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)priv,ADC_INJECTED_RANK_1);
}

static uint16_t getRsSampleValueV(void *priv)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)priv,ADC_INJECTED_RANK_2);
}

static uint16_t getRsSampleValueW(void *priv)
{
	return (uint16_t)HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)priv,ADC_INJECTED_RANK_3);
}

static void RsSampleADCInit(void *priv)
{
	HAL_ADCEx_Calibration_Start((ADC_HandleTypeDef *)priv, ADC_SINGLE_ENDED);
  	HAL_ADCEx_InjectedStart_IT((ADC_HandleTypeDef *)priv);
}

static const RsSamp_t RsSampDefaultCfg = 
{
	.init 						= RsSampleADCInit,
	.getRsSample[RESISTOR_U] 	= getRsSampleValueU,
	.getRsSample[RESISTOR_V] 	= getRsSampleValueV,
	.getRsSample[RESISTOR_W] 	= getRsSampleValueW,
	.sampRefVolt 				= 3.4,
	.amplifier 					= 11,
	.mOhm 						= 50,
};

FOC_t *motor_init(TIM_HandleTypeDef *htim, ADC_HandleTypeDef *hadc)
{
	
	FOC_t *pFOC = (FOC_t*)malloc(sizeof(FOC_t));
	if(pFOC == NULL)
	{
		goto pFOC_malloc_error;
	}

	if(!Svpwm_register(pFOC, (PWM_Opt*)&pwmOptDefault, htim))
	{
		goto Svpwm_register_error;
	}

	if(!RsSamp_register(pFOC, (RsSamp_t *)&RsSampDefaultCfg, hadc))
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

void get_adc_offset(FOC_t *pFoc)
{
	// HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);

}
