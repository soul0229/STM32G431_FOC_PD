#include <stdint.h>
#include "main.h"
#include "FocCommon.h"
#include "Motor_port_template_G431.h"
#include "ThreePhaseHall.h"

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

static const PWM_t pwmOptDefault = 
{
	.enable 	= TimerxChannel4ITEnable,
	.setPWM 	= SetTIMxUVWIChannelHighLeaveTimePWM,
	.udc		= 5.0f,
	.cycle		= 4199
};


/* adc */
static void RsSampleADCInit(void *this)
{
	ADC_t *pADC	= (ADC_t *)this;
	ADC_HandleTypeDef *phadc = pADC->priv;
	HAL_ADCEx_Calibration_Start(phadc, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(phadc, (uint32_t *)pADC->sampValue, pADC->resistor_num);
}

static bool getRsSampleValue(void *this, int16_t *currect)
{
	ADC_t *pADC	= (ADC_t *)this;
	
	currect[RESISTOR_U] = pADC->sampValue[RESISTOR_U] - pADC->offset[RESISTOR_U];
	currect[RESISTOR_V] = pADC->sampValue[RESISTOR_V] - pADC->offset[RESISTOR_V];
	currect[RESISTOR_W] = pADC->sampValue[RESISTOR_W] - pADC->offset[RESISTOR_W];

	if(pADC->Average_cnt < ADC_AVERAGE_NUM)
	{
		pADC->offset[RESISTOR_U] += pADC->sampValue[RESISTOR_U];
		pADC->offset[RESISTOR_V] += pADC->sampValue[RESISTOR_V];
		pADC->offset[RESISTOR_W] += pADC->sampValue[RESISTOR_W];

		if(++(pADC->Average_cnt) == ADC_AVERAGE_NUM)
		{
			pADC->offset[RESISTOR_U] /= ADC_AVERAGE_NUM;
			pADC->offset[RESISTOR_V] /= ADC_AVERAGE_NUM;
			pADC->offset[RESISTOR_W] /= ADC_AVERAGE_NUM;
		}

		return false;
	}
	
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
	.resistor_num				= 3,
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

	pFilter->priv_data = pLPF_data;

	return true;
}


static void LPF_Fliter_Update(void* this, int16_t *phase)
{
	RsLPF_t *pFilter 		= (RsLPF_t *)this;
	LPF_data_t *pLPF_data 	= (LPF_data_t *)pFilter->priv_data;

    pLPF_data->preValue[RESISTOR_U] = phase[RESISTOR_U] = pLPF_data->alpha * phase[RESISTOR_U] + (1.0f - pLPF_data->alpha) * pLPF_data->preValue[RESISTOR_U];
	pLPF_data->preValue[RESISTOR_V] = phase[RESISTOR_V] = pLPF_data->alpha * phase[RESISTOR_V] + (1.0f - pLPF_data->alpha) * pLPF_data->preValue[RESISTOR_V];
	pLPF_data->preValue[RESISTOR_W] = phase[RESISTOR_W] = pLPF_data->alpha * phase[RESISTOR_W] + (1.0f - pLPF_data->alpha) * pLPF_data->preValue[RESISTOR_W];

}

static const RsLPF_t RsLPFcDefaultCfg = 
{
	.init 			= LPF_Init,
	.fliter			= LPF_Fliter_Update,
	.cutOffFreq 	= 100,
	.sampFreq 		= 20000
};

extern UART_HandleTypeDef huart1;
const FocParam_t default_param = {.header  = 0x55aa,.tail = 0xaa55};
FocParam_t param_buff[PRINTF_BUF_NUM * PRINTF_SUBBUF_NUM];
FocParam_t *param = param_buff;

void foc_call_back(void *this)
{
	static uint32_t counter = 0;
	FOC_t *pFOC = this;
	// // Svpwm_t *pSvpwm     = pFOC->pSvpwm;
    RsSamp_t *pRsSamp   = pFOC->pRsSamp;
    // Sensor_t *pSensor   = pFOC->pSensor;
	// HallData_t 	*data 	= pSensor->priv_data;

	param->adc_value[RESISTOR_U] = pRsSamp->currect[RESISTOR_U];
    param->adc_value[RESISTOR_V] = pRsSamp->currect[RESISTOR_V];
    param->adc_value[RESISTOR_W] = pRsSamp->currect[RESISTOR_W];
	param->Id_out = pFOC->idPID.out;
	param->Iq_out = pFOC->iqPID.out;
	param->angle = pFOC->radian;

	// param->Id = pFOC->id;
    // param->Iq = pFOC->iq;

	// param->Id_out = pFOC->idPID.out;
    // param->Iq_out = pFOC->iqPID.out;

	// param->angle		= *pSensor->angle;
	// // param->hall_angle 	= data->trans_state[0][data->dir][data->sector];
	// // param->dir			= data->dir;
	// // param->sector		= data->sector;

	if((counter & PRINTF_SUBBUF_MASK) == (PRINTF_SUBBUF_MASK))
    {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(param - PRINTF_SUBBUF_MASK), sizeof(FocParam_t) * PRINTF_SUBBUF_NUM);
    }
    param = &param_buff[++counter & PRINTF_TOTAL_MASK];
}

void sensor_init_cb(void *this)
{
	Sensor_t 	*pSensor = (Sensor_t *)this;
	HAL_TIMEx_HallSensor_Start_IT(pSensor->priv);
}

/* init */
FOC_t *motor_init(void *tim_priv, void *adc_priv, void *sensor_priv)
{
	FOC_t *pFOC = (FOC_t*)malloc(sizeof(FOC_t));
	if(pFOC == NULL)
	{
		goto pFOC_malloc_error;
	}
	memset(pFOC, 0x00, sizeof(FOC_t));

	if(tim_priv != NULL)
	{
		PWM_t PWM_cfg = pwmOptDefault;
		PWM_cfg.priv = tim_priv;
		if(!Svpwm_register(pFOC, &PWM_cfg))
		{
			goto Svpwm_register_error;
		}
	}

	if(adc_priv != NULL)
	{
		ADC_t ADC_cfg = adcDefaultCfg;
		ADC_cfg.priv = adc_priv;
		if(!RsSamp_register(pFOC, &ADC_cfg, &RsLPFcDefaultCfg))
		{
			goto RsSamp_register_error;
		}
	}

	if(sensor_priv != NULL)
	{
		Sensor_t sensor_cfg = sensor;
		sensor_cfg.priv = sensor_priv;
		sensor_cfg.init_cb = sensor_init_cb;
		if(!Sensor_register(pFOC, &sensor_cfg, sensor_priv))
		{
			goto Sensor_register_error;
		}
	}

	if(!FOC_init(pFOC, foc_call_back))
	{
		goto FOC_init_error;
	}

	for(uint16_t cnt = 0; cnt < PRINTF_BUF_NUM * PRINTF_SUBBUF_NUM; cnt++)
    {
        param_buff[cnt] = default_param;
    }

	return pFOC;

FOC_init_error:
	Sensor_unregister(pFOC);
Sensor_register_error:
	RsSamp_unregister(pFOC);
RsSamp_register_error:
	Svpwm_unregister(pFOC);
Svpwm_register_error:
	free(pFOC);
pFOC_malloc_error:      
	return NULL;
}
