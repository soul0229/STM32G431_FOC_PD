#include "FOC.h"
#include "Svpwm.h"
#include <stdlib.h>
#include "main.h"
#include "stdint.h"
#include "stdbool.h"

const FOC_Info FocDefault = {
	.rNum = 3,
	.idPID.kp = 0.0008f,
	.idPID.ki = 0.0004f,

	.iqPID.kp = 0.0008f,
	.iqPID.ki = 0.0004f,
	.iqPID.out = 3,
};

void SetTIM1Channel1HighLeaveTime_us(void *p, int16_t time)
{
	TIM_HandleTypeDef *htim = p;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL1, time);
}

void SetTIM1Channel2HighLeaveTime_us(void *p, int16_t time)
{
	TIM_HandleTypeDef *htim = p;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL2, time);
}

void SetTIM1Channel3HighLeaveTime_us(void *p, int16_t time)
{
	TIM_HandleTypeDef *htim = p;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL3, time);
}

void SetTIM1Channel4HighLeaveTime_us(void *p, int16_t time)
{
	TIM_HandleTypeDef *htim = p;
    __HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL4, time);
}

void TimerxChannel4ITEnable(void *p, bool isEnable)
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

void Motor_Init(void *p)
{
	TIM_HandleTypeDef *htim = p;
    SetTIM1Channel1HighLeaveTime_us(htim, 0);
    SetTIM1Channel2HighLeaveTime_us(htim, 0);
    SetTIM1Channel3HighLeaveTime_us(htim, 0);
    SetTIM1Channel3HighLeaveTime_us(htim, 4000);
		    
	TimerxChannel4ITEnable(htim, false);
}

pFOC_Info motor_init(TIM_HandleTypeDef *htim)
{
	PWM_Opt *p_opts = malloc(sizeof(PWM_Opt));
	if(p_opts == NULL)
	{
		return NULL;
	}

	*p_opts = (PWM_Opt){
	.enable = TimerxChannel4ITEnable,
	.init = Motor_Init,
	.SetPWM[PHASE_U] = SetTIM1Channel1HighLeaveTime_us,
	.SetPWM[PHASE_V] = SetTIM1Channel2HighLeaveTime_us,
	.SetPWM[PHASE_W] = SetTIM1Channel3HighLeaveTime_us,
	.SetPWM[INT] = SetTIM1Channel4HighLeaveTime_us,
	.private = htim,
	};

	pFOC_Info foc = malloc(sizeof(FOC_Info));
	if(foc == NULL)
	{
		free(p_opts);
		return NULL;
	}
	*foc = FocDefault;
	FOC_init(foc, p_opts);
	Motor_Init(htim);

	return foc;
}

void set_pwm(void *p, int16_t timeu, int16_t timev, int16_t timew)
{
	TIM_HandleTypeDef *htim = p;
	SetTIM1Channel1HighLeaveTime_us(htim, timeu);
	SetTIM1Channel2HighLeaveTime_us(htim, timev);
	SetTIM1Channel3HighLeaveTime_us(htim, timew);
}