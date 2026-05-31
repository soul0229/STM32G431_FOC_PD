#include "FocCommon.h"

#define FOC_ANGLE_TO_RADIN      0.01745f
#define M_OUTMAX                9.0f * 0.577f
#define M_KP                    0.0008f
#define M_KI                    0.0004f
#define M_KD                    0.0f

extern UART_HandleTypeDef huart1;
const FocParam default_param = {.header  = 0x55aa,.tail = 0xaa55};
FocParam param_buff[PRINTF_BUF_NUM * PRINTF_SUBBUF_NUM];
FocParam *param = param_buff;
uint32_t counter = 0;

// FOC_data FOCdata = DATA_TX_INIT;
// int16_t lastAdcData[RESIST_NUM] = {0, 0, 0};

// static void GetMotorPreCurrent(float *ua,float *ub,float *uc);

// static void CurrentPIControlID(PFOC_Struct pFOC)
// {
//     //获取实际值
//     pFOC->idPID.pre = pFOC->id ;
//     //获取目标值
//     pFOC->idPID.tar = pFOC->tarid;
//     //计算偏差
//     pFOC->idPID.bias = pFOC->idPID.tar - pFOC->idPID.pre;
//     //计算PID输出值
//     pFOC->idPID.out += pFOC->idPID.kp * (pFOC->idPID.bias - pFOC->idPID.lastBias) + pFOC->idPID.ki * pFOC->idPID.bias;
//     //保存偏差
//     pFOC->idPID.lastBias = pFOC->idPID.bias;

// 	pFOC->idPID.out = LIMIT(pFOC->idPID.out,-pFOC->idPID.outMax,pFOC->idPID.outMax);

// }

// static void CurrentPIControlIQ(PFOC_Struct pFOC)
// {
//     //获取实际值
//     pFOC->iqPID.pre = pFOC->iq;
//     //获取目标值
//     pFOC->iqPID.tar = pFOC->tariq;
//     //计算偏差
//     pFOC->iqPID.bias = pFOC->iqPID.tar - pFOC->iqPID.pre;
//     //计算PID输出值
//     pFOC->iqPID.out += pFOC->iqPID.kp * (pFOC->iqPID.bias - pFOC->iqPID.lastBias) + pFOC->iqPID.ki * pFOC->iqPID.bias;
//     //保存偏差
//     pFOC->iqPID.lastBias = pFOC->iqPID.bias;
	
// 	pFOC->iqPID.out = LIMIT(pFOC->iqPID.out,-pFOC->iqPID.outMax,pFOC->iqPID.outMax);
// }

// /*************************************************************
// ** Function name:       FocContorl
// ** Descriptions:        FOC控制流程
// ** Input parameters:    pFOC:结构体指针
// ** Output parameters:   None
// ** Returned value:      None
// ** Remarks:             None
// *************************************************************/
// void FocContorl(PFOC_Struct pFOC)
// {
//     //0.获取电气角度
// 		pFOC->radian +=200;
// //    GetElectricalAngle(pFOC);
//     //1计算实际电流值
//    //1.0电流重构
// 		CurrentReconstruction(pFOC);
// 		//1.1Clarke变换
// 		ClarkeTransform(pFOC);
// 		//1.2Park变换
// 		ParkTransform(pFOC);
// //    //2.做PID闭环
//     CurrentPIControlID(pFOC);
//     CurrentPIControlIQ(pFOC);
//     //3.计算输出值iα i贝塔
//     ParkAntiTransform(pFOC);
//     //4.输出SVPWM
//     pFOC->SvpwmGenerate(pFOC->ialphaSVPWM,pFOC->ibetaSVPWM);
// }

// void SetCurrentPIDTar(PFOC_Struct pFOC,float tarid,float tariq)
// {
//     pFOC->tarid = tarid;
//     pFOC->tariq = tariq;
// }

// void SetCurrentPIDParams(PFOC_Struct pFOC,float kp,float ki,float kd,float outMax)
// {
//     pFOC->idPID.kp = kp;
//     pFOC->idPID.ki = ki;
//     pFOC->idPID.kd = kd;
//     pFOC->idPID.outMax = outMax;

//     pFOC->iqPID.kp = kp;
//     pFOC->iqPID.ki = ki;
//     pFOC->iqPID.kd = kd;
//     pFOC->iqPID.outMax = outMax;
// }

// void SetFocEnable(PFOC_Struct pFOC,uint8_t isEnable)
// {
//     pFOC->isEnable = isEnable;
// 		pFOC->SetEnable(pFOC->isEnable);
// }


// void SetTarIDIQ(float id,float iq)
// {
// 	SetCurrentPIDTar(&gMotorFOC,id,iq);
// }


// #define ORDER 2
// float b[ORDER + 1] = {0.20657208f, 0.41314417f, 0.20657208f};  // 从Python生成的b系数
// float a[ORDER + 1] = {1.0f, -0.36952738f, 0.19581571f};        // 从Python生成的a系数

// // 状态数组
// float x[ORDER + 1] = {0};  // 输入历史值
// float y[ORDER + 1] = {0};  // 输出历史值

// float iir_filter(float input)
// {
//     for (int8_t i = ORDER; i > 0; --i) {
//         x[i] = x[i - 1];
//         y[i] = y[i - 1];
//     }
//     x[0] = input;
    
//     float output = 0;
//     for (int8_t i = 0; i <= ORDER; ++i) {
//         output += b[i] * x[i];
//     }
//     for (int8_t i = 1; i <= ORDER; ++i) {
//         output -= a[i] * y[i];
//     }
    
//     y[0] = output;
    
//     return output;
// }


// static void GetMotorPreCurrent(float *ua,float *ub,float *uc)
// {
// 		float value[3];
// 		value[0] = ((float)(lastAdcData[0]  + (ADC_offset[0][0] - ADC_offset[1][0]) )/2);
// 		value[1] = ((float)(lastAdcData[1]  + (ADC_offset[0][1] - ADC_offset[1][1]) )/2);
// 		value[2] = ((float)(lastAdcData[2]  + (ADC_offset[0][2] - ADC_offset[1][2]) )/2);
	
// 		*ua = value[0];
// 		FOCdata.adc[0] = value[0];
		
// 		*ub = value[1];
// 		FOCdata.adc[1] = value[1];
		
// 		*uc = value[2];
// 		FOCdata.adc[2] = value[2];
// }


// int16_t ADC_offset[2][RESIST_NUM];
// void CalculateAdcOffset(void)
// {
// 	SetTarIDIQ(0.0f,1.0f);
// 	SetCurrentPIDParams(&gMotorFOC,M_KP,M_KI,M_KD,M_OUTMAX);
// 	HAL_ADCEx_Calibration_Start(&ADC_PORT);    //ADC内部校准
// 	HAL_ADC_Start_DMA(&ADC_PORT, (uint32_t *)ADC_offset, RESIST_NUM);
// }

typedef struct {
    float alpha;
    float cutoff_freq;
    float sample_freq;
    int16_t prev_value[3];
} LPF_3Phase;
LPF_3Phase filter_ADC;
void LPF_Init(LPF_3Phase* filter, float cutoff_freq, float sample_freq) {
    filter->cutoff_freq = cutoff_freq;
    filter->sample_freq = sample_freq;

    float RC = 1.0f / (2.0f * 3.1415926535f * cutoff_freq);
    float Ts = 1.0f / sample_freq;
    filter->alpha = Ts / (Ts + RC);
    
    for (int i = 0; i < 3; i++) {
        filter->prev_value[i] = 0;
    }
}


int16_t LPF_SingleUpdate(LPF_3Phase* filter, int16_t input, int16_t phase) {
    int16_t output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_value[phase];
    filter->prev_value[phase] = output;
    return output;
}


void LPF_3PhaseUpdate(LPF_3Phase* filter, int16_t input[3], int16_t output[3]) {
    for (int i = 0; i < 3; i++) {
        output[i] = LPF_SingleUpdate(filter, input[i], i);
    }
}


static void CurrentReconstruction(FOC_t *pFOC)
{
    Svpwm_t *pSvpwm = pFOC->pSvpwm;
	// pFOC->GetPreCurrent(&pFOC->ia, &pFOC->ib, &pFOC->ic);
	if (pFOC->rNum < 3) {
        return;
    }
    switch (pSvpwm->sector) 
    {
        case SECTOR_1:
            pFOC->ia = 0.0f - pFOC->ib - pFOC->ic;
            break;
        case SECTOR_2:
            pFOC->ib = 0.0f - pFOC->ia - pFOC->ic;
            break;
        case SECTOR_3:
            pFOC->ib = 0.0f - pFOC->ia - pFOC->ic;
            break;
        case SECTOR_4:
            pFOC->ic = 0.0f - pFOC->ia - pFOC->ib;
            break;
        case SECTOR_5:
            pFOC->ic = 0.0f - pFOC->ia - pFOC->ib;
            break;
        case SECTOR_6:
            pFOC->ia = 0.0f - pFOC->ib - pFOC->ic;
            break;
        default:
            break;
    }
}

static void ClarkeTransform(FOC_t *pFOC)
{
    pFOC->iAlpha = pFOC->ia;
    pFOC->iBeta = (pFOC->ia + 2.0f * pFOC->ib) / FLOAT_SQRT3;
}

static void ParkTransform(FOC_t *pFOC)
{
    pFOC->id = pFOC->iAlpha * (float)arm_cos_q15(pFOC->radian)  + pFOC->iBeta * (float)arm_sin_q15(pFOC->radian);
    pFOC->iq = -pFOC->iAlpha * (float)arm_sin_q15(pFOC->radian)  + pFOC->iBeta * (float)arm_cos_q15(pFOC->radian);
}

static void ParkAntiTransform(FOC_t *pFOC)
{
    Svpwm_t *pSvpwm = pFOC->pSvpwm;
    pSvpwm->u_alpha = pFOC->iAlphaSVPWM = pFOC->idPID.out * (float)arm_cos_q15(pFOC->radian) / (uint16_t)32768  - pFOC->iqPID.out * (float)arm_sin_q15(pFOC->radian) / (uint16_t)32768;
    pSvpwm->u_beta = pFOC->iBetaSVPWM = pFOC->idPID.out * (float)arm_sin_q15(pFOC->radian) / (uint16_t)32768  + pFOC->iqPID.out * (float)arm_cos_q15(pFOC->radian) / (uint16_t)32768;
}

static void ADCGetPreCurrent(void *this)
{
    FOC_t *pFOC = this;
    ADC_t *pADC = pFOC->pADC;
    // ADC_t *pADC = pFOC->pADC;
    // pFOC->ia = pADC->getADCSample[0](pADC->priv);
    // pFOC->ib = pADC->getADCSample[1](pADC->priv);
    // pFOC->ic = pADC->getADCSample[2](pADC->priv);
    param->adc_value[0] = pADC->getADCSample[0](pADC->priv);
    param->adc_value[1] = pADC->getADCSample[1](pADC->priv);
    param->adc_value[2] = pADC->getADCSample[2](pADC->priv);
    // if(pADC->averageCnt < 5000)
    // {
    //     pADC->offset[0] += pFOC->ia;
    //     pADC->offset[1] += pFOC->ib;
    //     pADC->offset[2] += pFOC->ic;
    //     pADC->averageCnt++;
    //     if(pADC->averageCnt == 5000)
    //     {
    //         pADC->offset[0] /= 5000;
    //         pADC->offset[1] /= 5000;
    //         pADC->offset[2] /= 5000;
            // pFOC->isEnable = true;
            pFOC->iqPID.out = 4;
    //     }
    //     return;
    // }
    // pFOC->ia = pFOC->ia - adc_opts->offset[0];
    // pFOC->ib = pFOC->ib - adc_opts->offset[1];
    // pFOC->ic = pFOC->ic - adc_opts->offset[2];
    param->adc_value[0] = LPF_SingleUpdate(&filter_ADC, param->adc_value[0], 0);
    param->adc_value[1] = LPF_SingleUpdate(&filter_ADC, param->adc_value[1], 1);
    param->adc_value[2] = LPF_SingleUpdate(&filter_ADC, param->adc_value[2], 2);
 
}

static void MotorEnable(void *this, bool enable)
{
    PWM_Opt *pPWM =  &((FOC_t*)this)->pSvpwm->pwm_opts;
    pPWM->enable(pPWM->priv, enable);
}

bool FOC_init(FOC_t *pFOC, Svpwm_t *pSvpwm, ADC_t *pADC)
{
    if(pFOC == NULL || pSvpwm == NULL || pADC == NULL)
    {
        return false;
    }

    pFOC->pSvpwm = pSvpwm;
    pFOC->pADC = pADC;
    pFOC->GetPreCurrent = ADCGetPreCurrent;
    pFOC->EnableMotor = MotorEnable;
    
    pSvpwm->pwm_opts.init(pSvpwm);
	pADC->ADC_Init(pADC->priv);

    LPF_Init(&filter_ADC, 200, 20000);

    for(uint16_t cnt = 0; cnt < PRINTF_BUF_NUM * PRINTF_SUBBUF_NUM; cnt++)
    {
        param_buff[cnt] = default_param;
    }

    return true;
}

void FocControl(FOC_t *pFOC)
{
    if(pFOC == NULL)
    {
        return;
    }
    

    Svpwm_t *pSvpwm = pFOC->pSvpwm;
    // if(pFOC->isEnable)
    // {
        pFOC->radian = ((uint16_t)(pFOC->radian+32))&0x7fff;
        param->angle = pFOC->radian;
    // }
    // ClarkeTransform(pFOC);
    // param.adc_value[0] = atan2f(pFOC->iBeta, pFOC->iAlpha)/PI*32768;
    ParkAntiTransform(pFOC);
    pSvpwm->SvpwmControl(pSvpwm);

    // memcpy(param->PWM, pSvpwm->t_PWM, sizeof(pSvpwm->t_PWM));
    if((counter&PRINTF_SUBBUF_MASK) == (PRINTF_SUBBUF_MASK))
    {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(param - PRINTF_SUBBUF_MASK), sizeof(FocParam) * PRINTF_SUBBUF_NUM);
    }
    param = &param_buff[++counter & PRINTF_TOTAL_MASK];
}


