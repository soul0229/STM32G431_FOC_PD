#include "FocCommon.h"
#include "arm_math.h"

extern UART_HandleTypeDef huart1;
const FocParam default_param = {.header  = 0x55aa,.tail = 0xaa55};
FocParam param_buff[PRINTF_BUF_NUM * PRINTF_SUBBUF_NUM];
FocParam *param = param_buff;
uint32_t counter = 0;


static void PID_Calc(PID_t *pid)
{
    float temp;
    if (pid == NULL) {
        return;
    }

    float bias = pid->target - pid->present;
    pid->out = temp = pid->kp * bias + pid->iSum * pid->kd;

    if (pid->out > pid->outMax) {
        pid->out = pid->outMax;
    } else if (pid->out < -pid->outMax) {
        pid->out = -pid->outMax;
    }

    pid->iSum = pid->kb *(pid->out - temp) + pid->ki * bias;
}

void sin_cos_4096(uint16_t angle, float32_t *sine, float32_t *cosine);
static void ClarkeTransform(FOC_t *pFOC)
{
    pFOC->iAlpha = pFOC->ia;
    pFOC->iBeta = (pFOC->ia + 2.0f * pFOC->ib) * FLOAT_SQRT3_3;
}

static void ParkTransform(FOC_t *pFOC)
{
    float32_t sine, cosine;
    sin_cos_4096(pFOC->radian, &sine, &cosine);
    pFOC->id = pFOC->iAlpha * cosine  + pFOC->iBeta * sine;
    pFOC->iq = -pFOC->iAlpha * sine  + pFOC->iBeta * cosine;
}

static void ParkAntiTransform(FOC_t *pFOC)
{
    Svpwm_t *pSvpwm = pFOC->pSvpwm;
    float32_t sine, cosine;
    sin_cos_4096(pFOC->radian, &sine, &cosine);
    pSvpwm->u_alpha = pFOC->idPID.out * cosine - pFOC->iqPID.out * sine;
    pSvpwm->u_beta  = pFOC->idPID.out * sine + pFOC->iqPID.out * cosine;
}

static void MotorEnable(void *this, bool enable)
{
    PWM_Opt *pPWM =  &((FOC_t*)this)->pSvpwm->PwmOpts;
    pPWM->enable(pPWM->priv, enable);
}

bool FOC_init(FOC_t *pFOC)
{
    if(pFOC == NULL)
    {
        return false;
    }

    if(!pFOC->pSvpwm || !pFOC->pRsSamp)
    {
        return false;
    }

    if(pFOC->pSensor == NULL)
    {
        
    }
    memset(&pFOC->idPID, 0x00, sizeof(pFOC->idPID));
    memset(&pFOC->iqPID, 0x00, sizeof(pFOC->iqPID));
    // pFOC->GetPreCurrent = ADCGetPreCurrent;
    pFOC->rNum = 3;
    pFOC->idPID.kp = 5.000f;
    pFOC->idPID.ki = 0.0001f;
    pFOC->idPID.kd = 0.0001f;
    pFOC->idPID.outMax = 2.6f;
    pFOC->idPID.target = 0;

    pFOC->iqPID.kp = 10.0f;
    pFOC->iqPID.ki = 0.01f;
    pFOC->iqPID.kd = 0.00001f;
    pFOC->iqPID.outMax = 2.6f;
    pFOC->iqPID.target = 1.0f;

    pFOC->EnableMotor = MotorEnable;
    pFOC->pSvpwm->Init(pFOC->pSvpwm);
    pFOC->pRsSamp->Init(pFOC->pRsSamp->priv);
    
    // LPF_Init(&filter_ADC, 200, 20000);

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
    
    Svpwm_t *pSvpwm     = pFOC->pSvpwm;
    RsSamp_t *pRsSamp   = pFOC->pRsSamp;
    // if(pFOC->isEnable)
    // {
        pFOC->radian += 32;
    // }
    pRsSamp->ADCGetPreCurrent(pRsSamp);
    pFOC->ia = (param->adc_value[RESISTOR_U] - 2914);
    pFOC->ib = (param->adc_value[RESISTOR_V] - 2974);
    pFOC->ic = (param->adc_value[RESISTOR_W] - 2962);

    ClarkeTransform(pFOC);
    ParkTransform(pFOC);

    pFOC->id = pFOC->id * 3.3f / 4096 / 0.55f;
    pFOC->iq = pFOC->iq * 3.3f / 4096 / 0.55f;
    param->Id = pFOC->id;
    param->Iq = pFOC->iq;
    pFOC->idPID.present = pFOC->id;
    pFOC->iqPID.present = pFOC->iq;
    PID_Calc(&pFOC->idPID);
    PID_Calc(&pFOC->iqPID);

    ParkAntiTransform(pFOC);
    
    pSvpwm->SectorJudgment(pSvpwm);
    pSvpwm->VectorTime(pSvpwm);
    pSvpwm->Generate(pSvpwm);

    if((counter&PRINTF_SUBBUF_MASK) == (PRINTF_SUBBUF_MASK))
    {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(param - PRINTF_SUBBUF_MASK), sizeof(FocParam) * PRINTF_SUBBUF_NUM);
    }
    param = &param_buff[++counter & PRINTF_TOTAL_MASK];
}


