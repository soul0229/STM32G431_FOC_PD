#include "FocCommon.h"
#include "arm_math.h"

void sin_cos_4096(uint16_t angle, float32_t *sine, float32_t *cosine);

static void PID_Calc(PID_t *pid, float target, float present)
{
    float temp;
    if (pid == NULL) {
        return;
    }

    float bias = target - present;
    pid->out = temp = pid->kp * bias + pid->iSum * pid->kd;

    if (pid->out > pid->outMax) {
        pid->out = pid->outMax;
    } else if (pid->out < -pid->outMax) {
        pid->out = -pid->outMax;
    }

    pid->iSum = pid->kb *(pid->out - temp) + pid->ki * bias;
}

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

    float modulus = pFOC->iqPID.out * pFOC->iqPID.out + pFOC->idPID.out * pFOC->idPID.out;
    if (modulus > pSvpwm->modulus_max) {
        float scale = pSvpwm->modulus_max / modulus;
        pFOC->idPID.out *= scale;
        pFOC->iqPID.out *= scale;
    }

    pSvpwm->u_alpha = pFOC->idPID.out * cosine - pFOC->iqPID.out * sine;
    pSvpwm->u_beta  = pFOC->idPID.out * sine + pFOC->iqPID.out * cosine;
}

static void MotorEnable(void *this, bool enable)
{
    PWM_t *pPWM =  ((FOC_t*)this)->pSvpwm->pPWM;
    pPWM->enable(pPWM->priv, enable);
}

static void call_back(void *this)
{

}

bool FOC_init(FOC_t *pFOC, cb_t *cb)
{
    if(pFOC == NULL)
    {
        return false;
    }

    if(!pFOC->pSvpwm || !pFOC->pRsSamp)
    {
        return false;
    }

    memset(&pFOC->idPID, 0x00, sizeof(pFOC->idPID));
    memset(&pFOC->iqPID, 0x00, sizeof(pFOC->iqPID));
    // pFOC->GetPreCurrent = ADCGetPreCurrent;
    pFOC->rNum = 3;
    pFOC->idPID.kp = 5.000f;
    pFOC->idPID.ki = 0.0001f;
    pFOC->idPID.kd = 0.0001f;
    pFOC->idPID.outMax = 3.0f;

    pFOC->iqPID.kp = 20.0f;
    pFOC->iqPID.ki = 1.0f;
    pFOC->iqPID.kd = 1.0f;
    pFOC->iqPID.outMax = 3.0f;

    pFOC->EnableMotor = MotorEnable;
    pFOC->pSvpwm->Init(pFOC->pSvpwm);
    pFOC->pRsSamp->init(pFOC->pRsSamp);
    pFOC->tar_iq = 0.75f;
    if(pFOC->pSensor != NULL)
    {
        pFOC->pSensor->init(pFOC->pSensor);
    }

    pFOC->callback = call_back;
    if(cb)
    {
        pFOC->callback = cb;
    }

    return true;
}

void Foc_Process(FOC_t *pFOC)
{
    if(pFOC == NULL)
    {
        return;
    }
    
    Svpwm_t *pSvpwm     = pFOC->pSvpwm;
    RsSamp_t *pRsSamp   = pFOC->pRsSamp;
    Sensor_t *pSensor   = pFOC->pSensor;

    if(!pRsSamp->getAmpere(pRsSamp))
    {
        return;
    }

    pSensor->getAngle(pSensor);

    pFOC->ia = pRsSamp->sampValue[RESISTOR_U];
    pFOC->ib = pRsSamp->sampValue[RESISTOR_V];
    pFOC->ic = pRsSamp->sampValue[RESISTOR_W];

    ClarkeTransform(pFOC);
    ParkTransform(pFOC);

    pFOC->id = pFOC->id * pRsSamp->toAmpere;
    pFOC->iq = pFOC->iq * pRsSamp->toAmpere;

    PID_Calc(&pFOC->idPID, pFOC->tar_id, pFOC->id);
    PID_Calc(&pFOC->iqPID, pFOC->tar_iq, pFOC->iq);

    ParkAntiTransform(pFOC);
    
    pSvpwm->SectorJudgment(pSvpwm);
    pSvpwm->VectorTime(pSvpwm);
    pSvpwm->Generate(pSvpwm);

    pFOC->callback(pFOC);
}


