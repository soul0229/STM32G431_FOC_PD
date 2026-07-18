#include "FocCommon.h"
#include "RsSamp.h"

static RsSampLPF_t filter_ADC;

static void LPF_Init(RsSampLPF_t* filter, float cutOffFreq, float sampFreq) {
    filter->cutOffFreq = cutOffFreq;
    filter->sampFreq = sampFreq;

    float RC = 1.0f / (2.0f * 3.1415926535f * cutOffFreq);
    float Ts = 1.0f / sampFreq;
    filter->alpha = Ts / (Ts + RC);
    
    for (int i = 0; i < RESISTOR_MAX; i++) {
        filter->prvValue[i] = 0;
    }
}


static int16_t LPF_SingleUpdate(RsSampLPF_t* filter, int16_t input, int16_t phase)
{
    int16_t output = filter->alpha * input + (1.0f - filter->alpha) * filter->prvValue[phase];
    filter->prvValue[phase] = output;
    return output;
}


static void ADCGetPreCurrent(void *this)
{
    RsSamp_t *pRsSamp = this;
    param->adc_value[RESISTOR_U] = LPF_SingleUpdate(&filter_ADC, pRsSamp->getRsSample[RESISTOR_U](pRsSamp->priv), RESISTOR_U);
    param->adc_value[RESISTOR_V] = LPF_SingleUpdate(&filter_ADC, pRsSamp->getRsSample[RESISTOR_V](pRsSamp->priv), RESISTOR_V);
    param->adc_value[RESISTOR_W] = LPF_SingleUpdate(&filter_ADC, pRsSamp->getRsSample[RESISTOR_W](pRsSamp->priv), RESISTOR_W);
}

bool RsSamp_register(FOC_t *pFOC, const RsSamp_t *pRsSamp, void *priv)
{
    if(pFOC == NULL || pRsSamp == NULL || priv == NULL)
    {
        return false;
    }

    if(!pRsSamp->getRsSample[RESISTOR_U] || !pRsSamp->getRsSample[RESISTOR_V] || !pRsSamp->getRsSample[RESISTOR_W])
    {
        return false;
    }

    if(pRsSamp->Init == NULL)
    {
        return false;
    }

    if(!pRsSamp->sampRefVolt || !pRsSamp->amplifier || !pRsSamp->mOhm)
    {
        return false;
    }
    
    pFOC->pRsSamp = (RsSamp_t*)malloc(sizeof(RsSamp_t));
    if(pFOC->pRsSamp == NULL)
    {
        return false;
    }

    *(pFOC->pRsSamp) = *pRsSamp;
    pFOC->pRsSamp->priv = priv;
    pFOC->pRsSamp->ADCGetPreCurrent = ADCGetPreCurrent;
    LPF_Init(&filter_ADC, 100, 20000);
    return true; // RsSamp registered successfully
}


bool RsSamp_unregister(FOC_t *pFOC)
{
    if(pFOC == NULL)
    {
        return false;
    }

    if(pFOC->pRsSamp)
    {
        free(pFOC->pRsSamp);
    }

    return true; // RsSamp registered successfully
}