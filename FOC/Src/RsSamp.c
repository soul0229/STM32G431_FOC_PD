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


static void LPF_3PhaseUpdate(RsSampLPF_t* filter, int16_t input[RESISTOR_MAX], int16_t output[RESISTOR_MAX])
{
    for (int i = 0; i < RESISTOR_MAX; i++) {
        output[i] = LPF_SingleUpdate(filter, input[i], i);
    }
}

static void ADCGetPreCurrent(void *this)
{
    FOC_t *pFOC = this;
    RsSamp_t *pRsSamp = pFOC->pRsSamp;
    // RsSamp_t *pRsSamp = pFOC->pRsSamp;
    // pFOC->ia = pRsSamp->getRsSample[0](pRsSamp->priv);
    // pFOC->ib = pRsSamp->getRsSample[1](pRsSamp->priv);
    // pFOC->ic = pRsSamp->getRsSample[2](pRsSamp->priv);
    param->adc_value[0] = pRsSamp->getRsSample[0](pRsSamp->priv);
    param->adc_value[1] = pRsSamp->getRsSample[1](pRsSamp->priv);
    param->adc_value[2] = pRsSamp->getRsSample[2](pRsSamp->priv);
    // if(pRsSamp->averageCnt < 5000)
    // {
    //     pRsSamp->offset[0] += pFOC->ia;
    //     pRsSamp->offset[1] += pFOC->ib;
    //     pRsSamp->offset[2] += pFOC->ic;
    //     pRsSamp->averageCnt++;
    //     if(pRsSamp->averageCnt == 5000)
    //     {
    //         pRsSamp->offset[0] /= 5000;
    //         pRsSamp->offset[1] /= 5000;
    //         pRsSamp->offset[2] /= 5000;
            // pFOC->isEnable = true;
            pFOC->iqPID.out = 4;
    //     }
    //     return;
    // }
    // pFOC->ia = pFOC->ia - adc_opts->offset[0];
    // pFOC->ib = pFOC->ib - adc_opts->offset[1];
    // pFOC->ic = pFOC->ic - adc_opts->offset[2];
    param->adc_value[0] = LPF_SingleUpdate(&filter_ADC, param->adc_value[RESISTOR_U], RESISTOR_U);
    param->adc_value[1] = LPF_SingleUpdate(&filter_ADC, param->adc_value[RESISTOR_V], RESISTOR_V);
    param->adc_value[2] = LPF_SingleUpdate(&filter_ADC, param->adc_value[RESISTOR_W], RESISTOR_W);
 
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