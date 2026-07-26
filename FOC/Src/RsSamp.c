#include "FocCommon.h"
#include "RsSamp.h"
#include <stdint.h>


static bool RsSamp_init(void *this)
{
    RsSamp_t *pRsSamp   = (RsSamp_t *)this;
    ADC_t *pADC         = pRsSamp->pADC;
    RsLPF_t *pLPF       = pRsSamp->pFilter;

    pADC->init(pADC);
    if(pLPF)
    {
       pLPF->init(pLPF); 
    }

    pRsSamp->toAmpere = 1.0f * 1000 * pADC->maxValue / pADC->maxValue / pADC->amplifier / pADC->mOhm ;

    return true;
}

static bool RsSample_getAmpere(void *this)
{
    bool status = true;
    RsSamp_t *pRsSamp   = (RsSamp_t *)this;
    ADC_t *pADC         = pRsSamp->pADC;
    RsLPF_t *pLPF       = pRsSamp->pFilter;

    status = pADC->getSample(pADC, pRsSamp->sampValue);
    if(pLPF)
    {
        pLPF->fliter(pLPF, pRsSamp->sampValue);
    }
    return status;
}

bool RsSamp_register(FOC_t *pFOC, const ADC_t *pADC, const RsLPF_t *pLPF)
{
    if(pFOC == NULL || pADC == NULL)
    {
        goto pointer_check_fail;
    }

    if(pADC->init == NULL || pADC->getSample == NULL || pADC->priv == NULL)
    {
        goto pointer_check_fail;
    }

    if(pLPF)
    {
        if(pLPF->init == NULL || pLPF->fliter == NULL)
        {
            goto pointer_check_fail;
        }
    }
    
    RsSamp_t *pRsSamp = (RsSamp_t*)malloc(sizeof(RsSamp_t));
    if(pRsSamp == NULL)
    {
        goto RsSamp_amlloc_fail;
    }
    memset(pRsSamp, 0x00, sizeof(RsSamp_t));

    pRsSamp->pADC = (ADC_t*)malloc(sizeof(ADC_t));
    if(pRsSamp->pADC == NULL)
    {
        goto adc_malloc_fail;
    }
    *pRsSamp->pADC = *pADC;

    if(pLPF)
    {
        pRsSamp->pFilter = (RsLPF_t*)malloc(sizeof(RsLPF_t));
        if(pRsSamp->pFilter == NULL)
        {
            goto Filter_malloc_fail;
        }
        *pRsSamp->pFilter = *pLPF;
    }

    pRsSamp->init       = RsSamp_init;
    pRsSamp->getAmpere  = RsSample_getAmpere;
    pFOC->pRsSamp = pRsSamp;

    return true; // RsSamp registered successfully

Filter_malloc_fail:
    free(pRsSamp->pADC);
adc_malloc_fail:
    free(pRsSamp);
RsSamp_amlloc_fail:
pointer_check_fail:

    return false;
}


bool RsSamp_unregister(FOC_t *pFOC)
{
    if(pFOC == NULL)
    {
        return false;
    }

    if(pFOC->pRsSamp)
    {
        RsSamp_t *pRsSamp = pFOC->pRsSamp;
        if(pRsSamp->pADC)
        {
            free(pRsSamp->pADC);
            pRsSamp->pADC = NULL;
        }

        
        if(pRsSamp->pFilter)
        {
            RsLPF_t *pFilter = pRsSamp->pFilter;
            if(pFilter->priv_data)
            {
                free(pFilter->priv_data);
                pFilter->priv_data = NULL;
            }
    
            free(pRsSamp->pFilter);
            pRsSamp->pFilter = NULL;
        }

        free(pFOC->pRsSamp);
        pFOC->pRsSamp = NULL;
    }

    return true; // RsSamp registered successfully
}