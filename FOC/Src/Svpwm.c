#include "FocCommon.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static const uint8_t sector_judg[MAX_SECTOR] = {0, SECTOR_5, SECTOR_3, SECTOR_4, SECTOR_1, SECTOR_6, SECTOR_2, 0};

static void SvpwmInit(void *this)
{
    Svpwm_t *pSvpwm = this;
	PWM_Opt *pPWM = &pSvpwm->PwmOpts;

    pSvpwm->ts = pPWM->cycle * 0.95f;
    pSvpwm->tk = FLOAT_SQRT3 * pSvpwm->ts / pPWM->udc;
    memset(pSvpwm->PWM, 0x00, sizeof(pSvpwm->PWM));
    pSvpwm->PWM[PHASE_INT] = (pSvpwm->ts + pPWM->cycle) / 2;
	pPWM->SetPWM(pPWM->priv, pSvpwm->PWM);
	pPWM->enable(pPWM->priv, false);
}

static void SvpwmSectorJudgment(void *this)
{
    Svpwm_t *pSvpwm = this;
    uint32_t *U = (uint32_t *)&pSvpwm->u1;
    float u_alpha_sqrt3_2, u_beta, u_beta_2;

    u_alpha_sqrt3_2 = pSvpwm->u_alpha * FLOAT_SQRT3_2;
    u_beta          = pSvpwm->u_beta;
    u_beta_2        = u_beta / 2;

    pSvpwm->u1 = u_beta;
    pSvpwm->u2 = u_alpha_sqrt3_2 - u_beta_2;
    pSvpwm->u3 = -u_alpha_sqrt3_2 - u_beta_2;

    pSvpwm->sector = sector_judg[(U[0] >> 31) | ((U[1] & 0x80000000) >> 30) | ((U[2] & 0x80000000) >> 29)];
}

/* sector process */
static void svpwm_sector_valid_process(Svpwm_t *pSvpwm, uint16_t *pPWM)
{
    pPWM[PHASE_U] = 0;
    pPWM[PHASE_V] = 0;
    pPWM[PHASE_W] = 0;
}

static void svpwm_sector_1_process(Svpwm_t *pSvpwm, uint16_t *pPWM)
{
    pSvpwm->t4 = pSvpwm->u2 * pSvpwm->tk;
    pSvpwm->t6 = pSvpwm->u1 * pSvpwm->tk;
    pSvpwm->t0 = pSvpwm->t7 = (pSvpwm->ts - pSvpwm->t4 - pSvpwm->t6) / 2;

    pPWM[PHASE_U] = pSvpwm->ts - (pSvpwm->t4 + pSvpwm->t6 + pSvpwm->t7);
    pPWM[PHASE_V] = pSvpwm->ts - (pSvpwm->t6 + pSvpwm->t7);
    pPWM[PHASE_W] = pSvpwm->ts - (pSvpwm->t7);
}

static void svpwm_sector_2_process(Svpwm_t *pSvpwm, uint16_t *pPWM)
{
    pSvpwm->t2 = - pSvpwm->u2 * pSvpwm->tk;
    pSvpwm->t6 = - pSvpwm->u3 * pSvpwm->tk;
    pSvpwm->t0 = pSvpwm->t7 = (pSvpwm->ts - pSvpwm->t2 - pSvpwm->t6) / 2;

    pPWM[PHASE_U] = pSvpwm->ts - (pSvpwm->t6 + pSvpwm->t7);
    pPWM[PHASE_V] = pSvpwm->ts - (pSvpwm->t2 + pSvpwm->t6 + pSvpwm->t7);
    pPWM[PHASE_W] = pSvpwm->ts - (pSvpwm->t7);
}

static void svpwm_sector_3_process(Svpwm_t *pSvpwm, uint16_t *pPWM)
{
    pSvpwm->t2 = pSvpwm->u1 * pSvpwm->tk;
    pSvpwm->t3 = pSvpwm->u3 * pSvpwm->tk;
    pSvpwm->t0 = pSvpwm->t7 = (pSvpwm->ts - pSvpwm->t2 - pSvpwm->t3) / 2;

    pPWM[PHASE_U] = pSvpwm->ts - (pSvpwm->t7);
    pPWM[PHASE_V] = pSvpwm->ts - (pSvpwm->t2 + pSvpwm->t3 + pSvpwm->t7);
    pPWM[PHASE_W] = pSvpwm->ts - (pSvpwm->t3 + pSvpwm->t7);
}

static void svpwm_sector_4_process(Svpwm_t *pSvpwm, uint16_t *pPWM)
{
    pSvpwm->t1 = - pSvpwm->u1 * pSvpwm->tk;
    pSvpwm->t3 = - pSvpwm->u2 * pSvpwm->tk;
    pSvpwm->t0 = pSvpwm->t7 = (pSvpwm->ts - pSvpwm->t1 - pSvpwm->t3) / 2;

    pPWM[PHASE_U] = pSvpwm->ts - (pSvpwm->t7);
    pPWM[PHASE_V] = pSvpwm->ts - (pSvpwm->t3 + pSvpwm->t7);
    pPWM[PHASE_W] = pSvpwm->ts - (pSvpwm->t1 + pSvpwm->t3 + pSvpwm->t7);
}

static void svpwm_sector_5_process(Svpwm_t *pSvpwm, uint16_t *pPWM)
{
    pSvpwm->t1 = pSvpwm->u3 * pSvpwm->tk;
    pSvpwm->t5 = pSvpwm->u2 * pSvpwm->tk;
    pSvpwm->t0 = pSvpwm->t7 = (pSvpwm->ts - pSvpwm->t1 - pSvpwm->t5) / 2;

    pPWM[PHASE_U] = pSvpwm->ts - (pSvpwm->t5 + pSvpwm->t7);
    pPWM[PHASE_V] = pSvpwm->ts - (pSvpwm->t7);
    pPWM[PHASE_W] = pSvpwm->ts - (pSvpwm->t1 + pSvpwm->t5 + pSvpwm->t7);
}

static void svpwm_sector_6_process(Svpwm_t *pSvpwm, uint16_t *pPWM)
{
    pSvpwm->t4 = - pSvpwm->u3 * pSvpwm->tk;
    pSvpwm->t5 = - pSvpwm->u1 * pSvpwm->tk;
    pSvpwm->t0 = pSvpwm->t7 = (pSvpwm->ts - pSvpwm->t4 - pSvpwm->t5) / 2;

    pPWM[PHASE_U] = pSvpwm->ts - (pSvpwm->t4 + pSvpwm->t5 + pSvpwm->t7);
    pPWM[PHASE_V] = pSvpwm->ts - (pSvpwm->t7);
    pPWM[PHASE_W] = pSvpwm->ts - (pSvpwm->t5 + pSvpwm->t7);
}

static void (*const sector_process[MAX_SECTOR])(Svpwm_t *, uint16_t *) = 
{
    [SECTOR_INVALID_0]  = svpwm_sector_valid_process,
    [SECTOR_1]          = svpwm_sector_1_process,
    [SECTOR_2]          = svpwm_sector_2_process,
    [SECTOR_3]          = svpwm_sector_3_process,
    [SECTOR_4]          = svpwm_sector_4_process,
    [SECTOR_5]          = svpwm_sector_5_process,
    [SECTOR_6]          = svpwm_sector_6_process,
    [SECTOR_INVALID_7]  = svpwm_sector_valid_process,
};

static void GetVectorDuration(void *this)
{
    Svpwm_t *pSvpwm = this;
    uint16_t *pPWM = pSvpwm->PWM;
    sector_process[pSvpwm->sector](pSvpwm, pPWM);
}

static void SvpwmGenerate(void *this)
{
    Svpwm_t *pSvpwm = this;
    uint16_t *pPWM = pSvpwm->PWM;
    pSvpwm->PwmOpts.SetPWM(pSvpwm->PwmOpts.priv, pPWM);
}

static const Svpwm_t defaultSvpwm = 
{
    .Init           = SvpwmInit,
    .SectorJudgment = SvpwmSectorJudgment,
    .VectorTime     = GetVectorDuration,
    .Generate       = SvpwmGenerate,
};

bool Svpwm_register(FOC_t *pFOC, const PWM_Opt *pPWM_opts, void *priv)
{
    if(pFOC == NULL || pPWM_opts == NULL || priv == NULL)
    {
        return false;
    }

    if(pPWM_opts->enable == NULL || pPWM_opts->SetPWM == NULL)
    {
        return false;
    }

    Svpwm_t *pSvpwm = (Svpwm_t*)malloc(sizeof(Svpwm_t));
    if(pSvpwm == NULL)
    {
        return false;
    }
    
    *pSvpwm = defaultSvpwm;
    pSvpwm->PwmOpts = *pPWM_opts;
    pSvpwm->PwmOpts.priv = priv;

    pFOC->pSvpwm = pSvpwm;

    return true;
} 

bool Svpwm_unregister(FOC_t *pFOC)
{
    if(pFOC == NULL)
    {
        return false;
    }

    if(pFOC->pSvpwm)
    {
        free(pFOC->pSvpwm);
    }
    return true;
}


