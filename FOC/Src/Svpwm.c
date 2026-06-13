#include "FocCommon.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

void SvpwmInit(void *this)
{
    Svpwm_t *pSvpwm = this;
	PWM_Opt *pPWM = &pSvpwm->PwmOpts;

	pPWM->SetPWM[PHASE_U](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_V](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_W](pPWM->priv, 0);
	pPWM->SetPWM[PHASE_INT](pPWM->priv, 4000);
	pPWM->enable(pPWM->priv, false);
}

static void SvpwmSectorJudgment(void *this)
{
    Svpwm_t *pSvpwm = this;
    uint32_t *U = (uint32_t *)&pSvpwm->u1;
    const uint8_t sector_judg[8] = {0, SECTOR_2, SECTOR_6, SECTOR_1, SECTOR_4, SECTOR_3, SECTOR_5, 0};

    float u_alpha_sqrt3_2, u_beta, u_beta_2;

    u_alpha_sqrt3_2 = pSvpwm->u_alpha * FLOAT_SQRT3_2;
    u_beta = pSvpwm->u_beta;
    u_beta_2 = u_beta/2;

    pSvpwm->u1 = u_beta;
    pSvpwm->u2 = u_alpha_sqrt3_2 - u_beta_2;
    pSvpwm->u3 = -u_alpha_sqrt3_2 - u_beta_2;
    pSvpwm->sector = sector_judg[(U[0] >> 31) | ((U[1] & 0x80000000) >> 30) | ((U[2] & 0x80000000) >> 29)];
}

/* sector process */
static void svpwm_sector_valid_process(Svpwm_t *pSvpwm, uint16_t *t_PWM)
{
}

static void svpwm_sector_1_process(Svpwm_t *pSvpwm, uint16_t *t_PWM)
{
    int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;
    pSvpwm->t4 = FLOAT_SQRT3_2 * ts * pSvpwm->u2 / udc;
    pSvpwm->t6 = FLOAT_SQRT3_2 * ts * pSvpwm->u1 / udc;
    pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t4 - pSvpwm->t6) / 2;

    t_PWM[PHASE_U] = pSvpwm->t4 + pSvpwm->t6 + pSvpwm->t7;
    t_PWM[PHASE_V] = pSvpwm->t6 + pSvpwm->t7;
    t_PWM[PHASE_W] = pSvpwm->t7;
}

static void svpwm_sector_2_process(Svpwm_t *pSvpwm, uint16_t *t_PWM)
{
    int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;
    pSvpwm->t2 = - FLOAT_SQRT3_2 * ts * pSvpwm->u2 / udc;
    pSvpwm->t6 = - FLOAT_SQRT3_2 * ts * pSvpwm->u3 / udc;
    pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t2 - pSvpwm->t6) / 2;

    t_PWM[PHASE_U] = pSvpwm->t6 + pSvpwm->t7;
    t_PWM[PHASE_V] = pSvpwm->t2 + pSvpwm->t6 + pSvpwm->t7;
    t_PWM[PHASE_W] = pSvpwm->t7;
}

static void svpwm_sector_3_process(Svpwm_t *pSvpwm, uint16_t *t_PWM)
{
    int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;
    pSvpwm->t2 = FLOAT_SQRT3_2 * ts * pSvpwm->u1 / udc;
    pSvpwm->t3 = FLOAT_SQRT3_2 * ts * pSvpwm->u3 / udc;
    pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t2 - pSvpwm->t3) / 2;

    t_PWM[PHASE_U] = pSvpwm->t7;
    t_PWM[PHASE_V] = pSvpwm->t2 + pSvpwm->t3 + pSvpwm->t7;
    t_PWM[PHASE_W] = pSvpwm->t3 + pSvpwm->t7;
}

static void svpwm_sector_4_process(Svpwm_t *pSvpwm, uint16_t *t_PWM)
{
    int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;
    pSvpwm->t1 = - FLOAT_SQRT3_2 * ts * pSvpwm->u1 / udc;
    pSvpwm->t3 = - FLOAT_SQRT3_2 * ts * pSvpwm->u2 / udc;
    pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t1 - pSvpwm->t3) / 2;

    t_PWM[PHASE_U] = pSvpwm->t7;
    t_PWM[PHASE_V] = pSvpwm->t3 + pSvpwm->t7;
    t_PWM[PHASE_W] = pSvpwm->t1 + pSvpwm->t3 + pSvpwm->t7;
}

static void svpwm_sector_5_process(Svpwm_t *pSvpwm, uint16_t *t_PWM)
{
    int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;
    pSvpwm->t1 = FLOAT_SQRT3_2 * ts * pSvpwm->u3 / udc;
    pSvpwm->t5 = FLOAT_SQRT3_2 * ts * pSvpwm->u2 / udc;
    pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t1 - pSvpwm->t5) / 2;

    t_PWM[PHASE_U] = pSvpwm->t5 + pSvpwm->t7;
    t_PWM[PHASE_V] = pSvpwm->t7;
    t_PWM[PHASE_W] = pSvpwm->t1 + pSvpwm->t5 + pSvpwm->t7;
}

static void svpwm_sector_6_process(Svpwm_t *pSvpwm, uint16_t *t_PWM)
{
    int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;
    pSvpwm->t4 = - FLOAT_SQRT3_2 * ts * pSvpwm->u3 / udc;
    pSvpwm->t5 = - FLOAT_SQRT3_2 * ts * pSvpwm->u1 / udc;
    pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t4 - pSvpwm->t5) / 2;

    t_PWM[PHASE_U] = pSvpwm->t4 + pSvpwm->t5 + pSvpwm->t7;
    t_PWM[PHASE_V] = pSvpwm->t7;
    t_PWM[PHASE_W] = pSvpwm->t5 + pSvpwm->t7;
}

void (*const sector_process[MAX_SECTOR])(Svpwm_t *, uint16_t *) = 
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
    uint16_t *t_PWM = param->PWM;
    sector_process[pSvpwm->sector](pSvpwm, t_PWM);
}

static void SvpwmGenerate(void *this)
{
    Svpwm_t *pSvpwm = this;
	uint16_t *t_PWM = param->PWM;
    pSvpwm->PwmOpts.SetPWM[PHASE_U](pSvpwm->PwmOpts.priv, t_PWM[PHASE_U]);
    pSvpwm->PwmOpts.SetPWM[PHASE_V](pSvpwm->PwmOpts.priv, t_PWM[PHASE_V]);
    pSvpwm->PwmOpts.SetPWM[PHASE_W](pSvpwm->PwmOpts.priv, t_PWM[PHASE_W]);
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

    if(pPWM_opts->enable == NULL)
    {
        return false;
    }

    if(pPWM_opts->SetPWM[PHASE_U] == NULL || pPWM_opts->SetPWM[PHASE_V] == NULL || \
        pPWM_opts->SetPWM[PHASE_W] == NULL || pPWM_opts->SetPWM[PHASE_INT] == NULL)
    {
        return false;
    }

    pFOC->pSvpwm = (Svpwm_t*)malloc(sizeof(Svpwm_t));
    if(pFOC->pSvpwm == NULL)
    {
        return false;
    }
    *pFOC->pSvpwm = defaultSvpwm;
    pFOC->pSvpwm->udc = 5;
    pFOC->pSvpwm->ts = 4200;

    pFOC->pSvpwm->PwmOpts = *pPWM_opts;
    pFOC->pSvpwm->PwmOpts.priv = priv;

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


