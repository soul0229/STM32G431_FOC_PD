#include "FocCommon.h"
#include <stdbool.h>

static void SvpwmSectorJudgment(void *this)
{
    Svpwm_t *pSvpwm = this;
    const uint8_t sector_judg[8] = {0, SECTOR_2, SECTOR_6, SECTOR_1, SECTOR_4, SECTOR_3, SECTOR_5, 0};
    uint8_t a = 0;
    uint8_t b = 0;
    uint8_t c = 0;
    float u_alpha_sqrt3_2, u_beta, u_beta_2;

    u_alpha_sqrt3_2 = pSvpwm->u_alpha * FLOAT_SQRT3_2;
    u_beta = pSvpwm->u_beta;
    u_beta_2 = u_beta/2;

    pSvpwm->u1 = u_beta;
    pSvpwm->u2 = u_alpha_sqrt3_2 - u_beta_2;
    pSvpwm->u3 = -u_alpha_sqrt3_2 - u_beta_2;

    if(pSvpwm->u1 > 0)
    {
        a = 1;
    } 
    if(pSvpwm->u2 > 0)
    {
        b = 1;
    } 
    if(pSvpwm->u3 > 0)
    {
        c = 1;
    }
    pSvpwm->sector = sector_judg[c<<2|b<<1|a];
}


static void GetVectorDuration(void *this)
{
    Svpwm_t *pSvpwm = this;
	int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;

    switch (pSvpwm->sector) 
    {
        case SECTOR_1:
            pSvpwm->t4 = FLOAT_SQRT3_2 * ts * pSvpwm->u2 / udc;
            pSvpwm->t6 = FLOAT_SQRT3_2 * ts * pSvpwm->u1 / udc;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t4 - pSvpwm->t6) / 2;
            break;
        case SECTOR_2:
            pSvpwm->t2 = - FLOAT_SQRT3_2 * ts * pSvpwm->u2 / udc;
            pSvpwm->t6 = - FLOAT_SQRT3_2 * ts * pSvpwm->u3 / udc;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t2 - pSvpwm->t6) / 2;
            break;
        case SECTOR_3:
            pSvpwm->t2 = FLOAT_SQRT3_2 * ts * pSvpwm->u1 / udc;
            pSvpwm->t3 = FLOAT_SQRT3_2 * ts * pSvpwm->u3 / udc;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t2 - pSvpwm->t3) / 2;
            break;
        case SECTOR_4:
            pSvpwm->t1 = - FLOAT_SQRT3_2 * ts * pSvpwm->u1 / udc;
            pSvpwm->t3 = - FLOAT_SQRT3_2 * ts * pSvpwm->u2 / udc;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t1 - pSvpwm->t3) / 2;
            break;
        case SECTOR_5:
            pSvpwm->t1 = FLOAT_SQRT3_2 * ts * pSvpwm->u3 / udc;
            pSvpwm->t5 = FLOAT_SQRT3_2 * ts * pSvpwm->u2 / udc;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t1 - pSvpwm->t5) / 2;
            break;
        case SECTOR_6:
            pSvpwm->t4 = - FLOAT_SQRT3_2 * ts * pSvpwm->u3 / udc;
            pSvpwm->t5 = - FLOAT_SQRT3_2 * ts * pSvpwm->u1 / udc;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t4 - pSvpwm->t5) / 2;
            break;
        default:
            break;
    }
}
extern TIM_HandleTypeDef htim8;
static void SvpwmGenerate(void *this)
{
    Svpwm_t *pSvpwm = this;
	uint16_t *t_PWM = param->PWM;
    switch (pSvpwm->sector) 
    {
        case SECTOR_1:
            t_PWM[PHASE_U] = pSvpwm->t4 + pSvpwm->t6 + pSvpwm->t7;
            t_PWM[PHASE_V] = pSvpwm->t6 + pSvpwm->t7;
            t_PWM[PHASE_W] = pSvpwm->t7;
            break;
        case SECTOR_2:
            t_PWM[PHASE_U] = pSvpwm->t6 + pSvpwm->t7;
            t_PWM[PHASE_V] = pSvpwm->t2 + pSvpwm->t6 + pSvpwm->t7;
            t_PWM[PHASE_W] = pSvpwm->t7;
            break;
        case SECTOR_3:
            t_PWM[PHASE_U] = pSvpwm->t7;
            t_PWM[PHASE_V] = pSvpwm->t2 + pSvpwm->t3 + pSvpwm->t7;
            t_PWM[PHASE_W] = pSvpwm->t3 + pSvpwm->t7;
            break;
        case SECTOR_4:
            t_PWM[PHASE_U] = pSvpwm->t7;
            t_PWM[PHASE_V] = pSvpwm->t3 + pSvpwm->t7;
            t_PWM[PHASE_W] = pSvpwm->t1 + pSvpwm->t3 + pSvpwm->t7;
            break;
        case SECTOR_5:
            t_PWM[PHASE_U] = pSvpwm->t5 + pSvpwm->t7;
            t_PWM[PHASE_V] = pSvpwm->t7;
            t_PWM[PHASE_W] = pSvpwm->t1 + pSvpwm->t5 + pSvpwm->t7;
            break;
        case SECTOR_6:
            t_PWM[PHASE_U] = pSvpwm->t4 + pSvpwm->t5 + pSvpwm->t7;
            t_PWM[PHASE_V] = pSvpwm->t7;
            t_PWM[PHASE_W] = pSvpwm->t5 + pSvpwm->t7;
            break;
        default: return;
    }

    pSvpwm->pwm_opts.SetPWM[PHASE_U](pSvpwm->pwm_opts.priv, t_PWM[PHASE_U]);
    pSvpwm->pwm_opts.SetPWM[PHASE_V](pSvpwm->pwm_opts.priv, t_PWM[PHASE_V]);
    pSvpwm->pwm_opts.SetPWM[PHASE_W](pSvpwm->pwm_opts.priv, t_PWM[PHASE_W]);
}

static const Svpwm_t defaultSvpwm = 
{
    .SectorJudgment = SvpwmSectorJudgment,
    .VectorTime     = GetVectorDuration,
    .Generate       = SvpwmGenerate,
};


static bool check_pwm_opts(PWM_Opt *opts, void *priv)
{
    if(!(opts->enable && opts->init && priv))
    {
        return false;
    }

    if(!(opts->SetPWM[PHASE_U] && opts->SetPWM[PHASE_V] && \
        opts->SetPWM[PHASE_W] && opts->SetPWM[PHASE_INT]))
    {
        return false;
    }

    return true;
}

bool Svpwm_register(FOC_t *pFOC, PWM_Opt *pPWM_opts, void *priv)
{
    if(pFOC == NULL || pPWM_opts == NULL || priv == NULL)
    {
        return false;
    }

    if(pPWM_opts->enable == NULL || pPWM_opts->init == NULL)
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

    pFOC->pSvpwm->pwm_opts = *pPWM_opts;
    pFOC->pSvpwm->pwm_opts.priv = priv;

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


