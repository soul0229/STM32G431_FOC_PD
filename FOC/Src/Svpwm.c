#include "Svpwm.h"
#include <stdint.h>
#include <string.h>
#include "tim.h"
#include "usart.h"
#include <stdbool.h>
#include <stdlib.h>

#include "usb_device.h"

const FocParam default_param = {.header  = {0xaa},.tail = {0xcc}};
FocParam param[PRINTF_BUF_NUM][PRINTF_SUBBUF_NUM];
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

void SvpwmSectorJudgment(void *this)
{
    pSvpwm_Info pSvpwm = this;
    const uint8_t sector_judg[8] = {0, 2, 6, 1, 4, 3, 5, 0};
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


void GetVectorDuration(void *this)
{
    pSvpwm_Info pSvpwm = this;
	int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;

    switch (pSvpwm->sector) {
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

void SvpwmGenerate(void *this)
{
    pSvpwm_Info pSvpwm = this;
    PWM_Opt *p_opts = pSvpwm->s_opts->pwm_opts;
    void *private = p_opts->private;

	uint16_t t[PHASE_MAX] = {0};
    switch (pSvpwm->sector) {
        case SECTOR_1:
            t[PHASE_U] = pSvpwm->t4 + pSvpwm->t6 + pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t6 + pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t7;
            break;
        case SECTOR_2:
            t[PHASE_U] = pSvpwm->t6 + pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t2 + pSvpwm->t6 + pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t7;
            break;
        case SECTOR_3:
            t[PHASE_U] = pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t2 + pSvpwm->t3 + pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t3 + pSvpwm->t7;
            break;
        case SECTOR_4:
            t[PHASE_U] = pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t3 + pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t1 + pSvpwm->t3 + pSvpwm->t7;
            break;
        case SECTOR_5:
            t[PHASE_U] = pSvpwm->t5 + pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t1 + pSvpwm->t5 + pSvpwm->t7;
            break;
        case SECTOR_6:
            t[PHASE_U] = pSvpwm->t4 + pSvpwm->t5 + pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t5 + pSvpwm->t7;
            break;
    }
    // uint8_t len = sprintf(uart, "U:%d V:%d W:%d\r\n", t[PHASE_U], t[PHASE_V], t[PHASE_W]);
    // memcpy(param.PWM, t, sizeof(t));
    t[PHASE_INT] = 4000;
    // CDC_Transmit_FS((uint8_t*)&param, sizeof(FocParam));
    p_opts->SetPWM[PHASE_U](private, t[PHASE_U]);
    p_opts->SetPWM[PHASE_V](private, t[PHASE_V]);
    p_opts->SetPWM[PHASE_W](private, t[PHASE_W]);
    p_opts->SetPWM[PHASE_INT](private, t[PHASE_INT]);
}

Sector GetSVPWMSector(void *this)
{
    pSvpwm_Info pSvpwm = this;
    return pSvpwm->sector;
}

void SvpwmControl(void *this)
{
    pSvpwm_Info pSvpwm = this;
    Svpwm_Opt *s_opts = pSvpwm->s_opts;
    s_opts->SectorJudgment(this);
    s_opts->VectorTime(this);
    s_opts->Generate(this);
}

const Svpwm_Opt svpwm_opts = 
{
    .Generate = SvpwmGenerate,
    .GetSector = GetSVPWMSector,
    .SectorJudgment = SvpwmSectorJudgment,
    .VectorTime = GetVectorDuration,
    .SvpwmControl = SvpwmControl
};


bool check_pwm_opts(PWM_Opt *opts)
{
    if(!(opts->enable && opts->init && opts->private))
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

pSvpwm_Info Svpwm_init(PWM_Opt *opts)
{
    if(!check_pwm_opts(opts))
    {
        goto check_pwm_opts_error;
    }

    for(uint8_t dim1 = 0; dim1 < PRINTF_BUF_NUM; dim1++)
    {
        for(uint8_t dim2 = 0; dim2 < PRINTF_BUF_NUM; dim2++)
        {
            param[dim1][dim2] = default_param;
        }
    }

    Svpwm_Opt *s_opts = malloc(sizeof(Svpwm_Opt));
    if(!s_opts)
    {
        goto svpwm_malloc_error;
    }
    *s_opts = svpwm_opts;
    s_opts->pwm_opts = opts;

    pSvpwm_Info svpwm = malloc(sizeof(Svpwm_Info));
    if(!svpwm)
    {
        goto pSvpwm_info_malloc_error;
    }
    memset(svpwm, 0x00, sizeof(Svpwm_Info));
    svpwm->udc = 5;
    svpwm->ts = 4200;

    svpwm->s_opts = s_opts;
    return svpwm;

pSvpwm_info_malloc_error:
    free(s_opts);
svpwm_malloc_error:
check_pwm_opts_error:

    return NULL;
}

void Svpwm_deinit(pSvpwm_Info svpwm)
{
    if(svpwm == NULL)
    {
        return;
    }

    if(svpwm->s_opts == NULL)
    {
        goto free_svpwm;
    }

    Svpwm_Opt *s_opts = svpwm->s_opts;
    if(s_opts->pwm_opts == NULL)
    {
        goto free_s_opts;
    }
    PWM_Opt *p_opts = s_opts->pwm_opts;

    free(p_opts);
free_s_opts:
    free(s_opts);
free_svpwm:
    free(svpwm);
}


