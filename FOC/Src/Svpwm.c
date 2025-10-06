#include "Svpwm.h"
#include <stdint.h>
#include <string.h>
#include "tim.h"
#include "usart.h"
#include <stdbool.h>
#include <stdlib.h>

#include "usb_device.h"
FocParam param = {
    .header  = {0xaa, 0xbb},
    .tail = {0xcc, 0xdd},
};
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

static const uint8_t sector_judg[2][2][2] = 
{
    {
        { 0, 2},
        { 6, 1},
    },
    {
        { 4, 3},
        { 5, 0},
    },
};

void SvpwmSectorJudgment(void *this)
{
    pSvpwm_Info pSvpwm = this;
    // uint8_t index[3];
    // memset(index, 0x00, sizeof(index));
    float u_alpha_sqrt3_2, u_beta, u_beta_2;

    u_alpha_sqrt3_2 = pSvpwm->u_alpha * FLOAT_SQRT3_2;
    u_beta = pSvpwm->u_beta;
    u_beta_2 = u_beta/2;

    pSvpwm->u1 = u_beta;
    pSvpwm->u2 = u_alpha_sqrt3_2 - u_beta_2;
    pSvpwm->u3 = -u_alpha_sqrt3_2 - u_beta_2;


    // if (pSvpwm->u1 > 0) {
    //     index[0] = 1;
    // }
    // if (pSvpwm->u2 > 0) {
    //     index[1] = 1;
    // }
    // if (pSvpwm->u3 > 0) {
    //     index[2] = 1;
    // }
    // pSvpwm->sector = sector_judg[index[2]][index[1]][index[0]];

    uint8_t a;
    uint8_t b;
    uint8_t c;
    uint8_t sector;
    if (pSvpwm->u1 > 0) {
        a = 1;
    } else {
        a = 0;
    }
    if (pSvpwm->u2 > 0) {
        b = 1;
    } else {
        b = 0;
    }
    if (pSvpwm->u3 > 0) {
        c = 1;
    } else {
        c = 0;
    }

    sector = 4*c + 2*b + a;
    switch (sector) {
        case 3:
            pSvpwm->sector = 1;
            break;
        case 1:
            pSvpwm->sector = 2;
            break;
        case 5:
            pSvpwm->sector = 3;
            break;
        case 4:
            pSvpwm->sector = 4;
            break;
        case 6:
            pSvpwm->sector = 5;
            break;
        case 2:
            pSvpwm->sector = 6;
            break;
    }
}


void GetVectorDuration(void *this)
{
    pSvpwm_Info pSvpwm = this;
	int32_t ts = pSvpwm->ts;
	float udc = pSvpwm->udc;
	
    switch (pSvpwm->sector) {
        case 1:
            pSvpwm->t4 = FLOAT_SQRT3 * ts / udc * pSvpwm->u2;
            pSvpwm->t6 = FLOAT_SQRT3 * ts / udc * pSvpwm->u1;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t4 - pSvpwm->t6) / 2;
            break;
        case 2:
            pSvpwm->t2 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u2;
            pSvpwm->t6 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u3;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t2 - pSvpwm->t6) / 2;
            break;
        case 3:
            pSvpwm->t2 = FLOAT_SQRT3 * ts / udc * pSvpwm->u1;
            pSvpwm->t3 = FLOAT_SQRT3 * ts / udc * pSvpwm->u3;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t2 - pSvpwm->t3) / 2;
            break;
        case 4:
            pSvpwm->t1 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u1;
            pSvpwm->t3 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u2;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t1 - pSvpwm->t3) / 2;
            break;
        case 5:
            pSvpwm->t1 = FLOAT_SQRT3 * ts / udc * pSvpwm->u3;
            pSvpwm->t5 = FLOAT_SQRT3 * ts / udc * pSvpwm->u2;
            pSvpwm->t0 = pSvpwm->t7 = (ts - pSvpwm->t1 - pSvpwm->t5) / 2;
            break;
        case 6:
            pSvpwm->t4 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u3;
            pSvpwm->t5 = - FLOAT_SQRT3 * ts / udc * pSvpwm->u1;
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

	uint16_t t[Total] = {0};
    switch (pSvpwm->sector) {
        case 1:
            t[PHASE_U] = pSvpwm->t4 + pSvpwm->t6 + pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t6 + pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t7;
            break;
        case 2:
            t[PHASE_U] = pSvpwm->t6 + pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t2 + pSvpwm->t6 + pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t7;
            break;
        case 3:
            t[PHASE_U] = pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t2 + pSvpwm->t3 + pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t3 + pSvpwm->t7;
            break;
        case 4:
            t[PHASE_U] = pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t3 + pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t1 + pSvpwm->t3 + pSvpwm->t7;
            break;
        case 5:
            t[PHASE_U] = pSvpwm->t5 + pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t1 + pSvpwm->t5 + pSvpwm->t7;
            break;
        case 6:
            t[PHASE_U] = pSvpwm->t4 + pSvpwm->t5 + pSvpwm->t7;
            t[PHASE_V] = pSvpwm->t7;
            t[PHASE_W] = pSvpwm->t5 + pSvpwm->t7;
            break;
    }
    // uint8_t len = sprintf(uart, "U:%d V:%d W:%d\r\n", t[PHASE_U], t[PHASE_V], t[PHASE_W]);
    memcpy(param.PWM, t, sizeof(t));
    t[INT] = 4000;
    // CDC_Transmit_FS((uint8_t*)&param, sizeof(FocParam));

    p_opts->SetPWM[PHASE_U](private, t[PHASE_U]);
    p_opts->SetPWM[PHASE_V](private, t[PHASE_V]);
    p_opts->SetPWM[PHASE_W](private, t[PHASE_W]);
    p_opts->SetPWM[INT](private, t[INT]);
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
    if(opts->enable == NULL)
    {
        return false;
    }

    if(opts->init == NULL)
    {
        return false;
    }

    if(opts->private == NULL)
    {
        return false;
    }

    for(uint8_t function = PHASE_U; function < Total; function++)
    {
        if(opts->SetPWM[function] == NULL)
        {
            return false;
        }
    }
    return true;
}

pSvpwm_Info Svpwm_init(PWM_Opt *opts)
{
    if(!check_pwm_opts(opts))
    {
        return NULL;
    }

    Svpwm_Opt *s_opts = malloc(sizeof(Svpwm_Opt));
    if(!s_opts)
    {
        return NULL;
    }
    *s_opts = svpwm_opts;
    s_opts->pwm_opts = opts;

    pSvpwm_Info svpwm = malloc(sizeof(Svpwm_Info));
    if(!svpwm)
    {
        free(s_opts);
        return NULL;
    }

    svpwm->udc = 5;
    svpwm->ts = 3800;

    svpwm->s_opts = s_opts;
    return svpwm;
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


