#ifndef __SVPWM_H__
#define __SVPWM_H__
#include "FocCommon.h"
#include <stdint.h>

enum
{
    PHASE_U = 0,
    PHASE_V,
    PHASE_W,
    PHASE_INT,
    PHASE_MAX
};

typedef enum
{
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6,
    MAX_SECTOR
} Sector;

typedef struct 
{
    void *priv;
    void (*init)(void *priv);
    void (*enable)(void *priv, bool);
    void (*SetPWM[PHASE_MAX])(void *priv, int16_t);
} PWM_Opt;

typedef struct
{
    PWM_Opt pwm_opts;
    void (*SectorJudgment)(void *this);
    void (*VectorTime)(void *this);
    void (*Generate)(void *this);
    void (*SvpwmControl)(void *this);
    Sector (*GetSector)(void *this);

    float udc;
    float u_alpha;
    float u_beta;
    float u1;
    float u2;
    float u3;

    float t0;
    float t1;
    float t2;
    float t3;
    float t4;
    float t5;
    float t6;
    float t7;
    int32_t ts;
    int32_t maxTs;
    int32_t adcTs;
    Sector sector;
} Svpwm_t;

Svpwm_t *Svpwm_init(PWM_Opt *pPWM_opts, void *priv);
void Svpwm_deinit(Svpwm_t *pSvpwm);

typedef struct {
	uint8_t header[1];
	uint16_t PWM[PHASE_MAX];
    uint8_t HallA[3];
    uint8_t HallB[3];
    uint8_t Dirction[2];
    int16_t adc_value[1];
    uint32_t angle;
    int32_t value;
    uint32_t speed;
    uint8_t tail[1];
} __attribute__((packed)) FocParam;

#endif /* Svpwm_h */
