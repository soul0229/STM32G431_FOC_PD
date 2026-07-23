#ifndef __SVPWM_H__
#define __SVPWM_H__

#include <stdint.h>
#include <stdbool.h>
typedef enum
{
    PHASE_U = 0,
    PHASE_V,
    PHASE_W,
    PHASE_INT,
    PHASE_MAX
} Phase_t;

typedef enum
{
    SECTOR_INVALID_0 = 0,
    SECTOR_1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6,
    SECTOR_INVALID_7,
    MAX_SECTOR
} Sector_t;

typedef struct 
{
    void *priv;
    void (*enable)(void *priv, bool);
    void (*SetPWM)(void *priv, uint16_t *);
    float udc;
    uint16_t cycle;
} PWM_Opt;

typedef struct
{
    PWM_Opt PwmOpts;
    void (*Init)(void *this);
    void (*SectorJudgment)(void *this);
    void (*VectorTime)(void *this);
    void (*Generate)(void *this);

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
    float tk;
    uint16_t ts;
    uint16_t adcTs;
    Sector_t sector;

    uint16_t PWM[PHASE_MAX];
} Svpwm_t;

Svpwm_t *Svpwm_init(PWM_Opt *pPWM_opts, void *priv);
void Svpwm_deinit(Svpwm_t *pSvpwm);

typedef struct {
	uint16_t header;
    int16_t adc_value[3];
    float   Id;
    float   Iq;
    uint16_t tail;
} FocParam_t;

#endif /* Svpwm_h */
