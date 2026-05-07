#ifndef __SVPWM_H__
#define __SVPWM_H__
#include "FocCommon.h"
#include <stdint.h>

enum
{
    PHASE_U = 0,
    PHASE_V,
    PHASE_W,
    INT,
    Total
};

typedef enum
{
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} Sector;

typedef struct 
{
    void (*init)(void*);
    void (*enable)(void* , bool);
    void (*SetPWM[Total])(void*, int16_t);
    void *private;
} PWM_Opt;

typedef struct 
{
    void (*SectorJudgment)(void*);
    void (*VectorTime)(void*);
    void (*Generate)(void*);
    void (*SvpwmControl)(void*);
    Sector (*GetSector)(void*);
    PWM_Opt *pwm_opts;
} Svpwm_Opt;

typedef struct
{
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
    Svpwm_Opt *s_opts;
} Svpwm_Info;
typedef Svpwm_Info* pSvpwm_Info;

pSvpwm_Info Svpwm_init(PWM_Opt *opts);
void Svpwm_deinit(pSvpwm_Info svpwm);

typedef struct {
	uint8_t header[1];
	// uint16_t PWM[Total];
    // uint8_t HallA[3];
    // uint8_t HallB[3];
    // uint8_t Dirction[2];
    int16_t adc_value[1];
    // uint32_t angle;
    // int32_t value;
    // uint32_t speed;
    uint8_t tail[1];
} __attribute__((packed)) FocParam;

#endif /* Svpwm_h */
