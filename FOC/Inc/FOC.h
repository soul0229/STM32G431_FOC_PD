#ifndef __FOC_H__
#define __FOC_H__

#include "FocCommon.h"
#include "Svpwm.h"
#include <stdint.h>

extern volatile uint32_t gFoc_TimeCNT;
struct PI_Struct
{
    float kp;
    float ki;
    float kd;

    float pre;
    float tar;
    float bias;
    float lastBias;
    float out;
    float outMax;
};

typedef struct
{
    void *private;
    void (*ADC_Init)(void*);
    uint16_t (*getADCSample[3])(void*);
    float amplifier;
    float adcRefVolt;
    uint32_t offset[3];
    uint16_t averageCnt;
    uint8_t mOhm;
} ADC_Info;

typedef struct
{
    bool isEnable;
    uint8_t polePairs;
	
	uint8_t rNum;
    int16_t ia;
    int16_t ib;
	int16_t ic;

    float iAlpha;
    float iBeta;
    float iq;
    float id;

    float tar_iq;
    float tar_id;
    float angle;            // 角度
    uint16_t radian;        // 弧度

    float iAlphaSVPWM;      // park反变换后 后 Iα
    float iBetaSVPWM;       // park反变换后 后 Iβ

    struct PI_Struct idPID;
    struct PI_Struct iqPID;

    pSvpwm_Info Svpwm;
    ADC_Info *adc;
    void (*EnableMotor)(void *this, bool);
    float (*GetEncoderAngle)(void *this);
    void (*GetPreCurrent)(void *this);
} FOC_Info;
typedef FOC_Info* pFOC_Info;

bool FOC_init(pFOC_Info pFOC, PWM_Opt *opts, ADC_Info *adcOpts);
void FocControl(pFOC_Info pFOC);

extern ADC_HandleTypeDef ADC_PORT;
extern int16_t ADC_offset[2][RESIST_NUM];
extern int16_t *ADC_Point;

#endif /* FOC_h */
