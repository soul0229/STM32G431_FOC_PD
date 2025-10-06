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
    bool isEnable;
    uint8_t polePairs;
	
	uint8_t rNum;
    float ia;
    float ib;
	float ic;

    float iAlpha;
    float iBeta;
    float iq;
    float id;

    float tar_iq;
    float tar_id;
    float angle;            // 角度
    int16_t radian;         // 弧度

    float iAlphaSVPWM;      // park反变换后 后 Iα
    float iBetaSVPWM;       // park反变换后 后 Iβ

    struct PI_Struct idPID;
    struct PI_Struct iqPID;

    pSvpwm_Info Svpwm;
    float (*GetEncoderAngle)(void);
    void (*GetPreCurrent)(float *ua,float *ub,float *uc);
    
} FOC_Info;
typedef FOC_Info* pFOC_Info;

bool FOC_init(pFOC_Info pFOC, PWM_Opt *opts);
void FocControl(pFOC_Info pFOC);
void MotorEnable(pFOC_Info pFOC, bool enable);

extern ADC_HandleTypeDef ADC_PORT;
extern int16_t ADC_offset[2][RESIST_NUM];
extern int16_t *ADC_Point;

#endif /* FOC_h */
