#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal_flash.h"
#include "Svpwm.h"
#include "Sensor.h"
#include "RsSamp.h"


typedef struct
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
} PI_t;

typedef struct
{
    bool        isEnable;
    uint8_t     polePairs;
	
	uint8_t     rNum;
    int16_t     ia;
    int16_t     ib;
	int16_t     ic;

    float       iAlpha;
    float       iBeta;
    float       iq;
    float       id;

    float       tar_iq;
    float       tar_id;
    float       angle;            // 角度
    uint16_t    radian;        // 弧度

    float       iAlphaSVPWM;      // park反变换后 后 Iα
    float       iBetaSVPWM;       // park反变换后 后 Iβ

    PI_t        idPID;
    PI_t        iqPID;

    Svpwm_t     *pSvpwm;
    RsSamp_t    *pRsSamp;
    Sensor_t    *pSensor;
    void        (*EnableMotor)(void *this, bool);
    float       (*GetEncoderAngle)(void *this);
    void        (*GetPreCurrent)(void *this);
} FOC_t;

extern volatile uint32_t gFoc_TimeCNT;



extern ADC_HandleTypeDef ADC_PORT;
// extern int16_t ADC_offset[2][RESIST_NUM];
extern int16_t *ADC_Point;

#endif /* FOC_h */
