#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>
#include <stdbool.h>
#include "Svpwm.h"
#include "Sensor.h"
#include "RsSamp.h"


typedef struct
{
    float kp;
    float ki;
    float kd;

    float kb;
    float iSum;
    float out;
    float outMax;
} PID_t;

typedef enum
{
    PHASE_A,
    PHASE_B,
    PHASE_C,
    PHASE_NUM
} Ampere_Phase_t;


typedef void(cb_t)(void *this);

typedef struct
{
    bool        isEnable;
	uint8_t     rNum;
    
    int16_t     ia;
    int16_t     ib;
	int16_t     ic;

    int16_t     iAlpha;
    int16_t     iBeta;

    float       iq;
    float       id;

    float       tar_iq;
    float       tar_id;
    
    uint16_t    radian;        // 弧度

    PID_t       idPID;
    PID_t       iqPID;

    Svpwm_t     *pSvpwm;
    RsSamp_t    *pRsSamp;
    Sensor_t    *pSensor;
    void        (*sin_cos_65536)(uint16_t angle, float *sine, float *cosine);
    void        (*Enable)(void *this);
    float       (*GetEncoderAngle)(void *this);
    void        (*GetPreCurrent)(void *this);
    cb_t        *callback;
} FOC_t;

void sin_cos_65536(uint16_t angle, float *sine, float *cosine);

#endif /* FOC_h */
