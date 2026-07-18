#ifndef __RSAMP_H__
#define __RSAMP_H__
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    RESISTOR_U = 0,
    RESISTOR_V,
    RESISTOR_W,
    RESISTOR_MAX
} Resistor_t;

typedef struct {
    float alpha;
    float cutOffFreq;
    float sampFreq;
    int16_t prvValue[RESISTOR_MAX];
} RsSampLPF_t;

typedef struct
{
    void        *priv;
    void        (*Init)(void *priv);
    uint16_t    (*getRsSample[RESISTOR_MAX])(void *priv);
    void        (*ADCGetPreCurrent)(void *this);
    void        *filterPriv;
    float       amplifier;
    float       sampRefVolt;
    float       sampOfsVolt;
    uint16_t    sampMaxValue;
    uint8_t     mOhm;
    

    uint32_t    offset[RESISTOR_MAX];
    uint16_t    averageCnt;
} RsSamp_t;



#endif
