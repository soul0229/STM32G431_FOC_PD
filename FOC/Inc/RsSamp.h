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

typedef struct 
{
    bool        (*init)(void *this);
    void        (*fliter)(void *this, int16_t *);
    uint16_t    cutOffFreq;
    uint16_t    sampFreq;
    void        *priv_data;
} RsLPF_t;

typedef struct
{
    void        *priv;
    void        (*init)(void *this);
    bool        (*getSample)(void *this, int16_t *);
    float       amplifier;
    float       RefVolt;
    uint16_t    maxValue;
    uint16_t    Average_cnt;
    uint32_t    offset[RESISTOR_MAX];
    uint16_t    sampValue[RESISTOR_MAX];
    uint8_t     mOhm;
    uint8_t     resistor_num;
} ADC_t;

typedef struct
{
    ADC_t       *pADC;
    RsLPF_t     *pFilter;
    bool        (*init)(void *this);
    bool        (*getAmpere)(void *this);
    float       toAmpere;
    int16_t     currect[RESISTOR_MAX];
} RsSamp_t;



#endif
