#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "FocCommon.h"


#define ADC_AVERAGE_NUM         (16)

#define PRINTF_BUF_NUM          2
#define PRINTF_SUBBUF_NUM       16
#define PRINTF_SUBBUF_MASK      (PRINTF_SUBBUF_NUM - 1)
#define PRINTF_TOTAL_MASK       (PRINTF_SUBBUF_NUM * PRINTF_BUF_NUM - 1)


typedef struct __attribute__((packed))
{
	uint16_t    header;
    // float       Id;
    // float       Iq;
    // float       Id_out;
    // float       Iq_out;
    uint16_t    angle;
    float       Id_out;
    float       Iq_out;
    uint16_t    adc_value[3];
    // uint16_t    hall_angle;
    // uint8_t     sector;
    // int8_t      dir;
    uint16_t    tail;
} FocParam_t;


FOC_t *motor_init(void *tim_priv, void *adc_priv, void *sensor_priv);

#endif