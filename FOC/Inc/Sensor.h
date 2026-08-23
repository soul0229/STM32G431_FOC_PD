#ifndef __SENSOR_H__
#define __SENSOR_H__
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SENSOR_TYPE_HALL        = 0,
    SENSOR_TYPE_ENCODER,
    SENSOR_TYPE_MAX
} SensorType_t;

typedef enum
{
    DIR_BACKWARD    = -1,
    DIR_INVALID     = 0,
    DIR_FORWARD     = 1,
    DIR_MAX         = 3
} Direction_t;

typedef struct
{
    void            *priv;
    SensorType_t    type;
    uint8_t         polePairs;
    bool            (*init)(void *this);
    void            (*update)(void *this, void *priv, uint16_t);
    bool            (*getAngle)(void *this);
    uint16_t        *angle;
    void            *priv_data;
} Sensor_t;

#endif
