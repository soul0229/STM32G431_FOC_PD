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
    
    DIR_INVALID     = 0,
    DIR_FORWARD     = 1,
    DIR_BACKWARD    = 2,
    DIR_MAX
} Direction_t;

typedef struct
{
    void            *priv;
    SensorType_t    type;
    bool            (*init)(void *this);
    void            (*update)(void *this, void *priv, uint16_t);
    bool            (*getAngle)(void *this);
    uint16_t        *angle;
    void            *priv_data;
} Sensor_t;

#endif
