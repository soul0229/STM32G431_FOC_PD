#ifndef __SENSOR_H__
#define __SENSOR_H__
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SENSOR_TYPE_HALL = 0,
    SENSOR_TYPE_ENCODER,
    SENSOR_TYPE_MAX
} SensorType_t;

typedef enum
{
    DIR_BACKWARD = -1,
    DIR_INVALID = 0,
    DIR_FORWARD = 1,
    DIR_MAX
} Direction_t;

typedef struct
{
    uint16_t (*get_angle)(void *this);
    bool (*init)(void *this);
    SensorType_t type;
    void *priv;
} Sensor_t;

#endif
