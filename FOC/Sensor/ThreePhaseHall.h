#ifndef __THREE_PHASE_HALL_H__
#define __THREE_PHASE_HALL_H__
#include "Sensor.h"

typedef struct
{
	uint16_t hall_trans_state[DIR_MAX][MAX_SECTOR];	
} HallSensorData;

#endif
