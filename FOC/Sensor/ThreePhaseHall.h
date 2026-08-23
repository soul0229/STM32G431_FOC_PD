#ifndef __THREE_PHASE_HALL_H__
#define __THREE_PHASE_HALL_H__
#include "FocCommon.h"
#include <stdint.h>

#define SAMPLE_CNT		(600)
#define FORCE_PULL		(64)

typedef struct
{
	uint16_t state_value[DIR_MAX][MAX_SECTOR];
} state_t;
typedef struct
{
	uint16_t 	(*trans_state)[DIR_MAX][MAX_SECTOR];
	uint16_t 	samp_cnt;
	uint16_t 	inc_value;
	uint16_t 	sector_ofs;
	Sector_t 	sector;
	Direction_t dir;
	uint16_t 	state_value[0][DIR_MAX][MAX_SECTOR];
} HallData_t;

#endif
