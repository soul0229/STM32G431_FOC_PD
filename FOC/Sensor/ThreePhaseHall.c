#include "ThreePhaseHall.h"
#include <stdint.h>
#include <stdlib.h>

static const int8_t hall_dir_map[MAX_SECTOR][MAX_SECTOR] = {
	[SECTOR_1] = {[SECTOR_3] = DIR_BACKWARD, [SECTOR_5] = DIR_FORWARD},
	[SECTOR_5] = {[SECTOR_1] = DIR_BACKWARD, [SECTOR_4] = DIR_FORWARD},
	[SECTOR_4] = {[SECTOR_5] = DIR_BACKWARD, [SECTOR_6] = DIR_FORWARD},
	[SECTOR_6] = {[SECTOR_4] = DIR_BACKWARD, [SECTOR_2] = DIR_FORWARD},
	[SECTOR_2] = {[SECTOR_6] = DIR_BACKWARD, [SECTOR_3] = DIR_FORWARD},
	[SECTOR_3] = {[SECTOR_2] = DIR_BACKWARD, [SECTOR_1] = DIR_FORWARD},
};

static bool three_phase_hall_init(void *this)
{
	// Hall sensor initialization code
	Sensor_t *pSensor = (Sensor_t *)this;

	// Initialize the sensor's private data if needed
	pSensor->type 		= SENSOR_TYPE_HALL;
	pSensor->priv_data 	= malloc(sizeof(HallData_t)); // Example of allocating private data
	if (pSensor->priv_data == NULL)
	{
		return false;
	}
	memset(pSensor->priv_data, 0x00, sizeof(HallData_t));
	((HallData_t*)pSensor->priv_data)->inc_value = FORCE_PULL;

	return true;
}


static bool three_phase_hall_get_angle(void *this)
{
	// Hall sensor initialization code
	Sensor_t 	*pSensor = (Sensor_t *)this;
	HallData_t 	*data 	= pSensor->priv_data;
	if(data->samp_cnt < SAMPLE_CNT)
	{
		(*pSensor->angle) += data->inc_value;
		return false;
	}

	*pSensor->angle = data->trans_state[data->dir][data->sector] + data->sector_ofs;
	data->sector_ofs += data->inc_value;
	return true;
}


/* your update code */

static void three_phase_hall_update(void *this, void *priv, uint16_t speed)
{
	// Hall sensor initialization code
	Sensor_t 		*pSensor 	= (Sensor_t *)this;
	HallData_t 		*data 		= pSensor->priv_data;
	Sector_t 		cur_sector 	= *(Sector_t*)priv;

	data->dir 			= hall_dir_map[data->sector][cur_sector];
	data->sector 		= cur_sector;
	data->sector_ofs 	= 0;

	if(data->samp_cnt++ < SAMPLE_CNT)
	{
		if(data->dir != DIR_INVALID)
		{
			data->trans_state[data->dir][cur_sector] += *pSensor->angle;
			data->trans_state[data->dir][cur_sector] /= 2;
		}
		return;
	}

	data->inc_value = (data->inc_value + speed) / 2;
}

const Sensor_t sensor = 
{
	.init		= three_phase_hall_init,
	.update		= three_phase_hall_update,
	.getAngle 	= three_phase_hall_get_angle,
};


