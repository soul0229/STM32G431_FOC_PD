#include "FocCommon.h"
#include "ThreePhaseHall.h"
#include <stdlib.h>

const int8_t three_phase_hall_dir_map[MAX_SECTOR][MAX_SECTOR] = {
	[SECTOR_1] = {[SECTOR_3] = DIR_BACKWARD, [SECTOR_5] = DIR_FORWARD},
	[SECTOR_5] = {[SECTOR_1] = DIR_BACKWARD, [SECTOR_4] = DIR_FORWARD},
	[SECTOR_4] = {[SECTOR_5] = DIR_BACKWARD, [SECTOR_6] = DIR_FORWARD},
	[SECTOR_6] = {[SECTOR_4] = DIR_BACKWARD, [SECTOR_2] = DIR_FORWARD},
	[SECTOR_2] = {[SECTOR_6] = DIR_BACKWARD, [SECTOR_3] = DIR_FORWARD},
	[SECTOR_3] = {[SECTOR_2] = DIR_BACKWARD, [SECTOR_1] = DIR_FORWARD},
};

int8_t get_three_phase_hall_dir(Sector_t last_sector, Sector_t current_sector)
{
    return three_phase_hall_dir_map[last_sector][current_sector];
}

bool three_phase_hall_init(void *this)
{
	// Hall sensor initialization code
	Sensor_t *sensor = (Sensor_t *)this;
	// Initialize the sensor's private data if needed
	sensor->type = SENSOR_TYPE_HALL;
	sensor->priv = malloc(sizeof(HallSensorData)); // Example of allocating private data
	if (sensor->priv == NULL)
	{
		return false; // Allocation failed
	}
	// Initialize the
	return true; // Initialization successful
}

