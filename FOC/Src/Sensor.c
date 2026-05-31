#include "FocCommon.h"
/* common */

bool sensor_register(FOC_t *pFOC, Sensor_t *pSensor)
{
	if(pFOC == NULL || pSensor == NULL)
	{
		return false;
	}
	pFOC->pSensor = pSensor;
	return true; // Sensor registered successfully
}

