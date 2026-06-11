#include "FocCommon.h"
/* common */

bool sensor_register(FOC_t *pFOC, const Sensor_t *pSensor)
{
	if(pFOC == NULL || pSensor == NULL)
	{
		return false;
	}
	if(!pSensor->get_angle || !pSensor->init)
	{
		return false;
	}

	pFOC->pSensor = (Sensor_t*)malloc(sizeof(Sensor_t));
	if(pFOC->pSensor == NULL)
	{
		return false;
	}

	*pFOC->pSensor = *pSensor;
	return true; // Sensor registered successfully
}

bool sensor_unregister(FOC_t *pFOC)
{
	if(pFOC == NULL)
	{
		return false;
	}
	if(pFOC->pSensor)
	{
		free(pFOC->pSensor);
	}
	
	return true; // Sensor registered successfully
}
