#include "FocCommon.h"

bool Sensor_register(FOC_t *pFOC, const Sensor_t *pSensor, void *priv)
{
	if(pFOC == NULL || pSensor == NULL)
	{
		return false;
	}
	if(!pSensor->getAngle || !pSensor->init || !pSensor->update)
	{
		return false;
	}

	pFOC->pSensor = (Sensor_t*)malloc(sizeof(Sensor_t));
	if(pFOC->pSensor == NULL)
	{
		return false;
	}

	*pFOC->pSensor 			= *pSensor;
	pFOC->pSensor->angle 	= &pFOC->radian;
	pFOC->pSensor->priv 	= priv;
	return true; // Sensor registered successfully
}

bool Sensor_unregister(FOC_t *pFOC)
{
	if(pFOC == NULL)
	{
		return false;
	}

	if(pFOC->pSensor->priv_data)
	{
		free(pFOC->pSensor->priv_data);
	}

	if(pFOC->pSensor)
	{
		free(pFOC->pSensor);
	}
	
	return true; // Sensor unregistered successfully
}
