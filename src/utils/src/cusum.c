#include "cusum.h"
#include <stdbool.h>
#include "debug.h"


void cusumInit(cusumData_t* data, cusumParams_t* params)
{ 
	memset(data, 0, sizeof(cusumData_t));
} 


void cusumUpdate(cusumData_t* data, cusumParams_t* params, float sample)
{ 
	float theta = params->lambda*data->theta + (1.0f - params->lambda)*sample;

	float eps = sample - theta; // - params->drift?

	data->sumPositive = fmax(data->sumPositive + eps - params->drift, 0);
	data->sumNegative = fmin(data->sumNegative - eps + params->drift, 0);

	data->theta = theta;
}

bool cusumThresholdExceeded(cusumData_t* data, float threshold)
{ 
	return data->sumPositive > threshold || data->sumNegative < -threshold;
} 

void cusumReset(cusumData_t* data, float y_t)
{ 
	data->theta = y_t;
	data->sumPositive = 0;
	data->sumNegative = 0;
} 
bool cusumRunTestFaultOver(runTest_t* runTest, uint32_t nowTick)
{ 
	if(!runTest->runTestPreviouslyTriggered)
	{ 
		return false;

	} 
	uint32_t elapsedSinceLastViolation = nowTick - runTest->firstViolation;
	uint32_t window = runTest->runTestWindow;

	return elapsedSinceLastViolation > window;
} 
