#include "linearfunction.h"

float linearFunctionApply(linearFuncParams_t* params, float x)
{ 
		return params->k*x + params->m;
} 
