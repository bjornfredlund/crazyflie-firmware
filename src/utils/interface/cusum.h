#ifndef CUSUM_H
#define CUSUM_H

#include "linearfunction.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "stdint.h"

typedef struct{ 
	float threshold;
	float drift;
	float lambda; // forgetting factor, = 1 will result in normal CUSUM test
} cusumParams_t;


typedef struct { 
	float theta;
	float sumPositive;
	float sumNegative;
} cusumData_t;


typedef struct{ 
	uint32_t firstViolation;
	int violations;
	int runTestViolationsPermitted;
	uint32_t runTestWindow;
	bool runTestPreviouslyTriggered;
} runTest_t;

typedef struct { 
	cusumParams_t params;
	linearFuncParams_t linearFunc;
	runTest_t runTest;
	uint16_t stateSignature;
} deckParams_t;


void cusumInit(cusumData_t*, cusumParams_t*);
void cusumUpdate(cusumData_t*, cusumParams_t*, float);
void cusumReset(cusumData_t*, float);
bool cusumThresholdExceeded(cusumData_t*, float);
bool cusumRunTestFaultOver(runTest_t*, uint32_t);

#endif
