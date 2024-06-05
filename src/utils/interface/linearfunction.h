#ifndef LINEAR_FUNCTION_H
#define LINEAR_FUNCTION_H

typedef struct{ 
	float k;
	float m;
} linearFuncParams_t;

float linearFunctionApply(linearFuncParams_t*, float);

#endif
