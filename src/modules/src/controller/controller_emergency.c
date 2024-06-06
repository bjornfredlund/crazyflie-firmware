
#include "controller_emergency.h"
#include "string.h"
#include "debug.h"
#include "freeRTOS.h"
#include "task.h"


void defaultLevelout(setpoint_t* setpoint)
{ 
	setpoint->mode.x = modeDisable;	
	setpoint->mode.y = modeDisable;	
	setpoint->mode.roll = modeAbs;	
	setpoint->mode.pitch = modeAbs;	
	setpoint->mode.yaw = modeVelocity;

	setpoint->attitude.roll = 0;
	setpoint->attitude.pitch = 0;
	setpoint->attitudeRate.yaw = 0;
} 

void noAction()
{ 
		
} 


void setpointOverrideLevelout(const setpoint_t* setpointIn, setpoint_t* setpointOut)
{ 
		memcpy(setpointOut, setpointIn, sizeof(setpoint_t));
} 



void accelerationDescent(setpoint_t* setpoint)
{ 
	static uint32_t initialTick = 0;
	setpoint->mode.z = modeAcc;

	static bool controllerSwitched = false;
	if(!controllerSwitched)
	{ 
		DEBUG_PRINT("Switching controller mode\n");
		controllerSwitched = true;
	}

	uint32_t nowTick = xTaskGetTickCount();

	if(initialTick == 0)
	{ 
		initialTick = xTaskGetTickCount();
	} 

	if((nowTick - initialTick) > 1000)
	{ 
		setpoint->acceleration.z = 0.0f;
	}  
	else {
		setpoint->acceleration.z = -1.0f;
	}
} 
