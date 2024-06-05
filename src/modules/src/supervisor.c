/*
*    ||          ____  _ __
* +------+      / __ )(_) /_______________ _____  ___
* | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
* +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
*  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
*
* Crazyflie control firmware
*
* Copyright (C) 2021 - 2023 Bitcraze AB
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, in version 3.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* supervisor.c - Keep track of system state
*/

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "FreeRTOS.h"
#include "task.h"

#include "cf_math.h"
#include "log.h"
#include "param.h"
#include "motors.h"
#include "power_distribution.h"
#include "supervisor.h"
#include "stabilizer.h"
#include "supervisor_state_machine.h"
#include "supervisor_board_disconnected.h"
#include "platform_defaults.h"
#include "crtp_localization_service.h"
#include "system.h"
#include "physicalConstants.h"
#include "autoconf.h"
#include "eventtrigger.h"
#include "controller_emergency.h"

#define DEBUG_MODULE "SUP"
#include "debug.h"


static setpoint_t defaultSetpoint = { 
		.mode.x = modeDisable,
		.mode.y = modeDisable,
		.mode.z = modeVelocity,
		.mode.roll = modeAbs,
		.mode.pitch = modeAbs,
		.attitude.roll = 0,
		.attitude.pitch = 0,
		.velocity.z = 0,
};


#define DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT (M2T(1000))

// The minimum time (in ms) we need to see low thrust before saying that we are not flying anymore
#define IS_FLYING_HYSTERESIS_THRESHOLD M2T(2000)

#define COMMANDER_WDT_TIMEOUT_STABILIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

#ifndef CONFIG_MOTORS_REQUIRE_ARMING
  #define AUTO_ARMING 1
#else
  #define AUTO_ARMING 0
#endif

static uint16_t landingTimeoutDuration = LANDING_TIMEOUT_MS;


typedef struct {
  bool canFly;
  bool isFlying;
  bool isTumbled;
  bool isArmingActivated;
  bool isCrashed;
  uint16_t infoBitfield;
  uint8_t paramEmergencyStop;

  // Deprecated, remove after 2024-06-01
  int8_t deprecatedArmParam;

  // The time (in ticks) of the first tumble event. 0=no tumble
  uint32_t initialTumbleTick;

  // The time (in ticks) of the latest high thrust event. 0=no high thrust event yet
  uint32_t latestThrustTick;

  // The time (in ticks) of the latest landing event. 0=no landing event yet
  uint32_t latestLandingTick;

  supervisorState_t state;

  safeDescentInfo_t crashInfo;

  boardStatus_t bs;

  // Copy of latest conditions, for logging
  supervisorConditionBits_t latestConditions;
  uint8_t doinfodump;
} SupervisorMem_t;

static SupervisorMem_t supervisorMem;

const static setpoint_t nullSetpoint;

void infoDump(const SupervisorMem_t* this);

bool supervisorCanFly() {
  return supervisorMem.canFly;
}

bool supervisorIsFlying() {
  return supervisorMem.isFlying;
}

bool supervisorIsTumbled() {
  return supervisorMem.isTumbled;
}

bool supervisorCanArm() {
  return supervisorStatePreFlChecksPassed == supervisorMem.state;
}

bool supervisorIsArmed() {
  return supervisorMem.isArmingActivated || supervisorMem.deprecatedArmParam;
}

bool supervisorIsLocked() {
  return supervisorStateLocked == supervisorMem.state;
}

bool supervisorIsCrashed() {
  return supervisorMem.isCrashed;
}

static void supervisorSetLatestLandingTime(SupervisorMem_t* this, const uint32_t currentTick) {
  this->latestLandingTick = currentTick;
}

bool supervisorIsLandingTimeout(SupervisorMem_t* this, const uint32_t currentTick) {
  if (0 == this->latestLandingTick) {
    return false;
  }

  const uint32_t landingTime = currentTick - this->latestLandingTick;
  return landingTime > M2T(landingTimeoutDuration);
}

bool supervisorRequestCrashRecovery(const bool doRecovery) {

  if (doRecovery && !supervisorIsCrashed()) {
    return true;
  } else if (doRecovery && supervisorIsCrashed() && !supervisorIsTumbled()) {
    supervisorMem.isCrashed = false;
    return true;
  } else if (!doRecovery) {
    supervisorMem.isCrashed = true;
    return true;
  }

  return false;
}

bool supervisorRequestArming(const bool doArm) {
  if (doArm == supervisorMem.isArmingActivated) {
    return true;
  }

  if (doArm && !supervisorCanArm()) {
    return false;
  }

  supervisorMem.isArmingActivated = doArm;
  return true;
}

//
// We say we are flying if one or more motors are running over the idle thrust.
//
static bool isFlyingCheck(SupervisorMem_t* this, const uint32_t tick) {
  bool isThrustOverIdle = false;
  const uint32_t idleThrust = powerDistributionGetIdleThrust();
  for (int i = 0; i < NBR_OF_MOTORS; ++i) {
    const uint32_t ratio = powerDistributionMotorType(i) * motorsGetRatio(i);
    if (ratio > idleThrust) {
      isThrustOverIdle = true;
      break;
    }
  }

  if (isThrustOverIdle) {
    this->latestThrustTick = tick;
  }

  bool result = false;
  if (0 != this->latestThrustTick) {
    if ((tick - this->latestThrustTick) < IS_FLYING_HYSTERESIS_THRESHOLD) {
      result = true;
    }
  }

  return result;
}

//
// Tumbling is defined as being tilted a bit for some time, or closer to up side down for a shorter time.
// Free falling is considered a valid flight mode.
//
// Once a tumbled situation is identified, we can use this for instance to cut
// the thrust to the motors, avoiding the Crazyflie from running propellers at
// significant thrust when accidentally crashing into walls or the ground.
//
static bool isTumbledCheck(SupervisorMem_t* this, const sensorData_t *data, const uint32_t tick) {
  const float freeFallThreshold = 0.1;

  const float acceptedTiltAccZ = 0.5;  // 60 degrees tilt (when stationary)
  const uint32_t maxTiltTime = M2T(1000);

  const float acceptedUpsideDownAccZ = -0.2;
  const uint32_t maxUpsideDownTime = M2T(100);

  const bool isFreeFalling = (fabsf(data->acc.z) < freeFallThreshold && fabsf(data->acc.y) < freeFallThreshold && fabsf(data->acc.x) < freeFallThreshold);
  if (isFreeFalling) {
    // Falling is OK, reset
    this->initialTumbleTick = 0;
  }

  const bool isTilted = (data->acc.z < acceptedTiltAccZ);
  if(isTilted) {  // Will also be true for up side down
    if (0 == this->initialTumbleTick) {
      // Start the clock
      this->initialTumbleTick = tick;
    }

    const uint32_t ticksBeingTumbled = tick - this->initialTumbleTick;

    const bool isUpSideDown = (data->acc.z < acceptedUpsideDownAccZ);
    if (isUpSideDown && (ticksBeingTumbled > maxUpsideDownTime)) {
      return true;
    }

    if (ticksBeingTumbled > maxTiltTime) {
      return true;
    }
  } else {
    // We're OK, reset
    this->initialTumbleTick = 0;
  }

  return false;
}

static bool checkEmergencyStopWatchdog(const uint32_t tick) {
  bool isOk = true;

  const uint32_t latestNotification = locSrvGetEmergencyStopWatchdogNotificationTick();
  if (latestNotification > 0) {
    isOk = tick < (latestNotification + DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT);
  }

  return isOk;
}
static void computeSafeDescentTimeout(safeDescentInfo_t* descentInfo, const uint32_t tick)
{ 
	descentInfo->initialDescentTime = T2M(xTaskGetTickCount());
	float t = 1.0f; // default value
	if(descentInfo->recoveryInfo.descendSetpoint.mode.z == modeVelocity)
	{ 
		t = descentInfo->stateSnapshot.position.z / 0.3f;
	} 

	else if(descentInfo->recoveryInfo.descendSetpoint.mode.z == modeAbs)
	{ 
		t = descentInfo->stateSnapshot.position.z / PID_POS_VEL_Z_MAX;
	} 
	else if(descentInfo->recoveryInfo.descendSetpoint.mode.z == modeAcc)
	{ 
		t = 1.0f; //apply 1.0 m/^s2 for 1 second to achieve 1.0m/s velocity

		// distance traveled during the acceleration period
		float s = 0.5f*t*t*1.0f;

		descentInfo->stateSnapshot.position.z -= s;

		//t += descentInfo->stateSnapshot.position.z / 1.0f + 0.4f;// Safety factor
		t += descentInfo->stateSnapshot.position.z / 1.0f;// + 0.6f;// Safety factor
		DEBUG_PRINT("Initial descent setpoint %ld\n", tick);

		//t = descentInfo->stateSnapshot.position.z / 0.3f;
	} 
	else if(descentInfo->recoveryInfo.descendSetpoint.mode.z == modeRaw)
	{ 
			t = 2.5f;
	} 

	t = t < 0 ? 0 : t;
	descentInfo->descentTimeRequired = t;

	DEBUG_PRINT("Estimated required time to land %.6fs\n", (double)descentInfo->descentTimeRequired);


} 

static bool descentOverCheck(safeDescentInfo_t* crashInfo, const uint32_t tick)
{ 
	bool timesUp = false;

	if( 0 == crashInfo->initialDescentTime)
	{ 
		return timesUp;
	} 

	if((tick - crashInfo->initialDescentTime) > (uint32_t)(crashInfo->descentTimeRequired*1000)) // + crashInfo->leveloutTime*1000))
	{ 
		DEBUG_PRINT("times up at tick %ld\n", tick);
		timesUp = true;
	} 
	else{ 
		//DEBUG_PRINT("Processing descent\n");
	} 

	return timesUp;
} 

float MAX_ACC = 3.0f;
static void leveloutEval(safeDescentInfo_t* failsafeInfo)
{ 
	DEBUG_PRINT("last x %.6f\n", (double)failsafeInfo->stateSnapshot.position.x);
	DEBUG_PRINT("last y %.6f\n", (double)failsafeInfo->stateSnapshot.position.y);
	DEBUG_PRINT("last z %.6f\n", (double)failsafeInfo->stateSnapshot.position.z);

	DEBUG_PRINT("last vx %.6f\n", (double)failsafeInfo->stateSnapshot.velocity.x);
	DEBUG_PRINT("last vy %.6f\n", (double)failsafeInfo->stateSnapshot.velocity.y);
	DEBUG_PRINT("last vz %.6f\n", (double)failsafeInfo->stateSnapshot.velocity.z);
	float leveloutTimeout = 1.5f;
	if(failsafeInfo->recoveryInfo.leveloutSetpoint.mode.x == modeAcc || failsafeInfo->recoveryInfo.leveloutSetpoint.mode.y == modeAcc|| failsafeInfo->recoveryInfo.leveloutSetpoint.mode.z == modeAcc)
	{ 
		float lastVx = failsafeInfo->stateSnapshot.velocity.x;
		float lastVy = failsafeInfo->stateSnapshot.velocity.y;
		float lastVz = failsafeInfo->stateSnapshot.velocity.z;

		//float highest = fabs(lastVx) > fabs(lastVy) ? lastVx : lastVy;

		float max1 = fmaxf((fabs(lastVy)), fabs(lastVx));

		if((float)fabs(lastVz) > max1){ 
			MAX_ACC = 1.3f;
		} 

		float highest = fmaxf((fabs(lastVz)), max1);

		DEBUG_PRINT("Highest: %.6f\n", (double)highest);

		leveloutTimeout = (float)fabs(highest) / MAX_ACC;

		float axSetpoint = - lastVx / leveloutTimeout;
		float aySetpoint = - lastVy / leveloutTimeout;
		float azSetpoint = - lastVz / leveloutTimeout;

		failsafeInfo->recoveryInfo.leveloutSetpoint.acceleration.x = axSetpoint;
		failsafeInfo->recoveryInfo.leveloutSetpoint.acceleration.y = aySetpoint;
		failsafeInfo->recoveryInfo.leveloutSetpoint.acceleration.z = azSetpoint;

		float riseTime = 1.0f / (1.0f + expf(-10.0f*(leveloutTimeout-0.5f)));
		DEBUG_PRINT("riseTime = %.6f\n", (double)riseTime);


		DEBUG_PRINT("Levelout timeout %.6f\n", (double)leveloutTimeout);
		DEBUG_PRINT("axSetpoint: %.6f\n", (double)axSetpoint);
		DEBUG_PRINT("aySetpoint: %.6f\n", (double)aySetpoint);
		DEBUG_PRINT("azSetpoint: %.6f\n", (double)azSetpoint);
		//leveloutTimeout += riseTime;
		//setLeveloutTimeout(leveloutTimeout);
	} 
	if(failsafeInfo->recoveryInfo.leveloutSetpoint.mode.x == modeVelocity || failsafeInfo->recoveryInfo.leveloutSetpoint.mode.y == modeVelocity)
	{ 
		float lastVx = failsafeInfo->stateSnapshot.velocity.x;
		float lastVy = failsafeInfo->stateSnapshot.velocity.y;
		float lastVz = failsafeInfo->stateSnapshot.velocity.z;

		float vNorm = sqrtf(lastVx*lastVx + lastVy*lastVy + lastVz*lastVz); 
		//float leveloutTimeout = 2.0f/(1.0f+expf(0.1f*(vNorm - 1.5f)));
		leveloutTimeout = 0.4f*vNorm + 0.3f;

		leveloutTimeout = leveloutTimeout > 2.0f ? 2.0f : leveloutTimeout;
		DEBUG_PRINT("Vel levelout timeout: %.6f\n", (double)leveloutTimeout);
		//setLeveloutTimeout(leveloutTimeout);
		failsafeInfo->leveloutTime = leveloutTimeout;
	} 
	if(failsafeInfo->recoveryInfo.actionToExecute == ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN){
			leveloutTimeout = 2.5f;
	} 
	else if(failsafeInfo->recoveryInfo.actionToExecute == ACTIONBIT_LEVELOUT){
			leveloutTimeout = 2.5f;
	} 
	failsafeInfo->leveloutTime = leveloutTimeout;
	DEBUG_PRINT("leveloutTime set: %.6f\n", (double)failsafeInfo->leveloutTime);
} 


static void postTransitionActions(SupervisorMem_t* this, const supervisorState_t previousState, const state_t* latestState, const uint32_t currentTick) {
	const supervisorState_t newState = this->state;

	if (newState == supervisorStateReadyToFly) {
		DEBUG_PRINT("Ready to fly\n");
	}

	if (newState == supervisorStateLanded) {
		supervisorSetLatestLandingTime(this, currentTick);
	}

	if ((previousState == supervisorStateLanded) && (newState == supervisorStateReset)) {
		DEBUG_PRINT("Landing timeout, disarming\n");
	}

	if (newState == supervisorStateLocked) {
		DEBUG_PRINT("Locked, reboot required\n");
	}

	if (newState == supervisorStateCrashed) {
		DEBUG_PRINT("Crashed, recovery required\n");
		supervisorRequestCrashRecovery(false);
	}

	if(newState == supervisorStateSafeDescent){ 
		computeSafeDescentTimeout(&this->crashInfo, currentTick);
		DEBUG_PRINT("Initializing safe descent at %ld\n", currentTick);
	} 

	if(newState == supervisorStateWarningLevelOut){ 
		memcpy(&this->crashInfo.stateSnapshot, latestState, sizeof(state_t));
		leveloutEval(&this->crashInfo);
		DEBUG_PRINT("Leveling out at %ld!\n", currentTick);
	} 
	if(newState == supervisorStateFlying){ 
		DEBUG_PRINT("Flying %ld!\n", currentTick);
	} 


	if ((previousState == supervisorStateNotInitialized || previousState == supervisorStateReadyToFly || previousState == supervisorStateFlying) &&
			newState != supervisorStateReadyToFly && newState != supervisorStateFlying && newState != supervisorStateLanded && newState != supervisorStateWarningLevelOut) {
		DEBUG_PRINT("Can not fly\n");
	}

	if (newState != supervisorStateReadyToFly &&
			newState != supervisorStateFlying &&
			newState != supervisorStateWarningLevelOut &&
			newState != supervisorStateLanded) {
		supervisorRequestArming(false);
	}

	// We do not require an arming action by the user, auto arm
	if (AUTO_ARMING || this->deprecatedArmParam) {
		if (newState == supervisorStatePreFlChecksPassed) {
			supervisorRequestArming(true);
		}
	}
}


static supervisorConditionBits_t updateAndPopulateConditions(SupervisorMem_t* this, const sensorData_t *sensors, const setpoint_t* setpoint, const uint32_t currentTick) {
	supervisorConditionBits_t conditions = 0;

	if (supervisorIsArmed()) {
		conditions |= SUPERVISOR_CB_ARMED;
	}

	const bool isFlying = isFlyingCheck(this, currentTick);
	if (isFlying) {
		conditions |= SUPERVISOR_CB_IS_FLYING;
	}
	const bool startUpKalmanStatesNominal = startupKalmanStatesNominal();
	if(startUpKalmanStatesNominal)
	{ 
		conditions |= SUPERVISOR_CB_KALMAN_RATES_OK;
	} 

	const bool isTumbled = isTumbledCheck(this, sensors, currentTick);
	if (isTumbled) {
		conditions |= SUPERVISOR_CB_IS_TUMBLED;
	}

	// in this order to transition to levelout as fast as possible, otherwise due to timings, we might wait too long to prevent a crash
	const bool faultDetectionChange = isBoardErrorCheck(&this->crashInfo.recoveryInfo);
	if(faultDetectionChange && this->crashInfo.recoveryInfo.errorInit)
	{ 
		DEBUG_PRINT("Got faultDetectionChange and this is init\n");
		safeDescentInfo_t* failsafeInfo = &this->crashInfo;
		this->crashInfo.active = true;

		//default, gets recalculated after state transition
		failsafeInfo->leveloutTime = 1500;
	} 
	// there was an error but it disappeared, we need to transition back to flying if we previously had levelout or levelout or descent action selected
	if(faultDetectionChange && !this->crashInfo.recoveryInfo.errorInit){ 
		const safeDescentInfo_t* failsafeInfo = &this->crashInfo;
		DEBUG_PRINT("Got faultDetectionChange and this is disable\n");

		// if transition to rdToFly from notRdyTo fly 
		 /*
		  *
		  * if(!superivosrisFlying())
		  * { 
		  		conditions |= SUPERVISOR_CB_NONE; 	
		  * } 
		  *
		  *
		  */
		switch (failsafeInfo->recoveryInfo.actionToExecute)
		{ 
			case ACTIONBIT_LEVELOUT:
			case ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN:
				DEBUG_PRINT("Transitioning back\n");
				conditions |= SUPERVISOR_CB_NONE;
				// otherwise it was a safe descent and we should ignore the no error call
				this->crashInfo.active = false;
				break;
		} 
		//this->crashInfo.active = false;
	} 

	const bool isRecoveryActive = this->crashInfo.active;
	if(isRecoveryActive)
	{ 
		const safeDescentInfo_t* failsafeInfo = &this->crashInfo;

		supervisorConditionBits_t bit = getTransition(failsafeInfo, currentTick);
		conditions |= bit;
	}


	const bool safeDescentTimesUp = descentOverCheck(&this->crashInfo, currentTick);
	if(safeDescentTimesUp)
	{ 
		conditions |= SUPERVISOR_CB_EMERGENCY_STOP;
	} 

	const uint32_t setpointAge = currentTick - setpoint->timestamp;
	if (setpointAge > COMMANDER_WDT_TIMEOUT_STABILIZE) {
		memcpy(&this->crashInfo.recoveryInfo.leveloutSetpoint, &defaultSetpoint, sizeof(setpoint_t));
		conditions |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
	}
	if (setpointAge > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
		conditions |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
	}

	if (!checkEmergencyStopWatchdog(currentTick)) {
		conditions |= SUPERVISOR_CB_EMERGENCY_STOP;
	}

	if (locSrvIsEmergencyStopRequested()) {
		conditions |= SUPERVISOR_CB_EMERGENCY_STOP;
	}

	if (this->paramEmergencyStop) {
		conditions |= SUPERVISOR_CB_EMERGENCY_STOP;
	}

	if (supervisorIsCrashed()) {
		conditions |= SUPERVISOR_CB_CRASHED;
	}

	if (supervisorIsLandingTimeout(this, currentTick)) {
		conditions |= SUPERVISOR_CB_LANDING_TIMEOUT;
	}

	return conditions;
}

static void updateLogData(SupervisorMem_t* this, const supervisorConditionBits_t conditions) {
	this->canFly = supervisorAreMotorsAllowedToRun();
	this->isFlying = (this->state == supervisorStateFlying) || (this->state == supervisorStateWarningLevelOut);
	this->isTumbled = (conditions & SUPERVISOR_CB_IS_TUMBLED) != 0;

	this->infoBitfield = 0;
	if (supervisorCanArm()) {
		this->infoBitfield |= 0x0001;
	}
	if (supervisorIsArmed()) {
		this->infoBitfield |= 0x0002;
	}
	if(AUTO_ARMING || this->deprecatedArmParam) {
		this->infoBitfield |= 0x0004;
	}
	if (this->canFly) {
		this->infoBitfield |= 0x0008;
	}
	if (this->isFlying) {
		this->infoBitfield |= 0x0010;
	}
	if (this->isTumbled) {
		this->infoBitfield |= 0x0020;
	}
	if (supervisorStateLocked == this->state) {
		this->infoBitfield |= 0x0040;
	}
	if (this->isCrashed) {
		this->infoBitfield |= 0x0080;
	}
}

void supervisorUpdate(const sensorData_t *sensors, const setpoint_t* setpoint, const state_t* latestState, stabilizerStep_t stabilizerStep) {
	if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
		return;
	}

	SupervisorMem_t* this = &supervisorMem;
	const uint32_t currentTick = xTaskGetTickCount();

	const supervisorConditionBits_t conditions = updateAndPopulateConditions(this, sensors, setpoint, currentTick);
	const supervisorState_t newState = supervisorStateUpdate(this->state, conditions);
	if (this->state != newState) {
		const supervisorState_t previousState = this->state;
		this->state = newState;
		postTransitionActions(this, previousState, latestState, currentTick);
	}

	this->latestConditions = conditions;
	updateLogData(this, conditions);
	if (this->doinfodump) {
		this->doinfodump = 0;
		infoDump(this);
	}
}

EVENTTRIGGER(leveloutState, float, now);
EVENTTRIGGER(descentState, float, now);
EVENTTRIGGER(lockedState, float, now);
void supervisorOverrideSetpoint(setpoint_t* setpoint) {
	SupervisorMem_t* this = &supervisorMem;
	switch(this->state){
		case supervisorStateReadyToFly:
			// Fall through
		case supervisorStateLanded:
			// Fall through
		case supervisorStateFlying:
			// Do nothing
			break;
		case supervisorStateSafeDescent:
			eventTrigger_descentState_payload.now = 1.0f;
			eventTrigger(&eventTrigger_descentState);

			memcpy(setpoint, &this->crashInfo.recoveryInfo.descendSetpoint, sizeof(setpoint_t));
			/*
			   if (setpoint->mode.z == modeRaw && z > 0.3f){ 
			   setpoint->thrust = 0.0f;
			   } 
			   */
			this->crashInfo.recoveryInfo.descentUpdate(setpoint);
			static bool mentioned = false;
			if(!mentioned)
			{ 
				DEBUG_PRINT("descending at %ld!\n", xTaskGetTickCount());
				mentioned = true;
			}
			break;

		case supervisorStateWarningLevelOut:
			eventTrigger_leveloutState_payload.now = 1.0f;
			eventTrigger(&eventTrigger_leveloutState);
			// override setpoint by the setpoint calculated by supervisor
			memcpy(setpoint, &this->crashInfo.recoveryInfo.leveloutSetpoint, sizeof(setpoint_t));
			break;

		case supervisorStateLocked:
			eventTrigger_lockedState_payload.now = 1.0f;
			eventTrigger(&eventTrigger_lockedState);
			break;

		default:
			// Replace with null setpoint to stop motors
			memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
			break;
	}
}

bool supervisorAreMotorsAllowedToRun() {
	SupervisorMem_t* this = &supervisorMem;
	return (this->state == supervisorStateReadyToFly) ||
		(this->state == supervisorStateFlying) ||
		(this->state == supervisorStateWarningLevelOut) ||
		(this->state == supervisorStateLanded || this->state == supervisorStateSafeDescent);
}

void infoDump(const SupervisorMem_t* this) {
	DEBUG_PRINT("Supervisor info ---\n");
	DEBUG_PRINT("State: %s\n", supervisorGetStateName(this->state));
	DEBUG_PRINT("Conditions: (0x%lx)\n", this->latestConditions);
	for (supervisorConditions_t condition = 0; condition < supervisorCondition_NrOfConditions; condition++) {
		const supervisorConditionBits_t bit = 1 << condition;
		int bitValue = 0;
		if (this->latestConditions & bit) {
			bitValue = 1;
		}

		DEBUG_PRINT("  %s (0x%lx): %u\n", supervisorGetConditionName(condition), bit, bitValue);
	}
}



/**
 *  System loggable variables to check different system states.
 */
LOG_GROUP_START(sys)
	/**
	 * @brief Nonzero if system is ready to fly.
	 *
	 * Deprecated, will be removed after 2024-06-01. Use supervisor.info instead
	 */
LOG_ADD_CORE(LOG_UINT8, canfly, &supervisorMem.canFly)
	/**
	 * @brief Nonzero if the system thinks it is flying
	 *
	 * Deprecated, will be removed after 2024-06-01. Use supervisor.info instead
	 */
LOG_ADD_CORE(LOG_UINT8, isFlying, &supervisorMem.isFlying)
	/**
	 * @brief Nonzero if the system thinks it is tumbled/crashed
	 *
	 * Deprecated, will be removed after 2024-06-01. Use supervisor.info instead
	 */
	LOG_ADD_CORE(LOG_UINT8, isTumbled, &supervisorMem.isTumbled)
LOG_GROUP_STOP(sys)


PARAM_GROUP_START(stabilizer)
	/**
	 * @brief If set to nonzero will turn off motors
	 */
	PARAM_ADD_CORE(PARAM_UINT8, stop, &supervisorMem.paramEmergencyStop)
PARAM_GROUP_STOP(stabilizer)


PARAM_GROUP_START(system)

	/**
	 * @brief Set to nonzero to arm the system. A nonzero value enables the auto arm functionality
	 *
	 * Deprecated, will be removed after 2024-06-01. Use the CRTP `PlatformCommand` `armSystem` on the CRTP_PORT_PLATFORM port instead.
	 */
	PARAM_ADD_CORE(PARAM_INT8, arm, &supervisorMem.deprecatedArmParam)
PARAM_GROUP_STOP(system)


	/**
	 * The purpose of the supervisor is to monitor the system and its state. Depending on the situation, the supervisor
	 * can enable/disable functionality as well as take action to protect the system or humans close by.
	 */
LOG_GROUP_START(supervisor)
	/**
	 * @brief Bitfield containing information about the supervisor status
	 * Bit 0 = Can be armed - the system can be armed and will accept an arming command
	 * Bit 1 = is armed - the system is armed
	 * Bit 2 = auto arm - the system is configured to automatically arm
	 * Bit 3 = can fly - the Crazyflie is ready to fly
	 * Bit 4 = is flying - the Crazyflie is flying.
	 * Bit 5 = is tumbled - the Crazyflie is up side down.
	 * Bit 6 = is locked - the Crazyflie is in the locked state and must be restarted.
	 */
	LOG_ADD(LOG_UINT16, info, &supervisorMem.infoBitfield)
LOG_GROUP_STOP(supervisor)


	/**
	 * The purpose of the supervisor is to monitor the system and its state. Depending on the situation, the supervisor
	 * can enable/disable functionality as well as take action to protect the system or humans close by.
	 */
PARAM_GROUP_START(supervisor)
	/**
	 * @brief Set to nonzero to dump information about the current supervisor state to the console log
	 */
PARAM_ADD(PARAM_UINT8, infdmp, &supervisorMem.doinfodump)

	/**
	 * @brief Landing timeout duration (ms)
	 */
PARAM_ADD(PARAM_UINT16 | PARAM_PERSISTENT, landedTimeout, &landingTimeoutDuration)

PARAM_GROUP_STOP(supervisor)
