#ifndef SUPERVISOR_BOARD_H
#define SUPERVISOR_BOARD_H
#include "supervisor.h"
#include "kalman_core.h"
#include "stdbool.h"
#include <stdint.h>
#include "stabilizer_types.h"
#include "supervisor_state_machine.h"
#include "cusum.h"
#include "runningstats.h"
#include <assert.h>
#include "controller_emergency.h"
#include "freeRTOSConfig.h"
#include "kalman_core.h"

#define IDENTIFIER_ARRAY_SIZE (2*DECK_MAX_COUNT)

#define CONNECTED_TEST_PERIOD 500
#define OW_TEST_PERIOD 250
#define KALMAN_UPDATE_TEST_PERIOD 20


#define STATES_X  (1 << KC_STATE_X)
#define STATES_Y  (1 << KC_STATE_Y)
#define STATES_Z  (1 << KC_STATE_Z)

#define STATES_VX (1 << KC_STATE_PX)
#define STATES_VY (1 << KC_STATE_PY)
#define STATES_VZ (1 << KC_STATE_PZ)


#define STATES_D0 (1 << KC_STATE_D0)
#define STATES_D1 (1 << KC_STATE_D1)
#define STATES_D2 (1 << KC_STATE_D2)

#define PERIOD_DO_EXECUTE(RATE_PERIOD, TICK, LAST_EXECUTE) ((TICK - LAST_EXECUTE) > RATE_PERIOD)


typedef enum { 
		NO_ERROR = 0,
		OW_ERROR,
		BUS_ERROR,
		STD_ERROR,
		STATE_UDATE_FREQ_ERROR,
		NUM_ERRORS,
} boardStatus_e;

#define ERROR_BIT_NO_ERROR 				   (1 << 0)
#define ERROR_BIT_OW_ERROR 				   (1 << OW_ERROR)
#define ERROR_BIT_BUS_ERROR 			   (1 << BUS_ERROR)
#define ERROR_BIT_STD_VIOLATION        	   (1 << STD_ERROR)
#define ERROR_BIT_KALMAN_UPDATE_FREQ_ERROR (1 << STATE_UDATE_FREQ_ERROR)
//
// in order of severity, if multiple errors indicated by task, boardErrorAction with highest level of severity gets executed
typedef enum{ 
	ACTION_NO_ACTION,
	ACTION_ALERT,
	ACTION_LEVELOUT,
	ACTION_LEVELOUT_OR_DESCENT,
	ACTION_SAFE_DESCENT,
	ACTION_FREEFALL,
	NUM_OF_ERROR_ACTIONS,
} boardErrorAction_e;


#define ACTIONBIT_NO_ACTION 		(0)
#define ACTIONBIT_ALERT 			(1 << ACTION_ALERT)
#define ACTIONBIT_LEVELOUT 			(1 << ACTION_LEVELOUT)	
#define ACTIONBIT_SAFE_DESCENT 		(1 << ACTION_SAFE_DESCENT)
//#define ACTIONBIT_BD_SAFE_DESCENT (1 << ACTION_BOARD_DISCONNECT_SAFE_DESCENT)
#define ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN (1 << ACTION_LEVELOUT_OR_DESCENT)	
#define ACTIONBIT_FREEFALL 			(1 << ACTION_FREEFALL)	

// cant fit all errors in uint8_t bitMap otherwise
static_assert(NUM_OF_ERROR_ACTIONS < 8);



typedef struct{ 
	uint32_t t0;
	uint8_t actionToExecute;
	bool errorInit;
	DescentStrategy descentUpdate;
	setpoint_t leveloutSetpoint;
	setpoint_t descendSetpoint;
} recoveryStrategy_t;

typedef struct { 
	uint8_t statusBitMap;
	uint8_t actionBitMap;
	recoveryStrategy_t recoveryInfo;
} boardStatus_t;

typedef struct { 
	float descentTimeRequired;
	uint32_t initialDescentTime;
	bool active;
	float leveloutTime;
	state_t stateSnapshot;
	//LeveloutStrategy leveloutStrategy;

	recoveryStrategy_t recoveryInfo;

} safeDescentInfo_t;


typedef enum { 
		MeasurementTypeVelocity,
		MeasurementTypeScalarUpdate,
} faultMeasurementType;

typedef struct { 
	float error;
	float s;
	uint16_t stateBitmap;
} scalarUpdateMeasurement_t;

typedef struct { 
	float vHypot;
	float altitude;
} velocityMeasurement_t;

typedef struct { 
	deckParams_t info;
	cusumData_t cusumData;
	runningStats_t stats;
	uint32_t lastUpdate;
	bool cusumActive;
	bool outage;
} stateIdentifier;

typedef struct { 
	faultMeasurementType type;
	union { 
		scalarUpdateMeasurement_t scalarUpdate;
		velocityMeasurement_t velUpdate;
	} data; 
} faultMeasurement_t;


bool boardErrorCheck();
void setLeveloutTimeout(float);
void boardSupervisorInit();
void boardStatusGet(boardStatus_t*);
void boardSupervisorTask(void*);
void scalarUpdateEnque(faultMeasurement_t*);
bool startupKalmanStatesNominal();
bool isBoardErrorCheck(recoveryStrategy_t*);
void enqueueVelocity(const kalmanCoreData_t*);
supervisorConditionBits_t getTransition(const safeDescentInfo_t*, uint32_t);
#endif
