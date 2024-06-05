#include "supervisor_board_disconnected.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "led.h"
#include <sys/types.h>
#include "kalman_core.h"
#include "stabilizer_types.h"
#include "freeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "config.h"
#include "debug.h"
#include "filter.h"
#include "system.h"
#include "static_mem.h"
#include "linearfunction.h"

#include "deck.h"
#include "ow.h"

#define DEBUG_MODULE "BOARD SUP"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c\n"
#define BYTE_TO_BINARY(byte)         \
	((byte) & 0x80 ? '1' : '0'),     \
	((byte) & 0x40 ? '1' : '0'), \
	((byte) & 0x20 ? '1' : '0'), \
	((byte) & 0x10 ? '1' : '0'), \
	((byte) & 0x08 ? '1' : '0'), \
	((byte) & 0x04 ? '1' : '0'), \
	((byte) & 0x02 ? '1' : '0'), \
	((byte) & 0x01 ? '1' : '0')

static uint32_t LEVEL_OUT_TIMEOUT = M2T(1000);

static bool isInit = false;
static int nDecksAtStartup = 0;
// bitmap for cusum test on kalmanfilter
static u_int16_t statesMeasured = 0;

// bitmap showcasing remaining available states after board error
static uint16_t availableStates = 0;

static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

// data to share with state machine
static boardStatus_t SharedBoardStatus;

static uint32_t lastOwCheckTick = 0;
static uint32_t lastConnectedCheckTick = 0;
static uint32_t lastKalmanUpdaRateCheck = 0;

#define VEL_LPF_CUTOFF_FREQ 0.3f
static lpf2pData velocityLpf;

void boardSupervisorTask(void *);

// STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(boardSupervisorTask, BOARD_SUP_TASK_STACKSIZE);
STATIC_MEM_TASK_ALLOC(boardSupervisorTask, BOARD_SUP_TASK_STACKSIZE);

#define SCALAR_UPDATE_QUEUE_SIZE (100)
static xQueueHandle scalarUpdateQueue;
STATIC_MEM_QUEUE_ALLOC(scalarUpdateQueue, SCALAR_UPDATE_QUEUE_SIZE, sizeof(faultMeasurement_t));

#define ERROR_QUEUE_SIZE (2)
static xQueueHandle errorQueue;
STATIC_MEM_QUEUE_ALLOC(errorQueue, ERROR_QUEUE_SIZE, sizeof(recoveryStrategy_t));


#include "kalman_core.h"
static uint32_t kalmanLastUpdate[KC_STATE_DIM];
#include "deck_core.h"
static stateIdentifier deckIdentifierArray[IDENTIFIER_ARRAY_SIZE];


static int currIdentifierIndex = 0;

static void print_bitmap(uint16_t);

// just use the measurement type instead of stateSignature
static bool getIdentifierByStateSignature(int* index, stateIdentifier* vec, int maxLen, uint16_t inputSignature)
{ 
	bool found = false;
	for(int i = 0; i < currIdentifierIndex; i++)
	{ 
		/*
		 * DEBUG_PRINT("vec[%d].info.stateSignature = ", i);
		 print_bitmap(vec[i].info.stateSignature);
		 DEBUG_PRINT("inputSignature = ");
		 print_bitmap(inputSignature);
		 */
		int match = vec[i].info.stateSignature == inputSignature;
		if(match)
		{ 
			*index = i;
			found = true;
			break;
		} 
	} 
	return found;
} 
static void print_bitmap(uint16_t bitmap)
{
	// Iterate through each bit of the uint16_t bitmap
	for (int i = 15; i >= 0; i--)
	{
		// Check the i-th bit
		uint16_t mask = 1 << i;
		uint16_t bit = (bitmap & mask) >> i;
		DEBUG_PRINT("%d", bit);
	}
	DEBUG_PRINT("\n");
}
void boardSupervisorInit()
{
	memset(&deckIdentifierArray, 0, sizeof(stateIdentifier));
	memset(&kalmanLastUpdate, 0, sizeof(kalmanLastUpdate));
	nDecksAtStartup = deckCount();

	for (int i = 0; i < nDecksAtStartup; i++)
	{
		DeckInfo *deck = deckInfo(i);

		DEBUG_PRINT("Board: %s\n", deck->driver->name);

		if (deck->driver->connected != NULL)
		{
			deck->driver->connected(&statesMeasured);
		}

		DEBUG_PRINT("len = %d\n", deck->driver->len);
		if (deck->driver->len > 0)
		{
			for (int i = 0; i < deck->driver->len; i++)
			{
				if (currIdentifierIndex < IDENTIFIER_ARRAY_SIZE)
				{
					memcpy(&deckIdentifierArray[currIdentifierIndex].info, &deck->driver->deckParams[i], sizeof(deckParams_t));

					DEBUG_PRINT("Startup signature found: ");
					print_bitmap(deck->driver->deckParams[i].stateSignature);
					currIdentifierIndex++;
				}
				else
				{
					DEBUG_PRINT("currIdentifierIndex out of bounds, try increasing buffer\n");
				}
			}
		}
	}
	for(int i = 0; i < currIdentifierIndex; i++)
	{ 
		DEBUG_PRINT("signature = ");
		print_bitmap(deckIdentifierArray[i].info.stateSignature);
		DEBUG_PRINT("runTestViolationsPermitted: %d. runTestWindow: %ld\n", deckIdentifierArray[i].info.runTest.runTestViolationsPermitted, deckIdentifierArray[i].info.runTest.runTestWindow);
	} 
	memcpy(&availableStates, &statesMeasured, sizeof(uint16_t));

	lpf2pInit(&velocityLpf, RATE_100_HZ, VEL_LPF_CUTOFF_FREQ);


	dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

	memset(&SharedBoardStatus, 0, sizeof(boardStatus_t));

	if (nDecksAtStartup > 0)
	{
		STATIC_MEM_TASK_CREATE(boardSupervisorTask, boardSupervisorTask, BOARD_SUP_TASK_NAME, NULL, BOARD_SUP_TASK_NAME_PRI);
		scalarUpdateQueue = STATIC_MEM_QUEUE_CREATE(scalarUpdateQueue);
		errorQueue = STATIC_MEM_QUEUE_CREATE(errorQueue);
		isInit = true;
	}
}


static bool scalarUpdateDeque(faultMeasurement_t *update)
{
	return pdTRUE == xQueueReceive(scalarUpdateQueue, update, 50);
}

bool isBoardErrorCheck(recoveryStrategy_t* info)
{
	if(!errorQueue)
	{ 
		return false;
	} 

	return pdTRUE == xQueueReceive(errorQueue, info, 0);
}
static void recoveryEnque(recoveryStrategy_t* info)
{
	if (!errorQueue)
	{
		return;
	}

	xQueueSend(errorQueue, info, 0);
}
void enqueueVelocity(const kalmanCoreData_t* coreData)
{ 
	float vx = coreData->S[KC_STATE_PX];
	float vy = coreData->S[KC_STATE_PY];
	float vz = coreData->S[KC_STATE_PZ];


	float z = coreData->S[KC_STATE_Z];

	float vHypot = powf(vx, 2.0f) + powf(vy, 2.0f) + powf(vz, 2.0f);

	faultMeasurement_t m;

	//m.measurementType.vHypot = sqrtf(vHypot);
	m.type = MeasurementTypeVelocity;

	m.data.velUpdate.vHypot = sqrtf(vHypot);
	m.data.velUpdate.altitude = z;
	scalarUpdateEnque(&m);
} 
void scalarUpdateEnque(faultMeasurement_t *update)
{
	if (!scalarUpdateQueue)
	{
		return;
	}

	xQueueSend(scalarUpdateQueue, update, 0);
}

bool boardSupervisorTest()
{
	return nDecksAtStartup > 0 ? isInit : true;
}


static void boardSupervisorDetermineAction(boardStatus_t *bs)
{
	// action to be executed is determined based on level of severity as defined in the enum, action with highest level of severity gets executed
	uint8_t actionBit = ACTION_NO_ACTION;
	for (uint8_t actionNum = NUM_OF_ERROR_ACTIONS - 1; actionNum > ACTION_NO_ACTION; actionNum--)
	{
		actionBit = (1 << actionNum) & bs->actionBitMap;
		if (actionBit)
		{
			// action to execute found
			break;
		}
		actionBit = ACTION_NO_ACTION;
	}

	bs->recoveryInfo.actionToExecute = actionBit;
}

supervisorConditionBits_t getTransitionadssad(const uint32_t currentTick)
{

	uint32_t timeElapsed = currentTick - SharedBoardStatus.recoveryInfo.t0;

	supervisorConditionBits_t bit = SUPERVISOR_CB_NONE;

	switch (SharedBoardStatus.recoveryInfo.actionToExecute)
	{
		case ACTIONBIT_ALERT:
			DEBUG_PRINT("ALERT\n");
			break;
		case ACTIONBIT_NO_ACTION:
			bit = SUPERVISOR_CB_NONE;
			break;
		case ACTIONBIT_LEVELOUT:
			bit = SUPERVISOR_CB_BOARD_ERROR;
			break;
		case ACTIONBIT_SAFE_DESCENT:
			bit = timeElapsed > LEVEL_OUT_TIMEOUT ? SUPERVISOR_CB_SAFE_DESCENT : SUPERVISOR_CB_BOARD_ERROR;
			break;
		case ACTIONBIT_FREEFALL:
			bit = SUPERVISOR_CB_EMERGENCY_STOP;
			break;
		default:
			bit = SUPERVISOR_CB_EMERGENCY_STOP;
			break;
	}

	return bit;
}


static bool isActionToExecute(boardStatus_t* bs)
{ 
	uint8_t action = bs->actionBitMap;
	// make sure we actually want to do anything relevant
	if (action != ACTIONBIT_SAFE_DESCENT && action != ACTIONBIT_LEVELOUT && action != ACTIONBIT_FREEFALL && action != ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN)
	{
		return false;
	}

	return true;
} 

static void determineStrategy(boardStatus_t *bs)
{
	//uint8_t action = bs->recoveryInfo.actionToExecute;

	setpoint_t leveloutSetpoint;
	setpoint_t descendSetpoint;

	memset(&leveloutSetpoint, 0, sizeof(setpoint_t));
	memset(&descendSetpoint, 0, sizeof(setpoint_t));

	/* levelout strategy */
	// states diverged, we cannot tell which sensor failed, but can be somewhat sure it is not the IMU
	if (bs->statusBitMap == ERROR_BIT_STD_VIOLATION)
	{
		DEBUG_PRINT("ERROR_BIT_STD_VIOLATION\n");
		leveloutSetpoint.mode.x = modeDisable;
		leveloutSetpoint.mode.y = modeDisable;
		// leave z to whatever it was
		leveloutSetpoint.mode.z = modeVelocity;
		leveloutSetpoint.velocity.z = 0;

		leveloutSetpoint.mode.roll = modeAbs;
		leveloutSetpoint.mode.pitch = modeAbs;
		leveloutSetpoint.attitude.roll = 0;
		leveloutSetpoint.attitude.pitch = 0;
	}
	else
	{
		DEBUG_PRINT("Velocity levelout\n");
		// else either OW error, connected or outage error, allow to control on velocity
		leveloutSetpoint.mode.x = modeVelocity;
		leveloutSetpoint.mode.y = modeVelocity;
		leveloutSetpoint.mode.z = modeVelocity;

		leveloutSetpoint.velocity.x = 0;
		leveloutSetpoint.velocity.y = 0;
		leveloutSetpoint.velocity.z = 0;

		/*
		leveloutSetpoint.mode.x = modeAcc;
		leveloutSetpoint.mode.y = modeAcc;
		leveloutSetpoint.mode.z = modeAcc;
		*/

	}

	/* descend strategy */
	//descendSetpoint.mode.z = modeAbs;
	descendSetpoint.mode.z = modeRaw;
	//descendSetpoint.position.z = 0.0f;
	bs->recoveryInfo.descentUpdate = noAction;
	//bs->recoveryInfo.descentUpdate = accelerationDescent;

	if(bs->statusBitMap == ERROR_BIT_STD_VIOLATION){ 

		DEBUG_PRINT("Copying descendSetpoint\n");
		memcpy(&descendSetpoint, &leveloutSetpoint, sizeof(setpoint_t));
		descendSetpoint.mode.z = modeRaw;
	} 
	else if(bs->statusBitMap == ERROR_BIT_KALMAN_UPDATE_FREQ_ERROR){ 
		// hold pitch roll during descend
		DEBUG_PRINT("roll pitch \n");

		descendSetpoint.mode.x = modeDisable;
		descendSetpoint.mode.y = modeDisable;

		descendSetpoint.mode.roll = modeAbs;
		descendSetpoint.mode.pitch = modeAbs;
		descendSetpoint.attitude.roll = 0;
		descendSetpoint.attitude.pitch = 0;

		/*
		descendSetpoint.mode.x = modeAcc;
		descendSetpoint.mode.y = modeAcc;
		descendSetpoint.mode.z = modeAcc;


		descendSetpoint.acceleration.x = 0;
		descendSetpoint.acceleration.y = 0;
		*/
		/*
		descendSetpoint.mode.x = modeVelocity;
		descendSetpoint.mode.y = modeVelocity;
		DEBUG_PRINT("Velocity descent\n");
		descendSetpoint.velocity.x = 0;
		descendSetpoint.velocity.y = 0;
		*/

	} 
	else
	{
		// else either ow or connected error allow to control on vel
		descendSetpoint.mode.x = modeVelocity;
		descendSetpoint.mode.y = modeVelocity;
		DEBUG_PRINT("Velocity descent\n");
		descendSetpoint.velocity.x = 0;
		descendSetpoint.velocity.y = 0;
	}
	memcpy(&bs->recoveryInfo.leveloutSetpoint, &leveloutSetpoint, sizeof(setpoint_t));
	memcpy(&bs->recoveryInfo.descendSetpoint, &descendSetpoint, sizeof(setpoint_t));
}


static bool isError(boardStatus_t *bs)
{
	return bs->statusBitMap != NO_ERROR;
}

#define errorHoldMs M2T(100)
#define MAX_OUTAGE_PERIOD M2T(1100)

uint16_t findAvailableStates(stateIdentifier* ids)
{ 
	uint16_t availableStates = 0x00;
	for (int i = 0; i < currIdentifierIndex; i++)
	{
		if(!ids[i].outage)
		{ 
			availableStates |= ids[i].info.stateSignature;
		} 
		else{ 
			DEBUG_PRINT("Rates of on: ");
			print_bitmap(ids[i].info.stateSignature);
		} 
		ids[i].outage = false; // reset
	}
	return availableStates;
} 
static bool updateRatesNominal(stateIdentifier* ids, uint16_t *stateFreqBitmap, uint32_t nowTick)
{
	bool allMeasurementratesNominal = true;
	//static uint32_t t0 = 0;
	for (int i = 0; i < currIdentifierIndex; i++)
	{
		if ((nowTick - deckIdentifierArray[i].lastUpdate) > MAX_OUTAGE_PERIOD)
		{
			ids[i].outage = true;
			//t0 = t0 == 0 ? nowTick : t0;
			//DEBUG_PRINT("scalar %d lost\n", i);
			allMeasurementratesNominal = false;
		}
	}

	/*
	// give errorHoldMs to let other measurement outages propagate
	if(t0 != 0 && (nowTick - t0) > errorHoldMs)
	{ 
		// find all states available here
		allMeasurementratesNominal = false;
		*stateFreqBitmap = findAvailableStates(ids);
		t0 = 0; // reset
	} 
	//allMeasurementratesNominal = true;
	*/

	return allMeasurementratesNominal;
}

void updateMeasurementTimestamp(faultMeasurement_t *m, uint32_t nowTick)
{
	// måste kolla på state signature istället 
	for (int i = 0; i < KC_STATE_DIM; i++)
	{
		uint16_t stateMeasured = (1 << i) & m->data.scalarUpdate.stateBitmap;

		if (stateMeasured)
		{
			kalmanLastUpdate[i] = nowTick;
		}
	}
	static uint32_t t0 = 0;

	if ((xTaskGetTickCount() - t0) > 2000)
	{
		t0 = xTaskGetTickCount();
		for (int i = 0; i < KC_STATE_DIM; i++)
		{
			// DEBUG_PRINT("timeStampUpdate[%d] = %.6f\n", i, (double)kalmanLastUpdate[i]);
		}
	}
}
static void performBoardConnectionTest(boardStatus_t* bs, uint16_t* errorenousState)
{ 
	bool boardConnectedFailed = false;
	for (int i = 0; i < nDecksAtStartup; i++)
	{
		const DeckDriver *deckDriver = deckInfo(i)->driver;
		if (deckDriver->connected == NULL) // connected logic not implemented for deck
		{
			continue;
		}

		if (!deckDriver->connected(errorenousState))
		{
			boardConnectedFailed = true;
			DEBUG_PRINT("Connection test for board %s failed \n", deckDriver->name);
			bs->statusBitMap |= ERROR_BIT_BUS_ERROR;
			//bs->actionBitMap |= deckDriver->boardErrorAction;
			//bs->actionBitMap |= ACTIONBIT_SAFE_DESCENT;
			//bs->actionBitMap |= ACTIONBIT_FREEFALL;
			bs->actionBitMap |= ACTIONBIT_LEVELOUT;
			//bs->actionBitMap |= ACTIONBIT_NO_ACTION;
			//bs->actionBitMap |= ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN;
		}

	} 
	if(!boardConnectedFailed){ 
			bs->statusBitMap &= ~ERROR_BIT_BUS_ERROR;
			bs->actionBitMap &= ~ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN;
	} 
}

void performOWTest(boardStatus_t* bs)
{ 
	bool boardOWFailed = false;
	for (int i = 0; i < nDecksAtStartup; i++)
	{
		if (!fastInfo(i))
		{
			boardOWFailed = true;
			const DeckDriver *deckDriver = deckInfo(i)->driver;
			DEBUG_PRINT("OW for board %s disconnected \n", deckDriver->name);
			bs->statusBitMap |= ERROR_BIT_OW_ERROR;
			bs->actionBitMap |= ACTIONBIT_SAFE_DESCENT;
			//bs.actionBitMap |= deckDriver->boardErrorAction;
		}
	}
	if(!boardOWFailed){ 
			bs->statusBitMap &= ~ERROR_BIT_OW_ERROR;
			bs->actionBitMap &= ~ACTIONBIT_SAFE_DESCENT;
	} 
} 

// should include a std check aswell
bool allKalmanStatesReceived(uint32_t nowTick)
{ 
	for (int i = 0; i < currIdentifierIndex; i++)
	{
		if((nowTick - deckIdentifierArray[i].lastUpdate) > 300)
		{ 
			return false;

		}
	} 
	return true;
}


supervisorConditionBits_t getTransition(const safeDescentInfo_t* failsafeInfo, uint32_t currentTick)
{ 
	uint32_t leveloutTimeout = (uint32_t)(failsafeInfo->leveloutTime*1000);
	uint32_t initialErrorTimestamp = failsafeInfo->recoveryInfo.t0;

	uint32_t timeElapsed = currentTick - initialErrorTimestamp;
	supervisorConditionBits_t bit = SUPERVISOR_CB_NONE;
	switch (failsafeInfo->recoveryInfo.actionToExecute)
	{
		case ACTIONBIT_LEVELOUT:
			DEBUG_PRINT("Levelout\n");
			bit = SUPERVISOR_CB_BOARD_ERROR;
			break;
		case ACTIONBIT_SAFE_DESCENT:
		case ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN:
			bit = timeElapsed > leveloutTimeout ? SUPERVISOR_CB_SAFE_DESCENT : SUPERVISOR_CB_BOARD_ERROR;
			break;
		case ACTIONBIT_FREEFALL:
			bit = SUPERVISOR_CB_EMERGENCY_STOP;
			break;
	} 
	return bit;
} 

static bool expectedkalmanStatesReceived = true;

bool startupKalmanStatesNominal()
{ 
	if(!isInit)
	{ 
		return true;
	} 
	return expectedkalmanStatesReceived;
} 


#define MAX_STD_DEVIATION_PERIOD 4000
#include "eventtrigger.h"
#include "cfassert.h"

/*
   EVENTTRIGGER(NxCU, float, sumPos, float, NIS);
   EVENTTRIGGER(NyCU, float, sumPos, float, NIS);
   EVENTTRIGGER(tofCU, float, sumPos, float, NIS);


   EVENTTRIGGER(NxStd, float, std, float, err, float, s);
   EVENTTRIGGER(NyStd, float, std, float, err, float, s);
   EVENTTRIGGER(tofStd, float, std, float, err, float, s);
   */
EVENTTRIGGER(cusumSweep, float, sumPos, float, NIS, float, v, float, threshold);
EVENTTRIGGER(cusumHeading, float, sumPos, float, NIS, float, v, float, threshold);

EVENTTRIGGER(cusumViolation, float, index, float, runTest);
EVENTTRIGGER(velIncoming, float, vfilt, float, vunfilt);


EVENTTRIGGER(sweepStd, float, std, float, err, float, s);
EVENTTRIGGER(headingStd, float, std, float, err, float, s);

void resetAll(stateIdentifier* ids)
{ 
	for(int i = 0; i < currIdentifierIndex; i++)
	{ 
		runningStatsReset(&ids[i].stats);
	} 
	DEBUG_PRINT("All stats reseted\n");

} 
 

static bool allRunTestsNominal(stateIdentifier* deckIdentifierArray)
{ 
	for(int i = 0; i < currIdentifierIndex; i++)
	{ 
		if(deckIdentifierArray[i].info.runTest.runTestPreviouslyTriggered)
		{ 
			return false;
		} 
	} 
	return true;
} 

void boardSupervisorTask(void *param)
{
	DEBUG_PRINT("BoardSupervisor Init\n");

	uint32_t nowTick = T2M(xTaskGetTickCount());
	uint16_t errorenousState = 0;
	boardStatus_t bs;
	memset(&bs, 0, sizeof(boardStatus_t));

	systemWaitStart();

	faultMeasurement_t m;
	memset(&m, 0, sizeof(faultMeasurement_t));
	DEBUG_PRINT("startup statesMeasured: ");
	print_bitmap(statesMeasured);

	float vfiltered = 0.0f;

	int index = 0;
	bool transitionedBack = true;
	resetAll(deckIdentifierArray);

	bool shouldCusum = false;


	for(;;)
	{ 
		xQueueReceive(scalarUpdateQueue, &m, portMAX_DELAY);
		nowTick = xTaskGetTickCount();

		//updateMeasurementTimestamp(&m, nowTick);
		if (getIdentifierByStateSignature(&index, deckIdentifierArray, IDENTIFIER_ARRAY_SIZE, m.data.scalarUpdate.stateBitmap))
		{
			deckIdentifierArray[index].lastUpdate = nowTick;
		}
		if(allKalmanStatesReceived(nowTick))
		{ 
			break;
		} 
	} 
	DEBUG_PRINT("moving on\n");

	for (;;)
	{
		nowTick = T2M(xTaskGetTickCount());
		if (scalarUpdateDeque(&m))
		{
			// decode which board sent the message and get measurement cusum limits and data  
			if (getIdentifierByStateSignature(&index, deckIdentifierArray, IDENTIFIER_ARRAY_SIZE, m.data.scalarUpdate.stateBitmap))
			{
				float NIS = powf(m.data.scalarUpdate.error, 2.0f) / m.data.scalarUpdate.s;
				if(shouldCusum)
				{ 
					cusumUpdate(&deckIdentifierArray[index].cusumData, &deckIdentifierArray[index].info.params, NIS);
					linearFuncParams_t* linearFuncParams = &deckIdentifierArray[index].info.linearFunc;
					float threshold = linearFunctionApply(linearFuncParams, vfiltered);

					if(cusumThresholdExceeded(&deckIdentifierArray[index].cusumData, threshold))
					{

						DEBUG_PRINT("Violation\n");
						//runTest_t* rt = &deckIdentifierArray[index].info.runTest;
						//rt->firstViolation +=0;
						uint32_t lastviolation_elapsed = nowTick - deckIdentifierArray[index].info.runTest.firstViolation;
						int nViolations = deckIdentifierArray[index].info.runTest.violations + 1;
						int runTestViolationsPermitted = deckIdentifierArray[index].info.runTest.runTestViolationsPermitted;
						uint32_t window = deckIdentifierArray[index].info.runTest.runTestWindow;

						if(lastviolation_elapsed < window && nViolations > runTestViolationsPermitted)
						{ 
							DEBUG_PRINT("Action\n");
							bs.statusBitMap |= ERROR_BIT_STD_VIOLATION;
							//bs.actionBitMap |= ACTIONBIT_SAFE_DESCENT;
							//bs.actionBitMap |= ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN;
							bs.actionBitMap |= ACTIONBIT_NO_ACTION;
							deckIdentifierArray[index].info.runTest.violations = runTestViolationsPermitted;
							deckIdentifierArray[index].info.runTest.runTestPreviouslyTriggered = true;
						} 
						else if(lastviolation_elapsed < window){ 
							deckIdentifierArray[index].info.runTest.violations++;
							DEBUG_PRINT("adding violations\n");
						} 
						else {
							DEBUG_PRINT("violation = 1\n");
							deckIdentifierArray[index].info.runTest.violations = 1;
							deckIdentifierArray[index].info.runTest.firstViolation = nowTick;
							deckIdentifierArray[index].info.runTest.runTestPreviouslyTriggered = false;
						}
						cusumReset(&deckIdentifierArray[index].cusumData, NIS);
					} 
					else if(cusumRunTestFaultOver(&deckIdentifierArray[index].info.runTest, nowTick))
					{ 
						deckIdentifierArray[index].info.runTest.runTestPreviouslyTriggered = false;
					} 
				}

				if(allRunTestsNominal(deckIdentifierArray)){ 
					bs.statusBitMap &= ~ERROR_BIT_STD_VIOLATION;
					bs.actionBitMap &= ~ACTIONBIT_SAFE_DESCENT;
				} 
				else{ 
					//DEBUG_PRINT("should not cusum!\n");
				} 
				// refresh measurement last update tick
				deckIdentifierArray[index].lastUpdate = nowTick;
			}
			else if(m.type == MeasurementTypeVelocity){ 
				vfiltered = lpf2pApply(&velocityLpf, m.data.velUpdate.vHypot);
				// for some reason the metric goes bananas below approx 0.6m
				shouldCusum = m.data.velUpdate.altitude > 1.0f; // m
			} 
			else
			{
				//DEBUG_PRINT("Could not identify measurement with signature: ");
				//print_bitmap(m.data.scalarUpdate.stateBitmap);
			}
		}
		if(PERIOD_DO_EXECUTE(KALMAN_UPDATE_TEST_PERIOD, nowTick, lastKalmanUpdaRateCheck))
		{ 
			lastKalmanUpdaRateCheck = nowTick;
			if(!updateRatesNominal(deckIdentifierArray, &availableStates, nowTick))
			{ 
				bs.statusBitMap |= ERROR_BIT_KALMAN_UPDATE_FREQ_ERROR;
				//bs.actionBitMap |= ACTIONBIT_LEVELOUT_DESCENT_OR_RETURN;
				bs.actionBitMap |= ACTIONBIT_SAFE_DESCENT;
			} 
			else{ 
				bs.statusBitMap &= ~ERROR_BIT_KALMAN_UPDATE_FREQ_ERROR;
				bs.actionBitMap &= ~ACTIONBIT_SAFE_DESCENT;
			} 
		} 
		if(PERIOD_DO_EXECUTE(CONNECTED_TEST_PERIOD, nowTick, lastConnectedCheckTick))
		{
			if(false){ 
				performBoardConnectionTest(&bs, &errorenousState);

			} 
			lastConnectedCheckTick = nowTick;
		}
		if(PERIOD_DO_EXECUTE(OW_TEST_PERIOD, nowTick, lastOwCheckTick))
		{
			performOWTest(&bs);
			lastOwCheckTick = nowTick;
		}

		if(bs.actionBitMap & ACTIONBIT_ALERT)
		{
			ledShowFaultPattern();
		}

		if (isError(&bs) && 0 == bs.recoveryInfo.t0 && isActionToExecute(&bs))
		{
			DEBUG_PRINT("Err code: ");
			print_bitmap(bs.statusBitMap);
			bs.recoveryInfo.t0 = nowTick;
			// inidicator to supervisor that transition needed from flying
			bs.recoveryInfo.errorInit = true;
			boardSupervisorDetermineAction(&bs);
			determineStrategy(&bs);
			DEBUG_PRINT("Sending fault detection action\n");
			recoveryEnque(&bs.recoveryInfo);
			bs.statusBitMap = NO_ERROR;
			transitionedBack = false;
		}
		else if((nowTick - bs.recoveryInfo.t0) > 800 && bs.statusBitMap == NO_ERROR && !transitionedBack){ 

			DEBUG_PRINT("No error, sending transition back\n");
			transitionedBack = true;
			bs.recoveryInfo.t0 = 0;
			bs.statusBitMap = NO_ERROR;
			bs.actionBitMap = 0;

			// return to flying state inidicator
			bs.recoveryInfo.errorInit = false;
			// send status to transition back
			recoveryEnque(&bs.recoveryInfo);
			bs.recoveryInfo.actionToExecute = ACTION_NO_ACTION;
		} 
	} 
}
