#include "stabilizer_types.h"

typedef void (*LeveloutStrategy)(setpoint_t*);
typedef void (*DescentStrategy)(setpoint_t*);


// Levelout strategies
void defaultLevelout(setpoint_t*);
void setpointOverrideLevelout(const setpoint_t*, setpoint_t*);


// descent strategies
void accelerationDescent(setpoint_t*);
void noAction();

