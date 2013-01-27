// formation_common.h

#ifndef _FORMATION_COMMON_h
#define _FORMATION_COMMON_h


//includes parameters
#include <FastSerial.h>
#include "config.h"
//including AP_Math.h should include: AP_Common, Arduino.h, stdint.h, c++.h, and AP_Param.h
#include "AP_Math.h"
#include "defines.h"

//Structure to make it easier to group relavent information about other flock memebers
struct Relative {
	uint8_t			Num_members;
	uint8_t			Member_ids[FLOCK_SIZE];
	int32_t			dX[FLOCK_SIZE];
	int32_t			dY[FLOCK_SIZE];
	int32_t			dZ[FLOCK_SIZE];
	int32_t			dXL;
	int32_t			dYL;
	int32_t			dZL;
	int32_t			d2L;
	int32_t			dvx;
	int32_t			dvy;
	int32_t			dvz;
};

//Specific to Huey, Dewey, Louie Formation: To easily tell who is part of the formation
struct RollCall {
	bool Huey;
	bool Dewey;
	bool Louie;
};

#endif

