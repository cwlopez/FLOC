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
	float			dX[FLOCK_SIZE];
	float			dY[FLOCK_SIZE];
	float			dZ[FLOCK_SIZE];
	float			dXL;
	float			dYL;
	float			dZL;
	float			d2L;
	float			dvx;
	float			dvy;
	float			dvz;
	uint16_t		hdgL;
};

//Specific to Huey, Dewey, Louie Formation: To easily tell who is part of the formation
struct RollCall {
	bool Huey;
	bool Dewey;
	bool Louie;
};

#define MtoFT(x) (x*3.28084)			//	*3.28084
#define De7ToFT(x) (x*0.0365214)		// Change Lat/Lon degrees * 10^7 to feet
#define De7ToM(x) (x*.01113195)			// Change Lat/Lon degrees *10^7 to M

//Log Tags (should not conflict with other APM log tags)
#define LOG_FLOCK_STATUS_MSG	0x0B
#define LOG_PF_FIELD_MSG		0x0C
#define LOG_VWP_MSG				0x0D
#define LOG_REL_MSG				0x0E
#define LOG_ERROR_ASSIST_MSG	0x0F
#endif 

