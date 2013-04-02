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
	int16_t			dVXL;
	int16_t			dVYL;
	int16_t			dVZL;
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

//FCOM Message Defines
enum ff_message {
	FF_HEARTBEAT,
	FF_LOCATION,
	FF_FLOCK_STATUS,
	FF_PF_FIELD,
	FF_VWP,
	FF_REL_STATE,
	FF_ERROR_ASSIST,
	FF_MSG_RETRY_DEFERRED // this must be last
};

enum XBee_Addresses {
	TO_GCS,
	TO_ALL
};
# define XBEE_API_OVERHEAD 15
#endif 

