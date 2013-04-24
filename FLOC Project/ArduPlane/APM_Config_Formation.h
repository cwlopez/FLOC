// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//This is to set values specific to the formation flight mode

//Select which airframe you are using
# define I_AM_HUEY
//# define I_AM_DEWEY
//# define I_AM_LOUIE

///////////////////////////////////////////////////////////////////////////////////////////
//FLOCK SIZE/MEMBER PARAMETERS
//Flock Member IDs
# define FLOCK_SIZE 3
# define HUEY_ID	1
# define DEWEY_ID	2
# define LOUIE_ID	3
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//INTER-AC COMMUNICATION NETWORK PARAMETERS
# define FCOM_UART2			ENABLED
//Baud rate for Inter-AC Coms
# define FCOM_BAUD			57600
//GCS link into the Inter-AC Coms
# define FCOM_GCS			ENABLED
//64-bit Address for QGCS XBee
# define QGCS_MSB			0x0013a200
# define QGCS_LSB			0x409a92ff

///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//CUSTOM FF LOG PARAMETERS
// These custom logs can not be turned on/off with CLI, so they must be enabled/disabled here
# define FF_LOGGING_ENABLED				ENABLED
# define LOG_FLOCK_STATUS				DISABLED
# define LOG_PF_FIELD					DISABLED
# define LOG_VWP						DISABLED
# define LOG_REL_STATE					ENABLED
# define LOG_GPS_ERROR_ASSIST			ENABLED
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//RELATIVE ALTITUDE MEASUREMENT
//Use only the altimeter for broadcasted relative altitude
# define FF_BARO_ALT	DISABLED
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//HARDCODE HOME LOCATION AT EFR
# define HARDCODE_EFR_HOME ENABLED
# define HOME_ALT 6500 //cm
# define HOME_LAT 353284311 //1e7
# define HOME_LNG -1207524114 //1e7
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//FLIGHT MODE SELECTIONS

//New Flight Modes
//Uses swarm algorithm and PFG to allign all participants into formation
#define FORMATION			13
//Gives pilot manual control, but continues to broadcast location as if participating in formation
#define MANUAL_IN_FORMATION 14
#define START_LOCATIONS		AUTO

//Pre-defined Flight Modes
# define FLIGHT_MODE_1		START_LOCATIONS
# define FLIGHT_MODE_2      FORMATION
# define FLIGHT_MODE_3		STABILIZE
# define FLIGHT_MODE_4		MANUAL_IN_FORMATION
# define FLIGHT_MODE_5      RTL
# define FLIGHT_MODE_6      MANUAL
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
//PFG/VWP PARAMETERS
////////	Parameter		|	Value
# define	PFG_CHI				0
# define	PFG_TAU				2000
# define	PFG_ZETA			200
# define	PFG_SIGMA			1
# define	PFG_X_LAMBDA		4
# define	PFG_Y_LAMBDA		4
# define	PFG_Z_LAMBDA		4
# define	PFG_x_OFFSET		1000
# define	PFG_y_OFFSET		1000
# define	PFG_z_OFFSET		0
# define	PFG_VWP_XY_OFFSET	60 //Meters
# define	PFG_VWP_Z_OFFSET	2 //meters
# define	PFG_X_PHI_NEAR		3
# define	PFG_K_V_NEAR		100
# define	PFG_K_ALT_V_NEAR	0
# define	PFG_K_PHI_V_NEAR	100
# define	PFG_K_V_FAR			100
# define	PFG_K_ALT_V_FAR		0
# define	PFG_K_PHI_V_FAR		0
# define	PFG_DEFAULT_SIDE	-1

# define	PFG_MIN_ALT			10000
# define	PFG_MAX_ALT			30000
# define	PFG_MIN_AIRSPEED_CM	800
# define	PFG_MAX_AIRSPEED_CM	1400

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OTHER PARAMETER DEFINES
# define	VIEW_RANGE 320
# define	FLOCKMEMBER_TIME_OUT 1500	//Amount of time between messages before we remove flock member from view (ms)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Goal Waypoint to Loiter around
//CalPoly EFR - Arround Lake
# define	GOAL_LAT		353283260
# define	GOAL_LON		-1207533502
# define	GOAL_ALT		15000
//Waypoint Parameters
# define	GOAL_LOITER_R	60		//[m] 60 meter radius loiter 
# define	GOAL_RANGE		60		//Based on minimum turn radius
# define	GOAL_WINDUP		1130973 // [m*100] based on 60 m radius and 30 turn windup
# define	GOAL_MARGIN		20		//Based on a 15 m offset
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AIRFRAME SPECIFIC DESIGNS- DO NOT CHANGE HERE- CHANGE AT THE TOP
#ifdef I_AM_HUEY
	# define AIRFRAME_NAME "HUEY" //for logging
	//Current configuration: 1 = Huey, 2 = Dewey, 3 = Louie
	# define MAV_SYSTEM_ID	HUEY_ID
#endif
#ifdef I_AM_DEWEY
	# define AIRFRAME_NAME "DEWEY" //for logging
	//Current configuration: 1 = Huey, 2 = Dewey, 3 = Louie
	# define MAV_SYSTEM_ID	DEWEY_ID
#endif
#ifdef I_AM_LOUIE
	# define AIRFRAME_NAME "LOUIE" //for logging
	//Current configuration: 1 = Huey, 2 = Dewey, 3 = Louie
	# define MAV_SYSTEM_ID	LOUIE_ID
#endif


