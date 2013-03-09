// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This is the configuration file for HUEY

# define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

//Use the UART2 Pins on the APM Board
# define TELEMETRY_UART2	ENABLED
//Use HIL Firmware w/ Attitude Supplied
# define HIL_MODE			HIL_MODE_DISABLED
//Use the Serial1 Port for Debugging w/ Visual Micro
# define HIL_DEBUG			DISABLED

# define LOGGING_ENABLED                ENABLED
# define LOG_ATTITUDE_FAST              DISABLED
# define LOG_ATTITUDE_MED               DISABLED
# define LOG_GPS                        ENABLED
# define LOG_PM                         DISABLED
# define LOG_CTUN                       DISABLED
# define LOG_NTUN                       ENABLED
# define LOG_MODE                       ENABLED
# define LOG_RAW                        DISABLED
# define LOG_CMD                        DISABLED
# define LOG_CUR                        DISABLED

# define GEOFENCE_ENABLED	DISABLED
//Enable Formation Flight Firmware
# define FORMATION_FLIGHT	ENABLED
//Identifier for a/c 
//Current configuration: 1 = Huey, 2 = Dewey, 3 = Louie
# define MAV_SYSTEM_ID		3
//Other Flock Member IDs
#define HUEY_ID		1
#define DEWEY_ID	2
#define LOUIE_ID	3
//New Flight Modes
//Uses swarm algorithm and PFG to allign all participants into formation
#define FORMATION	13
//Gives pilot manual control, but continues to broadcast location as if participating in formation
#define MANUAL_IN_FORMATION 14

//Pre-defined Flight Modes
# define FLIGHT_MODE_1                  LOITER
# define FLIGHT_MODE_2                  FORMATION
# define FLIGHT_MODE_3                  STABILIZE
# define FLIGHT_MODE_4                  MANUAL_IN_FORMATION
# define FLIGHT_MODE_5                  RTL
# define FLIGHT_MODE_6                  MANUAL

//////////////////////////////////////////////////////////////////////////////
// CAMERA TRIGGER AND CONTROL
//
// uses 1182 bytes of memory
# define CAMERA         DISABLED

//////////////////////////////////////////////////////////////////////////////
// MOUNT (ANTENNA OR CAMERA)
//
// uses 4174 bytes of memory on 1280 chips (MNT_JSTICK_SPD_OPTION, MNT_RETRACT_OPTION, MNT_STABILIZE_OPTION and MNT_MOUNT2_OPTION disabled)
// uses 7726 bytes of memory on 2560 chips (all options are enabled)
# define MOUNT          DISABLED

// second mount, can for example be used to keep an antenna pointed at the home position
#ifndef MOUNT2
 # define MOUNT2         DISABLED
#endif

/////////////////////////////////////////////////////////////////////////////
//Formation Flight Stuff
////////	Parameter		|	Value
# define	PFG_CHI				30
# define	PFG_TAU				200
# define	PFG_ZETA			25
# define	PFG_SIGMA			1
# define	PFG_X_LAMBDA		60
# define	PFG_Y_LAMBDA		50
# define	PFG_Z_LAMBDA		40
# define	PFG_x_OFFSET		1000
# define	PFG_y_OFFSET		1000
# define	PFG_z_OFFSET		0
# define	PFG_VWP_OFFSET		60 //Meters
# define	PFG_X_PHI_NEAR		3
# define	PFG_K_PHI_X_NEAR	100
# define	PFG_K_PHI_Y_NEAR	100
# define	PFG_K_PHI_Z_NEAR	100
# define	PFG_K_V_NEAR		100
# define	PFG_K_ALT_V_NEAR	0
# define	PFG_K_PHI_V_NEAR	0
# define	PFG_K_PHI_X_FAR		100
# define	PFG_K_PHI_Y_FAR		100
# define	PFG_K_PHI_Z_FAR		100
# define	PFG_K_V_FAR			100
# define	PFG_K_ALT_V_FAR		0
# define	PFG_K_PHI_V_FAR		0
# define	PFG_DEFAULT_SIDE	1

# define	PFG_MIN_ALT			10000
# define	PFG_MAX_ALT			30000
# define	PFG_MIN_AIRSPEED_CM	800
# define	PFG_MAX_AIRSPEED_CM	1400
// Other Formation Flight defines
# define	FLOCK_SIZE 3
# define	VIEW_RANGE 360
# define	FLOCKMEMBER_TIME_OUT 1500	//Amount of time between messages before we remove flock member from view (ms)
//Goal Waypoint to Loiter around
//South East corner of CalPoly EFR
# define	GOAL_LAT	353283260
# define	GOAL_LON	-1207533502
# define	GOAL_ALT	15000