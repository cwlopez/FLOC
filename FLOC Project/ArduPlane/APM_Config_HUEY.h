// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This is the configuration file for HUEY

#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

#define SYSID 1
//Use the UART2 Pins on the APM Board
# define TELEMETRY_UART2 ENABLED
#define HIL_MODE        HIL_MODE_ATTITUDE

# define GEOFENCE_ENABLED DISABLED
# define MAV_SYSTEM_ID          1 //1 = Huey, 2 = Dewey, 3 = Louie

#define FORMATION_FLIGHT_ENABLED 1

#define HUEY_ID		1
#define DEWEY_ID	2
#define LOUIE_ID	3

#define FORMATION	13
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
# define	PFG_TAU				8
# define	PFG_ZETA			2000
# define	PFG_SIGMA			1
# define	PFG_X_LAMBDA		60
# define	PFG_Y_LAMBDA		50
# define	PFG_Z_LAMBDA		40
# define	PFG_x_OFFSET		2000
# define	PFG_y_OFFSET		2000
# define	PFG_z_OFFSET		2000
# define	PFG_VWP_OFFSET		45
# define	PFG_X_PHI_NEAR		3
# define	PFG_K_PHI_X_NEAR	1
# define	PFG_K_PHI_Y_NEAR	1
# define	PFG_K_PHI_Z_NEAR	1
# define	PFG_K_V_NEAR		1
# define	PFG_K_ALT_V_NEAR	1
# define	PFG_K_PHI_V_NEAR	1
# define	PFG_K_PHI_X_FAR		1
# define	PFG_K_PHI_Y_FAR		1
# define	PFG_K_PHI_Z_FAR		1
# define	PFG_K_V_FAR			1
# define	PFG_K_ALT_V_FAR		1
# define	PFG_K_PHI_V_FAR		1
# define	PFG_DEFAULT_SIDE	-1

# define	PFG_MIN_ALT			100
# define	PFG_MAX_ALT			300
# define	PFG_MIN_AIRSPEED_CM	1000
# define	PFG_MAX_AIRSPEED_CM	2000
// Other Formation Flight defines
# define	FLOCK_SIZE 3
# define	VIEW_RANGE 180
# define	FLOCKMEMBER_TIME_OUT 1500	//Amount of time between messages before we remove flock member from view (ms)
//Goal Waypoint to Loiter around
# define	GOAL_LAT	35193797
# define	GOAL_LON	-120451022
# define	GOAL_ALT	150