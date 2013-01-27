// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This is the configuration file for HUEY

#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

#define SYSID 1
//Use the UART2 Pins on the APM Board
# define TELEMETRY_UART2 ENABLED

# define GEOFENCE_ENABLED DISABLED
# define MAV_SYSTEM_ID          1 //1 = Huey, 2 = Dewey, 3 = Louie

#define FORMATION_FLIGHT_ENABLED 0

#define HUEY_ID		1
#define DEWEY_ID	2
#define LOUIE_ID	3
#define FORMATION	13
#define MANUAL_IN_FORMATION 14

# define CH5_MIN        962
# define CH5_MAX        965

# define CH6_MIN        1500
# define CH6_MAX        1501

# define CH7_MIN        1500
# define CH7_MAX        1501

# define CH8_MIN        966
# define CH8_MAX        2074

//Pre-defined Flight Modes
# define FLIGHT_MODE_1                  LOITER
#
# define FLIGHT_MODE_2                  FORMATION
# define FLIGHT_MODE_3                  STABILIZE
# define FLIGHT_MODE_4                  MANUAL_IN_FORMATION
# define FLIGHT_MODE_5                  RTL
# define FLIGHT_MODE_6                  MANUAL
//////////////////////////////////////////////////////////////////////////////
// Altitude measurement and control.
//
# define ALT_EST_GAIN                   0.01
# define ALTITUDE_MIX                   1
//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_CRUISE
//
# define AIRSPEED_CRUISE				13.40 // 12 m/s		
//////////////////////////////////////////////////////////////////////////////
// Servo Mapping
//
# define THROTTLE_MIN                   0 // percent
# define THROTTLE_CRUISE                45
# define THROTTLE_MAX                   75
//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
//
# define HEAD_MAX                               45
# define PITCH_MAX                              20
# define PITCH_MIN                              -25

//////////////////////////////////////////////////////////////////////////////
// Attitude control gains
//

# define SERVO_ROLL_P         0.4
# define SERVO_ROLL_I         0.0
# define SERVO_ROLL_D         0.0
# define SERVO_ROLL_INT_MAX   5
# define ROLL_SLEW_LIMIT      0

# define SERVO_PITCH_P        0.6
# define SERVO_PITCH_I        0.0
# define SERVO_PITCH_D        0.0
# define SERVO_PITCH_INT_MAX  5
# define PITCH_COMP           0.2

# define SERVO_YAW_P          0.0
# define SERVO_YAW_I          0.0
# define SERVO_YAW_D          0.0
# define SERVO_YAW_INT_MAX    0
# define RUDDER_MIX           0.5

//////////////////////////////////////////////////////////////////////////////
// Navigation control gains
//
# define NAV_ROLL_P           0.7
# define NAV_ROLL_I           0.02
# define NAV_ROLL_D           0.1
# define NAV_ROLL_INT_MAX     5

# define NAV_PITCH_ASP_P      0.65
# define NAV_PITCH_ASP_I      0.1
# define NAV_PITCH_ASP_D      0.0
# define NAV_PITCH_ASP_INT_MAX 5


# define NAV_PITCH_ALT_P      0.65
# define NAV_PITCH_ALT_I      0.1
# define NAV_PITCH_ALT_D      0.0
# define NAV_PITCH_ALT_INT_MAX 5

//////////////////////////////////////////////////////////////////////////////
// Energy/Altitude control gains
//
# define THROTTLE_TE_P        0.50
# define THROTTLE_TE_I        0.0
# define THROTTLE_TE_D        0.0
# define THROTTLE_TE_INT_MAX  20
# define THROTTLE_SLEW_LIMIT  0
# define P_TO_T               0
# define T_TO_P               0
# define PITCH_TARGET         0

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
# define XTRACK_GAIN          1 // deg/m
# define XTRACK_ENTRY_ANGLE   30 // deg
# define XTRACK_GAIN_SCALED XTRACK_GAIN*100
//////////////////////////////////////////////////////////////////////////////
// Navigation defaults
//
# define WP_RADIUS_DEFAULT              20

# define LOITER_RADIUS_DEFAULT 30

# define ALT_HOLD_HOME 100

# define USE_CURRENT_ALT FALSE

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
# define	FORMATION_CURRENT_STATE 150 //Eroneous ID at the moment...may conflict with pre-defined MAVLink message ID
//Goal Waypoint to Loiter around
# define	GOAL_LAT	35193797
# define	GOAL_LON	120451022
# define	GOAL_ALT	150