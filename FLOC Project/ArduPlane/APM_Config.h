// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//This is to keep the programmers happy, and not edit the main config.h file
//All edited configurations are in each file

# define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

//Use the UART2 Pins on the APM Board for Telemetry
# define TELEMETRY_UART2	DISABLED
//Use the UART2 Pins on the APM Board for Inter-AC Coms
# define FCOM_UART2			ENABLED
//Baud rate for Inter-AC Coms
# define FCOM_BAUD			57600
//64-bit Address for QGCS XBee
# define QGCS_MSB			0x0013a200
# define QGCS_LSB			0x409a92ff

//Use HIL Firmware w/ Attitude Supplied
# define HIL_MODE			HIL_MODE_ATTITUDE
//Use the Serial1 Port for Debugging w/ Visual Micro
# define HIL_DEBUG			DISABLED

// Changing these will not change anything, they must be changed in the parameters or CLI
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

// These custom logs can not be turned on/off with CLI, so they must be enabled/disabled here
# define FF_LOGGING_ENABLED				ENABLED
# define LOG_FLOCK_STATUS				ENABLED
# define LOG_PF_FIELD					ENABLED
# define LOG_VWP						ENABLED
# define LOG_REL_STATE					ENABLED
# define LOG_GPS_ERROR_ASSIST			ENABLED

# define GEOFENCE_ENABLED	DISABLED
//Enable Formation Flight Firmware
# define FORMATION_FLIGHT	ENABLED
//Use only the altimeter for broadcasted relative altitude
# define FF_BARO_ALT	ENABLED

#include "APM_Config_HUEY.h"
//#include "APM_Config_DEWEY.h"
//#include "APM_Config_LOUIE.h"


