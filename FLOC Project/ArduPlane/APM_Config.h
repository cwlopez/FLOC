// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//This is to keep the programmers happy, and not edit the main config.h file
//All edited configurations are in each file

# define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

//Enable Formation Flight Firmware
# define FORMATION_FLIGHT	ENABLED

//Use the UART2 Pins on the APM Board for Telemetry
# define TELEMETRY_UART2	DISABLED

//Use HIL Firmware w/ Attitude Supplied
# define HIL_MODE			HIL_MODE_DISABLED
//Use the Serial1 Port for Debugging w/ Visual Micro
# define HIL_DEBUG			DISABLED

// Changing these will not change anything, they must be changed in the parameters or CLI
# define LOGGING_ENABLED                ENABLED
# define LOG_ATTITUDE_FAST              DISABLED
# define LOG_ATTITUDE_MED               DISABLED
# define LOG_GPS                        ENABLED
# define LOG_PM                         DISABLED
# define LOG_CTUN                       DISABLED
# define LOG_NTUN                       DISABLED
# define LOG_MODE                       ENABLED
# define LOG_RAW                        DISABLED
# define LOG_CMD                        DISABLED
# define LOG_CUR                        DISABLED

# define GEOFENCE_ENABLED	DISABLED

# if FORMATION_FLIGHT == ENABLED
	# include "APM_Config_Formation.h"
# endif