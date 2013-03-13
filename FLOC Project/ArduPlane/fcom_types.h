// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	FCOM.h
/// @brief	Interface definition for the inter-a/c network.

#ifndef __FCOM_TYPES_H
#define __FCOM_TYPES_H

#include<GCS_MAVLink.h>

typedef struct __xbee_api_heartbeat_t
{
	uint8_t mavlink_stx;					///start byte for mavlink protocol
	uint8_t length;							///length of mavlink packet
	uint8_t tx_seq;							///current tx seq
	uint8_t sysid;							///sysid of sender
	uint8_t compid;							///compid of sender
	uint8_t msgid;							///MAVLINK_MSG_ID_HEARTBEAT
	mavlink_heartbeat_t heartbeat;			///mavlink packet for heartbeat 
	uint8_t ck0;							///mavlink checksum high bite
	uint8_t ck1;							///mavlink checksum low bite
} xbee_api_heartbeat_t;


#define MAVLINK_HEARTBEAT_LEN 17			//9 for packet + 8 for non-payload bytes
#define MAVLINK_HEARTBEAT_CRC 50

typedef struct __xbee_api_global_position_int_t
{
	uint8_t mavlink_stx;					///start byte for mavlink protocol
	uint8_t length;							///length of mavlink packet
	uint8_t tx_seq;							///current tx seq
	uint8_t sysid;							///sysid of sender
	uint8_t compid;							///compid of sender
	uint8_t msgid;							///MAVLINK_MSG_ID_GLOBAL_POSITION_INT
	mavlink_global_position_int_t loc_packet;	///mavlink packet for gps state 
	uint8_t ck0;							///mavlink checksum high bite
	uint8_t ck1;							///mavlink checksum low bite
} xbee_api_global_position_int_t;


#define MAVLINK_GLOBAL_POSITION_INT_LEN 36	//28 for packet + 8 for non-payload bytes
#define MAVLINK_GLOBAL_POSITION_INT_CRC 104

typedef struct __xbee_api_ff_flock_status_t
{
	uint8_t mavlink_stx;					///start byte for mavlink protocol
	uint8_t length;							///length of mavlink packet
	uint8_t tx_seq;							///current tx seq
	uint8_t sysid;							///sysid of sender
	uint8_t compid;							///compid of sender
	uint8_t msgid;							///MAVLINK_MSG_ID_FF_FLOCK_STATUS
	mavlink_ff_flock_status_t ff_packet;	///mavlink packet for flock_status 
	uint8_t ck0;							///mavlink checksum high bite
	uint8_t ck1;							///mavlink checksum low bite
} xbee_api_ff_flock_status_t;


#define MAVLINK_FF_FLOCK_STATUS_LEN 26		//18 for packet + 8 for non-payload bytes
#define MAVLINK_FF_FLOCK_STATUS_CRC 6

typedef struct __xbee_api_ff_pf_field_t
{
	uint8_t mavlink_stx;					///start byte for mavlink protocol
	uint8_t length;							///length of mavlink packet
	uint8_t tx_seq;							///current tx seq
	uint8_t sysid;							///sysid of sender
	uint8_t compid;							///compid of sender
	uint8_t msgid;							///MAVLINK_MSG_ID_FF_PF_FIELD
	mavlink_ff_pf_field_t ff_packet;		///mavlink packet for pf_field 
	uint8_t ck0;							///mavlink checksum high bite
	uint8_t ck1;							///mavlink checksum low bite
} xbee_api_ff_pf_field_t;


#define MAVLINK_FF_PF_FIELD_LEN 54			//46 for packet + 8 for non-payload bytes
#define MAVLINK_FF_PF_FIELD_CRC 114

typedef struct __xbee_api_ff_vwp_t
{
	uint8_t mavlink_stx;					///start byte for mavlink protocol
	uint8_t length;							///length of mavlink packet
	uint8_t tx_seq;							///current tx seq
	uint8_t sysid;							///sysid of sender
	uint8_t compid;							///compid of sender
	uint8_t msgid;							///MAVLINK_MSG_ID_FF_VWP
	mavlink_ff_vwp_t ff_packet;				///mavlink packet for virtual waypoint
	uint8_t ck0;							///mavlink checksum high bite
	uint8_t ck1;							///mavlink checksum low bite
} xbee_api_ff_vwp_t;


#define MAVLINK_FF_VWP_LEN 31				//23 for packet + 8 for non-payload bytes
#define MAVLINK_FF_VWP_CRC 61

typedef struct __xbee_api_ff_rel_state_t
{
	uint8_t mavlink_stx;					///start byte for mavlink protocol
	uint8_t length;							///length of mavlink packet
	uint8_t tx_seq;							///current tx seq
	uint8_t sysid;							///sysid of sender
	uint8_t compid;							///compid of sender
	uint8_t msgid;							///MAVLINK_MSG_ID_FF_REL_STATE
	mavlink_ff_rel_state_t ff_packet;		///mavlink packet for relative state 
	uint8_t ck0;							///mavlink checksum high bite
	uint8_t ck1;							///mavlink checksum low bite
} xbee_api_ff_rel_state_t;


#define MAVLINK_FF_REL_STATE_LEN 41			//33 for packet + 8 for non-payload bytes
#define MAVLINK_FF_REL_STATE_CRC 0

typedef struct __xbee_api_ff_gps_error_assist_t
{
	uint8_t mavlink_stx;					///start byte for mavlink protocol
	uint8_t length;							///length of mavlink packet
	uint8_t tx_seq;							///current tx seq
	uint8_t sysid;							///sysid of sender
	uint8_t compid;							///compid of sender
	uint8_t msgid;							///MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST
	mavlink_ff_gps_error_assist_t ff_packet;///mavlink packet for gps error assistance
	uint8_t ck0;							///mavlink checksum high bite
	uint8_t ck1;							///mavlink checksum low bite
} xbee_api_ff_gps_error_assist_t;


#define MAVLINK_FF_GPS_ERROR_ASSIST_LEN 24	//16 for packet + 8 for non-payload bytes
#define MAVLINK_FF_GPS_ERROR_ASSIST_CRC 201

#endif // __FCOM_TYPES_H
