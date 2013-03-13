// MESSAGE FF_GPS_ERROR_ASSIST PACKING

#define MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST 194

typedef struct __mavlink_ff_gps_error_assist_t
{
 uint64_t time_usec; ///< Timestamp for data sync(microseconds since UNIX epoch)
 int32_t alt_rel; ///< Altitude in meters *1000 (millimeters) AGL (Pure GPS) 
 uint16_t eph; ///< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 uint8_t fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 uint8_t satellites_visible; ///< Number of satellites visible. If unknown, set to 255
} mavlink_ff_gps_error_assist_t;

#define MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST_LEN 16
#define MAVLINK_MSG_ID_194_LEN 16



#define MAVLINK_MESSAGE_INFO_FF_GPS_ERROR_ASSIST { \
	"FF_GPS_ERROR_ASSIST", \
	5, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ff_gps_error_assist_t, time_usec) }, \
         { "alt_rel", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_ff_gps_error_assist_t, alt_rel) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_ff_gps_error_assist_t, eph) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_ff_gps_error_assist_t, fix_type) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_ff_gps_error_assist_t, satellites_visible) }, \
         } \
}


/**
 * @brief Pack a ff_gps_error_assist message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @param alt_rel Altitude in meters *1000 (millimeters) AGL (Pure GPS) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_gps_error_assist_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t fix_type, uint16_t eph, uint8_t satellites_visible, int32_t alt_rel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, alt_rel);
	_mav_put_uint16_t(buf, 12, eph);
	_mav_put_uint8_t(buf, 14, fix_type);
	_mav_put_uint8_t(buf, 15, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 16);
#else
	mavlink_ff_gps_error_assist_t packet;
	packet.time_usec = time_usec;
	packet.alt_rel = alt_rel;
	packet.eph = eph;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 16);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST;
	return mavlink_finalize_message(msg, system_id, component_id, 16, 201);
}

/**
 * @brief Pack a ff_gps_error_assist message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @param alt_rel Altitude in meters *1000 (millimeters) AGL (Pure GPS) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_gps_error_assist_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t fix_type,uint16_t eph,uint8_t satellites_visible,int32_t alt_rel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, alt_rel);
	_mav_put_uint16_t(buf, 12, eph);
	_mav_put_uint8_t(buf, 14, fix_type);
	_mav_put_uint8_t(buf, 15, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 16);
#else
	mavlink_ff_gps_error_assist_t packet;
	packet.time_usec = time_usec;
	packet.alt_rel = alt_rel;
	packet.eph = eph;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 16);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 16, 201);
}

/**
 * @brief Encode a ff_gps_error_assist struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ff_gps_error_assist C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ff_gps_error_assist_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ff_gps_error_assist_t* ff_gps_error_assist)
{
	return mavlink_msg_ff_gps_error_assist_pack(system_id, component_id, msg, ff_gps_error_assist->time_usec, ff_gps_error_assist->fix_type, ff_gps_error_assist->eph, ff_gps_error_assist->satellites_visible, ff_gps_error_assist->alt_rel);
}

/**
 * @brief Send a ff_gps_error_assist message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @param alt_rel Altitude in meters *1000 (millimeters) AGL (Pure GPS) 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ff_gps_error_assist_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t fix_type, uint16_t eph, uint8_t satellites_visible, int32_t alt_rel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[16];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, alt_rel);
	_mav_put_uint16_t(buf, 12, eph);
	_mav_put_uint8_t(buf, 14, fix_type);
	_mav_put_uint8_t(buf, 15, satellites_visible);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST, buf, 16, 201);
#else
	mavlink_ff_gps_error_assist_t packet;
	packet.time_usec = time_usec;
	packet.alt_rel = alt_rel;
	packet.eph = eph;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST, (const char *)&packet, 16, 201);
#endif
}

#endif

// MESSAGE FF_GPS_ERROR_ASSIST UNPACKING


/**
 * @brief Get field time_usec from ff_gps_error_assist message
 *
 * @return Timestamp for data sync(microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_ff_gps_error_assist_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field fix_type from ff_gps_error_assist message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
static inline uint8_t mavlink_msg_ff_gps_error_assist_get_fix_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field eph from ff_gps_error_assist message
 *
 * @return GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_ff_gps_error_assist_get_eph(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field satellites_visible from ff_gps_error_assist message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_ff_gps_error_assist_get_satellites_visible(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field alt_rel from ff_gps_error_assist message
 *
 * @return Altitude in meters *1000 (millimeters) AGL (Pure GPS) 
 */
static inline int32_t mavlink_msg_ff_gps_error_assist_get_alt_rel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a ff_gps_error_assist message into a struct
 *
 * @param msg The message to decode
 * @param ff_gps_error_assist C-struct to decode the message contents into
 */
static inline void mavlink_msg_ff_gps_error_assist_decode(const mavlink_message_t* msg, mavlink_ff_gps_error_assist_t* ff_gps_error_assist)
{
#if MAVLINK_NEED_BYTE_SWAP
	ff_gps_error_assist->time_usec = mavlink_msg_ff_gps_error_assist_get_time_usec(msg);
	ff_gps_error_assist->alt_rel = mavlink_msg_ff_gps_error_assist_get_alt_rel(msg);
	ff_gps_error_assist->eph = mavlink_msg_ff_gps_error_assist_get_eph(msg);
	ff_gps_error_assist->fix_type = mavlink_msg_ff_gps_error_assist_get_fix_type(msg);
	ff_gps_error_assist->satellites_visible = mavlink_msg_ff_gps_error_assist_get_satellites_visible(msg);
#else
	memcpy(ff_gps_error_assist, _MAV_PAYLOAD(msg), 16);
#endif
}
