// MESSAGE FF_VWP PACKING

#define MAVLINK_MSG_ID_FF_VWP 192

typedef struct __mavlink_ff_vwp_t
{
 uint64_t time_usec; ///< Timestamp for data sync(microseconds since UNIX epoch)
 int32_t lat; ///<  Latitude of commanded virtual waypoint in degrees * 1E7
 int32_t lon; ///<  Longitude of commanded virtual waypoint in degrees * 1E7
 int32_t alt; ///<  Altitude in meters * 1000 (positive for up)
 uint16_t spd; ///< commanded airspeed or GPS groundspeed (m/s * 100). If unknown, set to: 65535
 uint8_t coordinate_frame; ///< Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
} mavlink_ff_vwp_t;

#define MAVLINK_MSG_ID_FF_VWP_LEN 23
#define MAVLINK_MSG_ID_192_LEN 23



#define MAVLINK_MESSAGE_INFO_FF_VWP { \
	"FF_VWP", \
	6, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ff_vwp_t, time_usec) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_ff_vwp_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_ff_vwp_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_ff_vwp_t, alt) }, \
         { "spd", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_ff_vwp_t, spd) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_ff_vwp_t, coordinate_frame) }, \
         } \
}


/**
 * @brief Pack a ff_vwp message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param lat  Latitude of commanded virtual waypoint in degrees * 1E7
 * @param lon  Longitude of commanded virtual waypoint in degrees * 1E7
 * @param alt  Altitude in meters * 1000 (positive for up)
 * @param spd commanded airspeed or GPS groundspeed (m/s * 100). If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_vwp_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t coordinate_frame, int32_t lat, int32_t lon, int32_t alt, uint16_t spd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[23];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, spd);
	_mav_put_uint8_t(buf, 22, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 23);
#else
	mavlink_ff_vwp_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.spd = spd;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 23);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_VWP;
	return mavlink_finalize_message(msg, system_id, component_id, 23, 61);
}

/**
 * @brief Pack a ff_vwp message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param lat  Latitude of commanded virtual waypoint in degrees * 1E7
 * @param lon  Longitude of commanded virtual waypoint in degrees * 1E7
 * @param alt  Altitude in meters * 1000 (positive for up)
 * @param spd commanded airspeed or GPS groundspeed (m/s * 100). If unknown, set to: 65535
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_vwp_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t coordinate_frame,int32_t lat,int32_t lon,int32_t alt,uint16_t spd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[23];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, spd);
	_mav_put_uint8_t(buf, 22, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 23);
#else
	mavlink_ff_vwp_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.spd = spd;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 23);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_VWP;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 23, 61);
}

/**
 * @brief Encode a ff_vwp struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ff_vwp C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ff_vwp_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ff_vwp_t* ff_vwp)
{
	return mavlink_msg_ff_vwp_pack(system_id, component_id, msg, ff_vwp->time_usec, ff_vwp->coordinate_frame, ff_vwp->lat, ff_vwp->lon, ff_vwp->alt, ff_vwp->spd);
}

/**
 * @brief Send a ff_vwp message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param lat  Latitude of commanded virtual waypoint in degrees * 1E7
 * @param lon  Longitude of commanded virtual waypoint in degrees * 1E7
 * @param alt  Altitude in meters * 1000 (positive for up)
 * @param spd commanded airspeed or GPS groundspeed (m/s * 100). If unknown, set to: 65535
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ff_vwp_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t coordinate_frame, int32_t lat, int32_t lon, int32_t alt, uint16_t spd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[23];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_int32_t(buf, 12, lon);
	_mav_put_int32_t(buf, 16, alt);
	_mav_put_uint16_t(buf, 20, spd);
	_mav_put_uint8_t(buf, 22, coordinate_frame);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_VWP, buf, 23, 61);
#else
	mavlink_ff_vwp_t packet;
	packet.time_usec = time_usec;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.spd = spd;
	packet.coordinate_frame = coordinate_frame;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_VWP, (const char *)&packet, 23, 61);
#endif
}

#endif

// MESSAGE FF_VWP UNPACKING


/**
 * @brief Get field time_usec from ff_vwp message
 *
 * @return Timestamp for data sync(microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_ff_vwp_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field coordinate_frame from ff_vwp message
 *
 * @return Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 */
static inline uint8_t mavlink_msg_ff_vwp_get_coordinate_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field lat from ff_vwp message
 *
 * @return  Latitude of commanded virtual waypoint in degrees * 1E7
 */
static inline int32_t mavlink_msg_ff_vwp_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from ff_vwp message
 *
 * @return  Longitude of commanded virtual waypoint in degrees * 1E7
 */
static inline int32_t mavlink_msg_ff_vwp_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from ff_vwp message
 *
 * @return  Altitude in meters * 1000 (positive for up)
 */
static inline int32_t mavlink_msg_ff_vwp_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field spd from ff_vwp message
 *
 * @return commanded airspeed or GPS groundspeed (m/s * 100). If unknown, set to: 65535
 */
static inline uint16_t mavlink_msg_ff_vwp_get_spd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Decode a ff_vwp message into a struct
 *
 * @param msg The message to decode
 * @param ff_vwp C-struct to decode the message contents into
 */
static inline void mavlink_msg_ff_vwp_decode(const mavlink_message_t* msg, mavlink_ff_vwp_t* ff_vwp)
{
#if MAVLINK_NEED_BYTE_SWAP
	ff_vwp->time_usec = mavlink_msg_ff_vwp_get_time_usec(msg);
	ff_vwp->lat = mavlink_msg_ff_vwp_get_lat(msg);
	ff_vwp->lon = mavlink_msg_ff_vwp_get_lon(msg);
	ff_vwp->alt = mavlink_msg_ff_vwp_get_alt(msg);
	ff_vwp->spd = mavlink_msg_ff_vwp_get_spd(msg);
	ff_vwp->coordinate_frame = mavlink_msg_ff_vwp_get_coordinate_frame(msg);
#else
	memcpy(ff_vwp, _MAV_PAYLOAD(msg), 23);
#endif
}
