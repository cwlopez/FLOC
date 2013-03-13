// MESSAGE FF_REL_STATE PACKING

#define MAVLINK_MSG_ID_FF_REL_STATE 193

typedef struct __mavlink_ff_rel_state_t
{
 uint64_t time_usec; ///< Timestamp for data sync(microseconds since UNIX epoch)
 float dx; ///< relative x position (meters)
 float dy; ///< relative Y Position
 float dz; ///< relative Z Position
 float dvx; ///< relative X Speed
 float dvy; ///< relative Y Speed
 float dvz; ///< relative Z Speed
 uint8_t coordinate_frame; ///< Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU, MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
} mavlink_ff_rel_state_t;

#define MAVLINK_MSG_ID_FF_REL_STATE_LEN 33
#define MAVLINK_MSG_ID_193_LEN 33



#define MAVLINK_MESSAGE_INFO_FF_REL_STATE { \
	"FF_REL_STATE", \
	8, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ff_rel_state_t, time_usec) }, \
         { "dx", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ff_rel_state_t, dx) }, \
         { "dy", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ff_rel_state_t, dy) }, \
         { "dz", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ff_rel_state_t, dz) }, \
         { "dvx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ff_rel_state_t, dvx) }, \
         { "dvy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ff_rel_state_t, dvy) }, \
         { "dvz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ff_rel_state_t, dvz) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_ff_rel_state_t, coordinate_frame) }, \
         } \
}


/**
 * @brief Pack a ff_rel_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU, MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param dx relative x position (meters)
 * @param dy relative Y Position
 * @param dz relative Z Position
 * @param dvx relative X Speed
 * @param dvy relative Y Speed
 * @param dvz relative Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_rel_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t coordinate_frame, float dx, float dy, float dz, float dvx, float dvy, float dvz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, dx);
	_mav_put_float(buf, 12, dy);
	_mav_put_float(buf, 16, dz);
	_mav_put_float(buf, 20, dvx);
	_mav_put_float(buf, 24, dvy);
	_mav_put_float(buf, 28, dvz);
	_mav_put_uint8_t(buf, 32, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 33);
#else
	mavlink_ff_rel_state_t packet;
	packet.time_usec = time_usec;
	packet.dx = dx;
	packet.dy = dy;
	packet.dz = dz;
	packet.dvx = dvx;
	packet.dvy = dvy;
	packet.dvz = dvz;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 33);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_REL_STATE;
	return mavlink_finalize_message(msg, system_id, component_id, 33, 0);
}

/**
 * @brief Pack a ff_rel_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU, MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param dx relative x position (meters)
 * @param dy relative Y Position
 * @param dz relative Z Position
 * @param dvx relative X Speed
 * @param dvy relative Y Speed
 * @param dvz relative Z Speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_rel_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t coordinate_frame,float dx,float dy,float dz,float dvx,float dvy,float dvz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, dx);
	_mav_put_float(buf, 12, dy);
	_mav_put_float(buf, 16, dz);
	_mav_put_float(buf, 20, dvx);
	_mav_put_float(buf, 24, dvy);
	_mav_put_float(buf, 28, dvz);
	_mav_put_uint8_t(buf, 32, coordinate_frame);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 33);
#else
	mavlink_ff_rel_state_t packet;
	packet.time_usec = time_usec;
	packet.dx = dx;
	packet.dy = dy;
	packet.dz = dz;
	packet.dvx = dvx;
	packet.dvy = dvy;
	packet.dvz = dvz;
	packet.coordinate_frame = coordinate_frame;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 33);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_REL_STATE;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 33, 0);
}

/**
 * @brief Encode a ff_rel_state struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ff_rel_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ff_rel_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ff_rel_state_t* ff_rel_state)
{
	return mavlink_msg_ff_rel_state_pack(system_id, component_id, msg, ff_rel_state->time_usec, ff_rel_state->coordinate_frame, ff_rel_state->dx, ff_rel_state->dy, ff_rel_state->dz, ff_rel_state->dvx, ff_rel_state->dvy, ff_rel_state->dvz);
}

/**
 * @brief Send a ff_rel_state message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU, MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param dx relative x position (meters)
 * @param dy relative Y Position
 * @param dz relative Z Position
 * @param dvx relative X Speed
 * @param dvy relative Y Speed
 * @param dvz relative Z Speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ff_rel_state_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t coordinate_frame, float dx, float dy, float dz, float dvx, float dvy, float dvz)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[33];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, dx);
	_mav_put_float(buf, 12, dy);
	_mav_put_float(buf, 16, dz);
	_mav_put_float(buf, 20, dvx);
	_mav_put_float(buf, 24, dvy);
	_mav_put_float(buf, 28, dvz);
	_mav_put_uint8_t(buf, 32, coordinate_frame);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_REL_STATE, buf, 33, 0);
#else
	mavlink_ff_rel_state_t packet;
	packet.time_usec = time_usec;
	packet.dx = dx;
	packet.dy = dy;
	packet.dz = dz;
	packet.dvx = dvx;
	packet.dvy = dvy;
	packet.dvz = dvz;
	packet.coordinate_frame = coordinate_frame;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_REL_STATE, (const char *)&packet, 33, 0);
#endif
}

#endif

// MESSAGE FF_REL_STATE UNPACKING


/**
 * @brief Get field time_usec from ff_rel_state message
 *
 * @return Timestamp for data sync(microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_ff_rel_state_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field coordinate_frame from ff_rel_state message
 *
 * @return Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU, MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 */
static inline uint8_t mavlink_msg_ff_rel_state_get_coordinate_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field dx from ff_rel_state message
 *
 * @return relative x position (meters)
 */
static inline float mavlink_msg_ff_rel_state_get_dx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field dy from ff_rel_state message
 *
 * @return relative Y Position
 */
static inline float mavlink_msg_ff_rel_state_get_dy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field dz from ff_rel_state message
 *
 * @return relative Z Position
 */
static inline float mavlink_msg_ff_rel_state_get_dz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field dvx from ff_rel_state message
 *
 * @return relative X Speed
 */
static inline float mavlink_msg_ff_rel_state_get_dvx(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field dvy from ff_rel_state message
 *
 * @return relative Y Speed
 */
static inline float mavlink_msg_ff_rel_state_get_dvy(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field dvz from ff_rel_state message
 *
 * @return relative Z Speed
 */
static inline float mavlink_msg_ff_rel_state_get_dvz(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a ff_rel_state message into a struct
 *
 * @param msg The message to decode
 * @param ff_rel_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_ff_rel_state_decode(const mavlink_message_t* msg, mavlink_ff_rel_state_t* ff_rel_state)
{
#if MAVLINK_NEED_BYTE_SWAP
	ff_rel_state->time_usec = mavlink_msg_ff_rel_state_get_time_usec(msg);
	ff_rel_state->dx = mavlink_msg_ff_rel_state_get_dx(msg);
	ff_rel_state->dy = mavlink_msg_ff_rel_state_get_dy(msg);
	ff_rel_state->dz = mavlink_msg_ff_rel_state_get_dz(msg);
	ff_rel_state->dvx = mavlink_msg_ff_rel_state_get_dvx(msg);
	ff_rel_state->dvy = mavlink_msg_ff_rel_state_get_dvy(msg);
	ff_rel_state->dvz = mavlink_msg_ff_rel_state_get_dvz(msg);
	ff_rel_state->coordinate_frame = mavlink_msg_ff_rel_state_get_coordinate_frame(msg);
#else
	memcpy(ff_rel_state, _MAV_PAYLOAD(msg), 33);
#endif
}
