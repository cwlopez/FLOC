// MESSAGE FF_PF_FIELD PACKING

#define MAVLINK_MSG_ID_FF_PF_FIELD 191

typedef struct __mavlink_ff_pf_field_t
{
 uint64_t time_usec; ///< Timestamp for data sync(microseconds since UNIX epoch)
 float x_att; ///<  X component of the attractive potential gradient
 float y_att; ///<  Y component of the attractive potential gradient
 float z_att; ///<  Z component of the attractive potential gradient
 float x_rep; ///<  X component of the repulsive potential gradient
 float y_rep; ///<  Y component of the repulsive potential gradient
 float z_rep; ///<  Z component of the repulsive potential gradient
 float x_norm; ///<  X component of the normalized total potential gradient
 float y_norm; ///<  Y component of the normalized total potential gradient
 float z_norm; ///<  Z component of the normalized total potential gradient
 uint8_t coordinate_frame; ///< Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 uint8_t regime_mask; ///<  1st bit : 0 for far-field, 1 for near-field; 2nd bit : 0 for uncorrected, 1 for corrected
} mavlink_ff_pf_field_t;

#define MAVLINK_MSG_ID_FF_PF_FIELD_LEN 46
#define MAVLINK_MSG_ID_191_LEN 46



#define MAVLINK_MESSAGE_INFO_FF_PF_FIELD { \
	"FF_PF_FIELD", \
	12, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ff_pf_field_t, time_usec) }, \
         { "x_att", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ff_pf_field_t, x_att) }, \
         { "y_att", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ff_pf_field_t, y_att) }, \
         { "z_att", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ff_pf_field_t, z_att) }, \
         { "x_rep", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ff_pf_field_t, x_rep) }, \
         { "y_rep", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ff_pf_field_t, y_rep) }, \
         { "z_rep", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ff_pf_field_t, z_rep) }, \
         { "x_norm", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ff_pf_field_t, x_norm) }, \
         { "y_norm", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ff_pf_field_t, y_norm) }, \
         { "z_norm", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ff_pf_field_t, z_norm) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_ff_pf_field_t, coordinate_frame) }, \
         { "regime_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_ff_pf_field_t, regime_mask) }, \
         } \
}


/**
 * @brief Pack a ff_pf_field message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param regime_mask  1st bit : 0 for far-field, 1 for near-field; 2nd bit : 0 for uncorrected, 1 for corrected
 * @param x_att  X component of the attractive potential gradient
 * @param y_att  Y component of the attractive potential gradient
 * @param z_att  Z component of the attractive potential gradient
 * @param x_rep  X component of the repulsive potential gradient
 * @param y_rep  Y component of the repulsive potential gradient
 * @param z_rep  Z component of the repulsive potential gradient
 * @param x_norm  X component of the normalized total potential gradient
 * @param y_norm  Y component of the normalized total potential gradient
 * @param z_norm  Z component of the normalized total potential gradient
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_pf_field_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t coordinate_frame, uint8_t regime_mask, float x_att, float y_att, float z_att, float x_rep, float y_rep, float z_rep, float x_norm, float y_norm, float z_norm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[46];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, x_att);
	_mav_put_float(buf, 12, y_att);
	_mav_put_float(buf, 16, z_att);
	_mav_put_float(buf, 20, x_rep);
	_mav_put_float(buf, 24, y_rep);
	_mav_put_float(buf, 28, z_rep);
	_mav_put_float(buf, 32, x_norm);
	_mav_put_float(buf, 36, y_norm);
	_mav_put_float(buf, 40, z_norm);
	_mav_put_uint8_t(buf, 44, coordinate_frame);
	_mav_put_uint8_t(buf, 45, regime_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 46);
#else
	mavlink_ff_pf_field_t packet;
	packet.time_usec = time_usec;
	packet.x_att = x_att;
	packet.y_att = y_att;
	packet.z_att = z_att;
	packet.x_rep = x_rep;
	packet.y_rep = y_rep;
	packet.z_rep = z_rep;
	packet.x_norm = x_norm;
	packet.y_norm = y_norm;
	packet.z_norm = z_norm;
	packet.coordinate_frame = coordinate_frame;
	packet.regime_mask = regime_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 46);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_PF_FIELD;
	return mavlink_finalize_message(msg, system_id, component_id, 46, 114);
}

/**
 * @brief Pack a ff_pf_field message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param regime_mask  1st bit : 0 for far-field, 1 for near-field; 2nd bit : 0 for uncorrected, 1 for corrected
 * @param x_att  X component of the attractive potential gradient
 * @param y_att  Y component of the attractive potential gradient
 * @param z_att  Z component of the attractive potential gradient
 * @param x_rep  X component of the repulsive potential gradient
 * @param y_rep  Y component of the repulsive potential gradient
 * @param z_rep  Z component of the repulsive potential gradient
 * @param x_norm  X component of the normalized total potential gradient
 * @param y_norm  Y component of the normalized total potential gradient
 * @param z_norm  Z component of the normalized total potential gradient
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_pf_field_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t coordinate_frame,uint8_t regime_mask,float x_att,float y_att,float z_att,float x_rep,float y_rep,float z_rep,float x_norm,float y_norm,float z_norm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[46];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, x_att);
	_mav_put_float(buf, 12, y_att);
	_mav_put_float(buf, 16, z_att);
	_mav_put_float(buf, 20, x_rep);
	_mav_put_float(buf, 24, y_rep);
	_mav_put_float(buf, 28, z_rep);
	_mav_put_float(buf, 32, x_norm);
	_mav_put_float(buf, 36, y_norm);
	_mav_put_float(buf, 40, z_norm);
	_mav_put_uint8_t(buf, 44, coordinate_frame);
	_mav_put_uint8_t(buf, 45, regime_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 46);
#else
	mavlink_ff_pf_field_t packet;
	packet.time_usec = time_usec;
	packet.x_att = x_att;
	packet.y_att = y_att;
	packet.z_att = z_att;
	packet.x_rep = x_rep;
	packet.y_rep = y_rep;
	packet.z_rep = z_rep;
	packet.x_norm = x_norm;
	packet.y_norm = y_norm;
	packet.z_norm = z_norm;
	packet.coordinate_frame = coordinate_frame;
	packet.regime_mask = regime_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 46);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_PF_FIELD;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 46, 114);
}

/**
 * @brief Encode a ff_pf_field struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ff_pf_field C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ff_pf_field_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ff_pf_field_t* ff_pf_field)
{
	return mavlink_msg_ff_pf_field_pack(system_id, component_id, msg, ff_pf_field->time_usec, ff_pf_field->coordinate_frame, ff_pf_field->regime_mask, ff_pf_field->x_att, ff_pf_field->y_att, ff_pf_field->z_att, ff_pf_field->x_rep, ff_pf_field->y_rep, ff_pf_field->z_rep, ff_pf_field->x_norm, ff_pf_field->y_norm, ff_pf_field->z_norm);
}

/**
 * @brief Send a ff_pf_field message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param coordinate_frame Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param regime_mask  1st bit : 0 for far-field, 1 for near-field; 2nd bit : 0 for uncorrected, 1 for corrected
 * @param x_att  X component of the attractive potential gradient
 * @param y_att  Y component of the attractive potential gradient
 * @param z_att  Z component of the attractive potential gradient
 * @param x_rep  X component of the repulsive potential gradient
 * @param y_rep  Y component of the repulsive potential gradient
 * @param z_rep  Z component of the repulsive potential gradient
 * @param x_norm  X component of the normalized total potential gradient
 * @param y_norm  Y component of the normalized total potential gradient
 * @param z_norm  Z component of the normalized total potential gradient
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ff_pf_field_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t coordinate_frame, uint8_t regime_mask, float x_att, float y_att, float z_att, float x_rep, float y_rep, float z_rep, float x_norm, float y_norm, float z_norm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[46];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_float(buf, 8, x_att);
	_mav_put_float(buf, 12, y_att);
	_mav_put_float(buf, 16, z_att);
	_mav_put_float(buf, 20, x_rep);
	_mav_put_float(buf, 24, y_rep);
	_mav_put_float(buf, 28, z_rep);
	_mav_put_float(buf, 32, x_norm);
	_mav_put_float(buf, 36, y_norm);
	_mav_put_float(buf, 40, z_norm);
	_mav_put_uint8_t(buf, 44, coordinate_frame);
	_mav_put_uint8_t(buf, 45, regime_mask);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_PF_FIELD, buf, 46, 114);
#else
	mavlink_ff_pf_field_t packet;
	packet.time_usec = time_usec;
	packet.x_att = x_att;
	packet.y_att = y_att;
	packet.z_att = z_att;
	packet.x_rep = x_rep;
	packet.y_rep = y_rep;
	packet.z_rep = z_rep;
	packet.x_norm = x_norm;
	packet.y_norm = y_norm;
	packet.z_norm = z_norm;
	packet.coordinate_frame = coordinate_frame;
	packet.regime_mask = regime_mask;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_PF_FIELD, (const char *)&packet, 46, 114);
#endif
}

#endif

// MESSAGE FF_PF_FIELD UNPACKING


/**
 * @brief Get field time_usec from ff_pf_field message
 *
 * @return Timestamp for data sync(microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_ff_pf_field_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field coordinate_frame from ff_pf_field message
 *
 * @return Coordinate frame - valid values are MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_ENU,MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 */
static inline uint8_t mavlink_msg_ff_pf_field_get_coordinate_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field regime_mask from ff_pf_field message
 *
 * @return  1st bit : 0 for far-field, 1 for near-field; 2nd bit : 0 for uncorrected, 1 for corrected
 */
static inline uint8_t mavlink_msg_ff_pf_field_get_regime_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field x_att from ff_pf_field message
 *
 * @return  X component of the attractive potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_x_att(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y_att from ff_pf_field message
 *
 * @return  Y component of the attractive potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_y_att(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z_att from ff_pf_field message
 *
 * @return  Z component of the attractive potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_z_att(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field x_rep from ff_pf_field message
 *
 * @return  X component of the repulsive potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_x_rep(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field y_rep from ff_pf_field message
 *
 * @return  Y component of the repulsive potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_y_rep(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field z_rep from ff_pf_field message
 *
 * @return  Z component of the repulsive potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_z_rep(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field x_norm from ff_pf_field message
 *
 * @return  X component of the normalized total potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_x_norm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field y_norm from ff_pf_field message
 *
 * @return  Y component of the normalized total potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_y_norm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field z_norm from ff_pf_field message
 *
 * @return  Z component of the normalized total potential gradient
 */
static inline float mavlink_msg_ff_pf_field_get_z_norm(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a ff_pf_field message into a struct
 *
 * @param msg The message to decode
 * @param ff_pf_field C-struct to decode the message contents into
 */
static inline void mavlink_msg_ff_pf_field_decode(const mavlink_message_t* msg, mavlink_ff_pf_field_t* ff_pf_field)
{
#if MAVLINK_NEED_BYTE_SWAP
	ff_pf_field->time_usec = mavlink_msg_ff_pf_field_get_time_usec(msg);
	ff_pf_field->x_att = mavlink_msg_ff_pf_field_get_x_att(msg);
	ff_pf_field->y_att = mavlink_msg_ff_pf_field_get_y_att(msg);
	ff_pf_field->z_att = mavlink_msg_ff_pf_field_get_z_att(msg);
	ff_pf_field->x_rep = mavlink_msg_ff_pf_field_get_x_rep(msg);
	ff_pf_field->y_rep = mavlink_msg_ff_pf_field_get_y_rep(msg);
	ff_pf_field->z_rep = mavlink_msg_ff_pf_field_get_z_rep(msg);
	ff_pf_field->x_norm = mavlink_msg_ff_pf_field_get_x_norm(msg);
	ff_pf_field->y_norm = mavlink_msg_ff_pf_field_get_y_norm(msg);
	ff_pf_field->z_norm = mavlink_msg_ff_pf_field_get_z_norm(msg);
	ff_pf_field->coordinate_frame = mavlink_msg_ff_pf_field_get_coordinate_frame(msg);
	ff_pf_field->regime_mask = mavlink_msg_ff_pf_field_get_regime_mask(msg);
#else
	memcpy(ff_pf_field, _MAV_PAYLOAD(msg), 46);
#endif
}
