// MESSAGE FF_FLOCK_STATUS PACKING

#define MAVLINK_MSG_ID_FF_FLOCK_STATUS 190

typedef struct __mavlink_ff_flock_status_t
{
 uint64_t time_usec; ///< Timestamp for data sync(microseconds since UNIX epoch)
 uint32_t member_ids; ///< bit-mask for sysids of members in view ( ie ...0011 means sysid 1 and 2 are in view) Supports 32 members.
 int32_t dist_to_goal; ///< distance to global goal (in meters)
 uint8_t leader; ///< sysid of local leader (0 means either Global leader, or not participating in formation) 
 uint8_t members_iv; ///< number of formation members in view of UAV
} mavlink_ff_flock_status_t;

#define MAVLINK_MSG_ID_FF_FLOCK_STATUS_LEN 18
#define MAVLINK_MSG_ID_190_LEN 18



#define MAVLINK_MESSAGE_INFO_FF_FLOCK_STATUS { \
	"FF_FLOCK_STATUS", \
	5, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ff_flock_status_t, time_usec) }, \
         { "member_ids", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_ff_flock_status_t, member_ids) }, \
         { "dist_to_goal", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_ff_flock_status_t, dist_to_goal) }, \
         { "leader", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_ff_flock_status_t, leader) }, \
         { "members_iv", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_ff_flock_status_t, members_iv) }, \
         } \
}


/**
 * @brief Pack a ff_flock_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param leader sysid of local leader (0 means either Global leader, or not participating in formation) 
 * @param members_iv number of formation members in view of UAV
 * @param member_ids bit-mask for sysids of members in view ( ie ...0011 means sysid 1 and 2 are in view) Supports 32 members.
 * @param dist_to_goal distance to global goal (in meters)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_flock_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t time_usec, uint8_t leader, uint8_t members_iv, uint32_t member_ids, int32_t dist_to_goal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint32_t(buf, 8, member_ids);
	_mav_put_int32_t(buf, 12, dist_to_goal);
	_mav_put_uint8_t(buf, 16, leader);
	_mav_put_uint8_t(buf, 17, members_iv);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 18);
#else
	mavlink_ff_flock_status_t packet;
	packet.time_usec = time_usec;
	packet.member_ids = member_ids;
	packet.dist_to_goal = dist_to_goal;
	packet.leader = leader;
	packet.members_iv = members_iv;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_FLOCK_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 18, 6);
}

/**
 * @brief Pack a ff_flock_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param leader sysid of local leader (0 means either Global leader, or not participating in formation) 
 * @param members_iv number of formation members in view of UAV
 * @param member_ids bit-mask for sysids of members in view ( ie ...0011 means sysid 1 and 2 are in view) Supports 32 members.
 * @param dist_to_goal distance to global goal (in meters)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ff_flock_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t time_usec,uint8_t leader,uint8_t members_iv,uint32_t member_ids,int32_t dist_to_goal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint32_t(buf, 8, member_ids);
	_mav_put_int32_t(buf, 12, dist_to_goal);
	_mav_put_uint8_t(buf, 16, leader);
	_mav_put_uint8_t(buf, 17, members_iv);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 18);
#else
	mavlink_ff_flock_status_t packet;
	packet.time_usec = time_usec;
	packet.member_ids = member_ids;
	packet.dist_to_goal = dist_to_goal;
	packet.leader = leader;
	packet.members_iv = members_iv;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 18);
#endif

	msg->msgid = MAVLINK_MSG_ID_FF_FLOCK_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 18, 6);
}

/**
 * @brief Encode a ff_flock_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ff_flock_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ff_flock_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ff_flock_status_t* ff_flock_status)
{
	return mavlink_msg_ff_flock_status_pack(system_id, component_id, msg, ff_flock_status->time_usec, ff_flock_status->leader, ff_flock_status->members_iv, ff_flock_status->member_ids, ff_flock_status->dist_to_goal);
}

/**
 * @brief Send a ff_flock_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec Timestamp for data sync(microseconds since UNIX epoch)
 * @param leader sysid of local leader (0 means either Global leader, or not participating in formation) 
 * @param members_iv number of formation members in view of UAV
 * @param member_ids bit-mask for sysids of members in view ( ie ...0011 means sysid 1 and 2 are in view) Supports 32 members.
 * @param dist_to_goal distance to global goal (in meters)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ff_flock_status_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t leader, uint8_t members_iv, uint32_t member_ids, int32_t dist_to_goal)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[18];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_uint32_t(buf, 8, member_ids);
	_mav_put_int32_t(buf, 12, dist_to_goal);
	_mav_put_uint8_t(buf, 16, leader);
	_mav_put_uint8_t(buf, 17, members_iv);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_FLOCK_STATUS, buf, 18, 6);
#else
	mavlink_ff_flock_status_t packet;
	packet.time_usec = time_usec;
	packet.member_ids = member_ids;
	packet.dist_to_goal = dist_to_goal;
	packet.leader = leader;
	packet.members_iv = members_iv;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FF_FLOCK_STATUS, (const char *)&packet, 18, 6);
#endif
}

#endif

// MESSAGE FF_FLOCK_STATUS UNPACKING


/**
 * @brief Get field time_usec from ff_flock_status message
 *
 * @return Timestamp for data sync(microseconds since UNIX epoch)
 */
static inline uint64_t mavlink_msg_ff_flock_status_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field leader from ff_flock_status message
 *
 * @return sysid of local leader (0 means either Global leader, or not participating in formation) 
 */
static inline uint8_t mavlink_msg_ff_flock_status_get_leader(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field members_iv from ff_flock_status message
 *
 * @return number of formation members in view of UAV
 */
static inline uint8_t mavlink_msg_ff_flock_status_get_members_iv(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field member_ids from ff_flock_status message
 *
 * @return bit-mask for sysids of members in view ( ie ...0011 means sysid 1 and 2 are in view) Supports 32 members.
 */
static inline uint32_t mavlink_msg_ff_flock_status_get_member_ids(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field dist_to_goal from ff_flock_status message
 *
 * @return distance to global goal (in meters)
 */
static inline int32_t mavlink_msg_ff_flock_status_get_dist_to_goal(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Decode a ff_flock_status message into a struct
 *
 * @param msg The message to decode
 * @param ff_flock_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_ff_flock_status_decode(const mavlink_message_t* msg, mavlink_ff_flock_status_t* ff_flock_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	ff_flock_status->time_usec = mavlink_msg_ff_flock_status_get_time_usec(msg);
	ff_flock_status->member_ids = mavlink_msg_ff_flock_status_get_member_ids(msg);
	ff_flock_status->dist_to_goal = mavlink_msg_ff_flock_status_get_dist_to_goal(msg);
	ff_flock_status->leader = mavlink_msg_ff_flock_status_get_leader(msg);
	ff_flock_status->members_iv = mavlink_msg_ff_flock_status_get_members_iv(msg);
#else
	memcpy(ff_flock_status, _MAV_PAYLOAD(msg), 18);
#endif
}
