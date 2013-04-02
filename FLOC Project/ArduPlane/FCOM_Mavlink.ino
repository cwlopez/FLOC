//Sketch to include location broadcast to other flock members
/*Notes on Scope: this sketch is concatenated with other ArduPlane .ino files. 
->Should have access to all global variables functions and instances (variables declared outside the setup or main loop)
->Should also have access to all "static" variables, functions, and instances declared in the ArduPlane .ino project files */

/*Notes on Dependencies:
Arduplane.ino should include
"GCS_MAVLink.h"
"AP_AHRS_DCM.h"
*/
//Making this static means it can only be called by functions inside the ArduPlane .ino project files
#if FORMATION_FLIGHT
static NOINLINE void fcom_status_LEDs(enum XBee_Addresses address_id)
{
	uint8_t api_packet = 0xF1;
	XBeeAddress64 address64;
	switch(address_id)
	{
	case TO_ALL:
		address64 = fcom_broadcast64;
	case TO_GCS:
		address64 = fcom_QGCS64;
	}
	Tx64Request status_LEDs_request64(address64, api_option , &api_packet, 1, frame_id);
	fcom_xbee.send(mavlink_comm_1_port, status_LEDs_request64);
}

static NOINLINE void fcom_send_heartbeat(enum XBee_Addresses address_id)
{
	XBeeAddress64 address64;
	switch(address_id)
	{
	case TO_ALL:
		address64 = fcom_broadcast64;
	case TO_GCS:
		address64 = fcom_QGCS64;
	}
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case FLY_BY_WIRE_C:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
	case FORMATION:
		base_mode = MAV_MODE_FLAG_AUTO_ENABLED;
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                    MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    if (control_mode != MANUAL && control_mode != INITIALISING) {
        // stabiliser of some form is enabled
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    if (g.stick_mixing && control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (control_mode != INITIALISING) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	mavlink_heartbeat_t packet;
	packet.custom_mode = custom_mode;
	packet.type = MAV_TYPE_FIXED_WING;
	packet.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
	packet.base_mode = base_mode;
	packet.system_status = system_status;
	packet.mavlink_version = 3;

	uint16_t checksum;
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(fcom_chan);
	
	xbee_api_heartbeat_t api_packet;
	api_packet.mavlink_stx = MAVLINK_STX;
	api_packet.length = MAVLINK_MSG_ID_HEARTBEAT_LEN;
	api_packet.tx_seq = status->current_tx_seq;
	api_packet.sysid = mavlink_system.sysid;
	api_packet.compid = mavlink_system.compid;
	api_packet.msgid = MAVLINK_MSG_ID_HEARTBEAT;
	api_packet.heartbeat = packet;
	
	status->current_tx_seq++;

	checksum = crc_calculate((uint8_t*)&api_packet.length, MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, (const char*)&packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(MAVLINK_HEARTBEAT_CRC, &checksum);
#endif
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	api_packet.ck0 = ck[0];
	api_packet.ck1 = ck[1];

	Tx64Request heartbeat_request64(address64, api_option , (uint8_t*)&api_packet, MAVLINK_HEARTBEAT_LEN, frame_id);
	fcom_xbee.send(mavlink_comm_1_port, heartbeat_request64);
}




static NOINLINE void fcom_send_location(enum XBee_Addresses address_id)
{
	XBeeAddress64 address64;
	switch(address_id)
	{
	case TO_ALL:
		address64 = fcom_broadcast64;
	case TO_GCS:
		address64 = fcom_QGCS64;
	}


	int32_t rel_alt;
#if FF_BARO_ALT == ENABLED
	rel_alt = read_barometer() ; //Use only the altimeter for relative altitude measurement
#else
	rel_alt = (current_loc.alt-home.alt); //Use the standard relative altitude measurement with corrected GPS and home altitude
#endif

	//Get the current state of the a/c flock member:
	//get the DCM rotation matrix
	Matrix3f rot = ahrs.get_dcm_matrix(); // neglecting angle of attack for now
	//initialize a packet for a "mavlink_global_position_int_t" message
	mavlink_global_position_int_t packet;
	//load packet with current state of the a/c flock member
	packet.time_boot_ms = millis();
	//If lock disappears, dead-reckoning should take over and continue to provide estimates for lat, lng
	packet.lat = current_loc.lat; 
	packet.lon = current_loc.lng;
	packet.alt = current_loc.alt; //[cm]
	packet.relative_alt = rel_alt; //[cm]
	packet.vx = g_gps->ground_speed * rot.a.x; //N
	packet.vy = g_gps->ground_speed * rot.b.x; //E
	packet.vz = g_gps->ground_speed * rot.c.x; //D
	packet.hdg = ahrs.yaw_sensor;

	uint16_t checksum;
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(fcom_chan);
	
	xbee_api_global_position_int_t api_packet;
	api_packet.mavlink_stx = MAVLINK_STX;
	api_packet.length = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN;
	api_packet.tx_seq = status->current_tx_seq;
	api_packet.sysid = mavlink_system.sysid;
	api_packet.compid = mavlink_system.compid;
	api_packet.msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	api_packet.loc_packet = packet;
	
	status->current_tx_seq++;

	checksum = crc_calculate((uint8_t*)&api_packet.length, MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, (const char*)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(MAVLINK_GLOBAL_POSITION_INT_CRC, &checksum);
#endif
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	api_packet.ck0 = ck[0];
	api_packet.ck1 = ck[1];

	Tx64Request location_request64(address64, api_option , (uint8_t*)&api_packet, MAVLINK_GLOBAL_POSITION_INT_LEN, frame_id);
	fcom_xbee.send(mavlink_comm_1_port, location_request64);	
}

static NOINLINE void fcom_send_flock_status(enum XBee_Addresses address_id)
{
	XBeeAddress64 address64;
	switch(address_id)
	{
	case TO_ALL:
		address64 = fcom_broadcast64;
	case TO_GCS:
		address64 = fcom_QGCS64;
	}

	uint8_t num_members = ac_flockmember.get_members_iv();
	uint32_t memberid_mask;
	//construct the bitmask for the memberids- only if it shows that there are members
	if(num_members)
	{
		memberid_mask = *ac_flockmember.get_membermask();
	}

	mavlink_ff_flock_status_t packet;
	packet.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet.leader = ac_flockmember.get_local_leader();
	packet.members_iv = num_members;
	packet.member_ids = memberid_mask;
	packet.dist_to_goal = *ac_flockmember.get_D2Goal(); //Distance from goal (includes loiter rotations)

	uint16_t checksum;
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(fcom_chan);
	
	xbee_api_ff_flock_status_t api_packet;
	api_packet.mavlink_stx = MAVLINK_STX;
	api_packet.length = MAVLINK_MSG_ID_FF_FLOCK_STATUS_LEN;
	api_packet.tx_seq = status->current_tx_seq;
	api_packet.sysid = mavlink_system.sysid;
	api_packet.compid = mavlink_system.compid;
	api_packet.msgid = MAVLINK_MSG_ID_FF_FLOCK_STATUS;
	api_packet.ff_packet = packet;
	
	status->current_tx_seq++;

	checksum = crc_calculate((uint8_t*)&api_packet.length, MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, (const char*)&packet, MAVLINK_MSG_ID_FF_FLOCK_STATUS_LEN);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(MAVLINK_FF_FLOCK_STATUS_CRC, &checksum);
#endif
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	api_packet.ck0 = ck[0];
	api_packet.ck1 = ck[1];

	Tx64Request ff_request64(address64, api_option , (uint8_t*)&api_packet, MAVLINK_FF_FLOCK_STATUS_LEN, frame_id);
	fcom_xbee.send(mavlink_comm_1_port, ff_request64);	
}

static NOINLINE void fcom_send_pf_field(enum XBee_Addresses address_id)
{
	XBeeAddress64 address64;
	switch(address_id)
	{
	case TO_ALL:
		address64 = fcom_broadcast64;
	case TO_GCS:
		address64 = fcom_QGCS64;
	}

	Vector3f pfg_att = *ac_pf_field.get_pfg_att();
	Vector3f pfg_rep = *ac_pf_field.get_pfg_rep();
	Vector3f pfg_norm = *ac_pf_field.get_pfg_norm();

	mavlink_ff_pf_field_t packet;
	packet.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet.coordinate_frame = MAV_FRAME_LOCAL_NED;
	packet.x_att = pfg_att.x;
	packet.y_att = pfg_att.y;
	packet.z_att = pfg_att.z;
	packet.x_rep = pfg_rep.x;
	packet.y_rep = pfg_rep.y;
	packet.z_rep = pfg_rep.z;
	packet.x_norm = pfg_norm.x;
	packet.y_norm = pfg_norm.y;
	packet.z_norm = pfg_norm.z;
	packet.regime_mask = ac_pf_field.get_regime_mask();

	uint16_t checksum;
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(fcom_chan);
	
	xbee_api_ff_pf_field_t api_packet;
	api_packet.mavlink_stx = MAVLINK_STX;
	api_packet.length = MAVLINK_MSG_ID_FF_PF_FIELD_LEN;
	api_packet.tx_seq = status->current_tx_seq;
	api_packet.sysid = mavlink_system.sysid;
	api_packet.compid = mavlink_system.compid;
	api_packet.msgid = MAVLINK_MSG_ID_FF_PF_FIELD;
	api_packet.ff_packet = packet;
	
	status->current_tx_seq++;

	checksum = crc_calculate((uint8_t*)&api_packet.length, MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, (const char*)&packet, MAVLINK_MSG_ID_FF_PF_FIELD_LEN);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(MAVLINK_FF_PF_FIELD_CRC, &checksum);
#endif
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	api_packet.ck0 = ck[0];
	api_packet.ck1 = ck[1];

	Tx64Request ff_request64(address64, api_option , (uint8_t*)&api_packet, MAVLINK_FF_PF_FIELD_LEN, frame_id);
	fcom_xbee.send(mavlink_comm_1_port, ff_request64);	
}

static NOINLINE void fcom_send_vwp(enum XBee_Addresses address_id)
{
	XBeeAddress64 address64;
	switch(address_id)
	{
	case TO_ALL:
		address64 = fcom_broadcast64;
	case TO_GCS:
		address64 = fcom_QGCS64;
	}

	const Location* p_VWP = ac_pf_field.get_VWP();
	const uint16_t* p_speed_com = ac_pf_field.get_new_speed();

	mavlink_ff_vwp_t packet;
	packet.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet.coordinate_frame = MAV_FRAME_LOCAL_NED;
	packet.lat= p_VWP->lat;
	packet.lon= p_VWP->lng;
	packet.alt= p_VWP->alt;
	packet.spd= *p_speed_com;

	uint16_t checksum;
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(fcom_chan);
	
	xbee_api_ff_vwp_t api_packet;
	api_packet.mavlink_stx = MAVLINK_STX;
	api_packet.length = MAVLINK_MSG_ID_FF_VWP_LEN;
	api_packet.tx_seq = status->current_tx_seq;
	api_packet.sysid = mavlink_system.sysid;
	api_packet.compid = mavlink_system.compid;
	api_packet.msgid = MAVLINK_MSG_ID_FF_VWP;
	api_packet.ff_packet = packet;
	
	status->current_tx_seq++;

	checksum = crc_calculate((uint8_t*)&api_packet.length, MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, (const char*)&packet, MAVLINK_MSG_ID_FF_VWP_LEN);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(MAVLINK_FF_VWP_CRC, &checksum);
#endif
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	api_packet.ck0 = ck[0];
	api_packet.ck1 = ck[1];

	Tx64Request ff_request64(address64, api_option , (uint8_t*)&api_packet, MAVLINK_FF_VWP_LEN, frame_id);
	fcom_xbee.send(mavlink_comm_1_port, ff_request64);	
}

static NOINLINE void fcom_send_rel_state(enum XBee_Addresses address_id)
{
	XBeeAddress64 address64;
	switch(address_id)
	{
	case TO_ALL:
		address64 = fcom_broadcast64;
	case TO_GCS:
		address64 = fcom_QGCS64;
	}

	const Relative* p_ac_rel = ac_flockmember.get_rel();

	mavlink_ff_rel_state_t packet;
	packet.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet.coordinate_frame = MAV_FRAME_LOCAL_NED;
	packet.dx = p_ac_rel->dXL/100.0;
	packet.dy = p_ac_rel->dYL/100.0;
	packet.dz = p_ac_rel->dZL/100.0;
	packet.dvx = p_ac_rel->dVXL/100.0;
	packet.dvy = p_ac_rel->dVYL/100.0;
	packet.dvz = p_ac_rel->dVZL/100.0;

	uint16_t checksum;
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(fcom_chan);
	
	xbee_api_ff_rel_state_t api_packet;
	api_packet.mavlink_stx = MAVLINK_STX;
	api_packet.length = MAVLINK_MSG_ID_FF_REL_STATE_LEN;
	api_packet.tx_seq = status->current_tx_seq;
	api_packet.sysid = mavlink_system.sysid;
	api_packet.compid = mavlink_system.compid;
	api_packet.msgid = MAVLINK_MSG_ID_FF_REL_STATE;
	api_packet.ff_packet = packet;
	
	status->current_tx_seq++;

	checksum = crc_calculate((uint8_t*)&api_packet.length, MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, (const char*)&packet, MAVLINK_MSG_ID_FF_REL_STATE_LEN);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(MAVLINK_FF_REL_STATE_CRC, &checksum);
#endif
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	api_packet.ck0 = ck[0];
	api_packet.ck1 = ck[1];

	Tx64Request ff_request64(address64, api_option , (uint8_t*)&api_packet, MAVLINK_FF_REL_STATE_LEN, frame_id);
	fcom_xbee.send(mavlink_comm_1_port, ff_request64);	
}
/*
static NOINLINE void fcom_send_gps_error_assist(enum XBee_Addresses address_id)
{
	XBeeAddress64 address64;
	switch(address_id)
	{
	case TO_ALL:
		address64 = fcom_broadcast64;
	case TO_GCS:
		address64 = fcom_QGCS64;
	}

	uint8_t gps_fix;
	if(g_gps->fix) gps_fix = 1;
	else gps_fix = 0;

	mavlink_ff_gps_error_assist_t packet;
	packet.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet.fix_type = gps_fix;
	packet.alt_rel = g_gps->altitude - home.alt;
	packet.eph = g_gps->hdop;
	packet.satellites_visible = g_gps->num_sats;

	uint16_t checksum;
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(fcom_chan);
	
	xbee_api_ff_gps_error_assist_t api_packet;
	api_packet.mavlink_stx = MAVLINK_STX;
	api_packet.length = MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST_LEN;
	api_packet.tx_seq = status->current_tx_seq;
	api_packet.sysid = mavlink_system.sysid;
	api_packet.compid = mavlink_system.compid;
	api_packet.msgid = MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST;
	api_packet.ff_packet = packet;
	
	status->current_tx_seq++;

	checksum = crc_calculate((uint8_t*)&api_packet.length, MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, (const char*)&packet, MAVLINK_MSG_ID_FF_GPS_ERROR_ASSIST_LEN);
#if MAVLINK_CRC_EXTRA
	crc_accumulate(MAVLINK_FF_GPS_ERROR_ASSIST_CRC, &checksum);
#endif
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	api_packet.ck0 = ck[0];
	api_packet.ck1 = ck[1];

	Tx64Request ff_request64(address64, api_option , (uint8_t*)&api_packet, MAVLINK_FF_GPS_ERROR_ASSIST_LEN, frame_id);
	fcom_xbee.send(mavlink_comm_1_port, ff_request64);	
}
*/
/*
static NOINLINE void telcom_send_flock_observe_1HZ()
{
	mavlink_channel_t chan = MAVLINK_COMM_0;

	uint8_t num_members = ac_flockmember.get_members_iv();
	uint32_t memberid_mask;
	//construct the bitmask for the memberids- only if it shows that there are members
	if(num_members)
	{
		memberid_mask = *ac_flockmember.get_membermask();
	}

	mavlink_ff_flock_status_t packet;
	packet.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet.leader = ac_flockmember.get_local_leader();
	packet.members_iv = num_members;
	packet.member_ids = memberid_mask;
	packet.dist_to_goal = *ac_flockmember.get_D2Goal(); //Distance from goal (includes loiter rotations)

	mavlink_msg_ff_flock_status_send(chan,packet.time_usec,packet.leader,packet.members_iv,packet.member_ids,packet.dist_to_goal); 

	const Location* p_VWP = ac_pf_field.get_VWP();
	const uint16_t* p_speed_com = ac_pf_field.get_new_speed();

	mavlink_ff_vwp_t packet0;
	packet0.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet0.coordinate_frame = MAV_FRAME_LOCAL_NED;
	packet0.lat= p_VWP->lat;
	packet0.lon= p_VWP->lng;
	packet0.alt= p_VWP->alt;
	packet0.spd= *p_speed_com;

	mavlink_msg_ff_vwp_send(chan, packet0.time_usec, packet0.coordinate_frame, packet0.lat, packet0.lon,packet0.alt,packet0.spd);

	
	Vector3f pfg_att = *ac_pf_field.get_pfg_att();
	Vector3f pfg_rep = *ac_pf_field.get_pfg_rep();
	Vector3f pfg_norm = *ac_pf_field.get_pfg_norm();

	mavlink_ff_pf_field_t packet1;
	packet1.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet1.coordinate_frame = MAV_FRAME_LOCAL_NED;
	packet1.x_att = pfg_att.x;
	packet1.y_att = pfg_att.y;
	packet1.z_att = pfg_att.z;
	packet1.x_rep = pfg_rep.x;
	packet1.y_rep = pfg_rep.y;
	packet1.z_rep = pfg_rep.z;
	packet1.x_norm = pfg_norm.x;
	packet1.y_norm = pfg_norm.y;
	packet1.z_norm = pfg_norm.z;
	packet1.regime_mask = ac_pf_field.get_regime_mask();

	mavlink_msg_ff_pf_field_send(chan,packet1.time_usec,packet1.coordinate_frame,packet1.regime_mask, packet1.x_att,packet1.y_att,packet1.z_att,
		packet1.x_rep, packet1.y_rep,packet1.z_rep,packet1.x_norm,packet1.y_norm, packet1.z_norm);

	const Relative* p_ac_rel = ac_flockmember.get_rel();

	mavlink_ff_rel_state_t packet2;
	packet2.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet2.coordinate_frame = MAV_FRAME_LOCAL_NED;
	packet2.dx = p_ac_rel->dXL;
	packet2.dy = p_ac_rel->dYL;
	packet2.dz = p_ac_rel->dZL;
	packet2.dvx = p_ac_rel->dVXL;
	packet2.dvy = p_ac_rel->dVYL;
	packet2.dvz = p_ac_rel->dVZL;

	mavlink_msg_ff_rel_state_send(chan,packet2.time_usec, packet2.coordinate_frame, packet2.dx, packet2.dy, packet2.dz, packet2.dvx, packet2.dvy, packet2.dvz);
	
	uint8_t gps_fix;
	if(g_gps->fix) gps_fix = 1;
	else gps_fix = 0;

	mavlink_ff_gps_error_assist_t packet3;
	packet3.time_usec = g_gps->last_fix_time*(uint64_t)1000;
	packet3.fix_type = gps_fix;
	packet3.alt_rel = g_gps->altitude - home.alt;
	packet3.eph = g_gps->hdop;
	packet3.satellites_visible = g_gps->num_sats;

	mavlink_msg_ff_gps_error_assist_send(chan, packet3.time_usec, packet3.fix_type, packet3.eph, packet3.satellites_visible, packet3.alt_rel);
}
*/
//Process Incoming FCOM messages
//These messages are re-routed from GCS_MAVLINK.pde (for standard parsing, etc)

static void handle_fcom_sender(mavlink_message_t* msg)
{
	flock_member* p_flockmember;
	bool* p_rollcall;
	switch(msg->sysid)
	{
#if(MAV_SYSTEM_ID!=HUEY_ID)
	case HUEY_ID:
		p_flockmember = &huey;
		p_rollcall = &ac_rollcall.Huey;
		handle_fcom_message(p_flockmember,p_rollcall,msg);
		return;
#endif
#if(MAV_SYSTEM_ID!=DEWEY_ID)
	case DEWEY_ID:
		p_flockmember = &dewey;
		p_rollcall = &ac_rollcall.Dewey;
		handle_fcom_message(p_flockmember,p_rollcall,msg);
		return;
#endif
#if(MAV_SYSTEM_ID!=LOUIE_ID)
	case LOUIE_ID:
		p_flockmember = &louie;
		p_rollcall = &ac_rollcall.Louie;
		handle_fcom_message(p_flockmember,p_rollcall,msg);
		return;
#endif
	}
}
static void handle_fcom_message(flock_member* p_flockmember, bool* p_rollcall, mavlink_message_t* msg)
{
	switch(msg->msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:
		mavlink_heartbeat_t heartbeat_packet;
		mavlink_msg_heartbeat_decode(msg, &heartbeat_packet);
		process_flockmember_heartbeat(msg->sysid,p_flockmember,p_rollcall,&heartbeat_packet);
		return;
	case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		mavlink_global_position_int_t location_packet;
		mavlink_msg_global_position_int_decode(msg, &location_packet);
		process_flockmember_location(p_flockmember,&location_packet);
		return;
	case MAVLINK_MSG_ID_FF_FLOCK_STATUS:
		mavlink_ff_flock_status_t ff_packet;
		mavlink_msg_ff_flock_status_decode(msg, &ff_packet);
		process_flockmember_status(p_flockmember,&ff_packet);
		return;
	}
}

static void process_flockmember_heartbeat(uint8_t sysid, flock_member* p_flockmember, bool* p_rollcall, mavlink_heartbeat_t* packet)
{
	uint32_t right_now = millis();
	//Just update timestamp if the member is already in view
	//If not, add them
	if(!*p_rollcall)
	{
		//Add the flock member to the list
		ac_flockmember.add_member_in_view(sysid, p_flockmember);
		//Update the last heartbeat received timestamp
		p_flockmember->set_last_heartbeat(&right_now);
		//Set the rollcall to true
		*p_rollcall = true;
	}
	else
	{
		//Update the last heartbeat received timestamp
		p_flockmember->set_last_heartbeat(&right_now);
	}
}

static void process_flockmember_location(flock_member* p_flockmember, mavlink_global_position_int_t* packet)
{
	//Set the state of each member
	p_flockmember->set_state(packet->lat, packet->lon, packet->alt, packet->relative_alt, packet->vx, packet->vy, packet->vz, packet->hdg);
}

static void process_flockmember_status(flock_member* p_flockmember, mavlink_ff_flock_status_t* packet)
{
	//All we care about right now is the distance to the goal (to evaluate global leadership)
	p_flockmember->set_D2Goal(&(packet->dist_to_goal));
}


//
#define MAX_DEFERRED_FF_MESSAGES FF_MSG_RETRY_DEFERRED
static struct fcom_queue {
    enum ff_message deferred_messages[MAX_DEFERRED_FF_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} fcom_queue[2];

// send a message using mavlink
static void fcom_send_message(enum XBee_Addresses address_id, enum ff_message id)
{
	uint8_t i, nextid;
    struct fcom_queue *q = &fcom_queue[(uint8_t)fcom_chan];
    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!fcom_try_send_message(address_id,
                                      q->deferred_messages[q->next_deferred_message])) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_FF_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == FF_MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_FF_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !fcom_try_send_message(address_id, id)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_FF_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_FF_MESSAGES) {
            nextid -= MAX_DEFERRED_FF_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}
// try to send a message, return false if it won't fit in the serial tx buffer
static bool fcom_try_send_message(enum XBee_Addresses address_id, enum ff_message id)
{
    int16_t payload_space = comm_get_txspace(fcom_chan) - XBEE_API_OVERHEAD;
    if (telemetry_delayed(fcom_chan)) {
        return false;
    }

    switch (id) {
    case FF_HEARTBEAT:
        if (payload_space < MAVLINK_HEARTBEAT_LEN)
		{ 
			return false;
		}

        fcom_send_heartbeat(address_id);
        return true;

	case FF_LOCATION:
		if (payload_space < MAVLINK_GLOBAL_POSITION_INT_LEN)
		{ 
			return false;
		}
        fcom_send_location(address_id);
        break;

    case FF_FLOCK_STATUS:
        if (payload_space < MAVLINK_FF_FLOCK_STATUS_LEN)
		{ 
			return false;
		}
        fcom_send_flock_status(address_id);
        break;

    case FF_PF_FIELD:
        if (payload_space < MAVLINK_FF_PF_FIELD_LEN)
		{ 
			return false;
		}
        fcom_send_pf_field(address_id);
        break;

    case FF_VWP:
        if (payload_space < MAVLINK_FF_VWP_LEN)
		{ 
			return false;
		}
        fcom_send_vwp(address_id);
        break;

    case FF_REL_STATE:
        if (payload_space < MAVLINK_FF_REL_STATE_LEN)
		{ 
			return false;
		}
        fcom_send_rel_state(address_id);
        break;
/*	
	case FF_ERROR_ASSIST:
        if (payload_space < MAVLINK_FF_GPS_ERROR_ASSIST_LEN)
		{ 
			return false;
		}
        fcom_send_gps_error_assist(address_id);
        break;
*/
    case FF_MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning
    }
    return true;
}

#endif
