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
//#if FORMATION_FLIGHT_ENABLED
void ac_location_broadcast()
{
	//Get the current state of the a/c flock member:
	//get the DCM rotation matrix
	Matrix3f rot = ahrs.get_dcm_matrix(); // neglecting angle of attack for now
	//initialize a packet for a "mavlink_global_position_int_t" message
	mavlink_global_position_int_t packet;
	//load packet with current state of the a/c flock member
	packet.time_boot_ms = millis();
	packet.lat = current_loc.lat;
	packet.lon = current_loc.lng;
	packet.alt = g_gps->altitude * 10;
	packet.relative_alt = (current_loc.alt-home.alt) * 10;
	packet.vx = g_gps->ground_speed * rot.a.x;
	packet.vy = g_gps->ground_speed * rot.b.x;
	packet.vz = g_gps->ground_speed * rot.c.x;
	packet.hdg = ahrs.yaw_sensor;

	uint8_t* locpacket = (uint8_t*)&packet;
	//information for the MAVLink message
	uint8_t msgid	= MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	uint8_t length	= 28;
	uint8_t crc_extra = 104;

	//initialize variables for MAVLink message
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_1);
	//fill MAVLink header buffer
	buf[0] = MAVLINK_STX;
	buf[1] = length;
	buf[2] = status->current_tx_seq;
	buf[3] = mavlink_system.sysid;
	buf[4] = mavlink_system.compid;
	buf[5] = msgid;
	status->current_tx_seq++;
	//calculate checksum for MAVLink header and packet
	checksum = crc_calculate((uint8_t*)&buf[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, (const char*)locpacket, length);
	crc_accumulate(crc_extra, &checksum);
	ck[0] = (uint8_t)(checksum & 0xFF);
	ck[1] = (uint8_t)(checksum >> 8);

	uint32_t msb = 0;
	uint32_t lsb = BROADCAST_ADDRESS;
	uint8_t api_option = DISABLE_ACK_OPTION;
	uint8_t frame_id = NO_RESPONSE_FRAME_ID;
	XBeeAddress64  gcs_xbee64(msb,lsb);
	uint8_t mavlink_packet[MAX_FRAME_DATA_SIZE];
	uint8_t mavlink_packet_length;
	uint8_t index =0;
	for(int i = 0; i < MAVLINK_NUM_HEADER_BYTES; i++)
	{
		mavlink_packet[index] = buf[i];
		index++;
	}
	for(int i = 0; i < length; i++)
	{
		mavlink_packet[index] = locpacket[i];
		index++;
	}
	for(int i =0; i < 2; i++)
	{
		mavlink_packet[index] = ck[i];
		index++;
	}
	mavlink_packet_length = index;
	Tx64Request mavlink_request64(gcs_xbee64, api_option , mavlink_packet, mavlink_packet_length, frame_id);
	ac_xbee.send(mavlink_comm_1_port, mavlink_request64);
}
//#endif
