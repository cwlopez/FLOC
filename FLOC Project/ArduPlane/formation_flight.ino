// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Formation flight high-level functions
//Christian Lopez
//1/18/13

#if FORMATION_FLIGHT
static void broadcast_my_location()
{
	ac_location_broadcast();

}
static void update_ac_flockmember()
{
	Matrix3f rot = ahrs.get_dcm_matrix(); // neglecting angle of attack for now
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = millis();
	packet.lat = current_loc.lat;
	packet.lon = current_loc.lng;
	packet.alt = g_gps->altitude * 10; // [mm]
	packet.relative_alt = (current_loc.alt-home.alt) * 10; //[mm]
	packet.vx = g_gps->ground_speed * rot.a.x; //[cm/s] body
	packet.vy = g_gps->ground_speed * rot.b.x; //[cm/s] body
	packet.vz = g_gps->ground_speed * rot.c.x; //[cm/s] body
	packet.hdg = ahrs.yaw_sensor; // degrees *100

	ac_flockmember.set_state(packet.lat, packet.lon, packet.alt, packet.relative_alt, packet.vx, packet.vy, packet.vz, packet.hdg);
	ac_flockmember.update_rel();
	const Relative* debug_rel = ac_flockmember.get_rel();
	float debug_d2L = debug_rel->d2L;
	float debug_dXL = debug_rel->dXL;
	float debug_dYL = debug_rel->dYL;
	float debug_dZL = debug_rel->dZL;
	bool freff_dafa = true;
	bool debug_relishmustard= true;
	bool debug_relishmayo= true;
	bool debug_relishcatzup= true;
	bool debug_relishbbq= true;
}
static void process_flockmember_location(uint8_t sysid, mavlink_global_position_int_t* packet)
{
	bool dededeeuub = false;
	//Who is the message even coming from?
	switch(sysid){
#if(MAV_SYSTEM_ID!=HUEY_ID)
	case HUEY_ID:
		if(ac_rollcall.Huey)
		{
			huey.set_state(packet->lat, packet->lon, packet->alt, packet->relative_alt, packet->vx, packet->vy, packet->vz, packet->hdg);
		}
		else
		{
			//if(member_in_view(&packet->lat, &packet->lon, &packet->alt))
			//{
				ac_flockmember.add_member_in_view(sysid,&huey);
				ac_rollcall.Huey = true;
				huey.set_state(packet->lat, packet->lon, packet->alt, packet->relative_alt, packet->vx, packet->vy, packet->vz, packet->hdg);
		
			//}
		}
		return;
#endif
#if(MAV_SYSTEM_ID!=DEWEY_ID)
	case DEWEY_ID:
		if(ac_rollcall.Dewey)
		{
			int32_t debug_dewey_lat = packet->lat;
			int32_t debug_dewey_lon = packet->lon;
			bool debugs_guns = false;
			dewey.set_state(packet->lat, packet->lon, packet->alt, packet->relative_alt, packet->vx, packet->vy, packet->vz, packet->hdg);
		}
		else
		{
			//if(member_in_view(&packet->lat, &packet->lon, &packet->alt))
			//{
				ac_flockmember.add_member_in_view(sysid,&dewey);
				ac_rollcall.Dewey = true;
				dewey.set_state(packet->lat, packet->lon, packet->alt, packet->relative_alt, packet->vx, packet->vy, packet->vz, packet->hdg);
			//}
		}
		return;
#endif
#if(MAV_SYSTEM_ID!=LOUIE_ID)
	case LOUIE_ID:
		if(ac_rollcall.Louie)
		{
			louie.set_state(packet->lat, packet->lon, packet->alt, packet->relative_alt, packet->vx, packet->vy, packet->vz, packet->hdg);
		}
		else
		{
			//if(member_in_view(&packet->lat, &packet->lon, &packet->alt))
			//{
				ac_flockmember.add_member_in_view(sysid,&louie);
				ac_rollcall.Louie = true;
				louie.set_state(packet->lat, packet->lon, packet->alt, packet->relative_alt, packet->vx, packet->vy, packet->vz, packet->hdg);
		
			//}
		}
		return;
#endif
	}
}
static void update_formation_flight_commands()
{
	
		F_airspeed_error = *ac_pf_field.get_new_speed();
		// copy the current location into the OldWP slot
		// ---------------------------------------
		prev_WP = current_loc;

		// Load the next_WP slot
		// ---------------------
		next_WP = *ac_pf_field.get_VWP();
		//ac_pf_field.updated = false;

        target_altitude_cm = next_WP.alt;
		V_altitude_error_cm       = target_altitude_cm - current_loc.alt;

		// this is handy for the groundstation
		wp_totalDistance        = get_distance(&current_loc, &next_WP);
		wp_distance             = wp_totalDistance;
		target_bearing_cd       = get_bearing_cd(&current_loc, &next_WP);

		// to check if we have missed the WP
		// ----------------------------
		old_target_bearing_cd = target_bearing_cd;

		// set a new crosstrack bearing
		// ----------------------------
		reset_crosstrack();
}
/*
static void update_flock_leadership(){
	Relative tmp_rel = *ac_flockmember.get_rel();
	switch(MAV_SYSTEM_ID)
	{
	case HUEY_ID:
		ac_flockmember.set_global_leader(true);

	case DEWEY_ID:
		if(tmp_rel.Num_members >0)
		{
			bool huey_check;
			uint8_t tmp_leader;
			for(int i=0; i< tmp_rel.Num_members; i++)
			{
				if(tmp_rel.Member_ids[i] == HUEY_ID)
				{
					tmp_leader = tmp_rel.Member_ids[i];
					huey_check = true;
				}
			}
			if(huey_check)
			{
				ac_flockmember.set_local_leader(tmp_leader);
				ac_flockmember.set_global_leader(false);
			}
			else
			{
				ac_flockmember.set_global_leader(true);
			}
		}
		else
		{
			ac_flockmember.set_global_leader(true);
		}

	case LOUIE_ID:
		if(tmp_rel.Num_members >0)
		{
			bool dewey_check;
			bool huey_check;
			uint8_t tmp_dewey;
			uint8_t tmp_huey;
			for(int i=0; i< tmp_rel.Num_members; i++)
			{
				if(tmp_rel.Member_ids[i] == DEWEY_ID)
				{
					tmp_dewey = tmp_rel.Member_ids[i];
					dewey_check = true;
				}
				else if(tmp_rel.Member_ids[i] == HUEY_ID)
				{
					tmp_huey = tmp_rel.Member_ids[i];
					huey_check = true;
				}
			}
			if(dewey_check)
			{
				ac_flockmember.set_local_leader(tmp_dewey);
				ac_flockmember.set_global_leader(false);
			}
			else if(huey_check)
			{
				ac_flockmember.set_local_leader(tmp_huey);
				ac_flockmember.set_global_leader(false);
			}
			else
			{
				ac_flockmember.set_global_leader(true);
			}
		}
		else
		{
			ac_flockmember.set_global_leader(true);
		}
	}
}
*/

static void update_flock_leadership(){
	Relative tmp_rel = *ac_flockmember.get_rel();
	///////////DEBUG/////////////////////////////
	bool debuggggs = true;
	//////////DEBUG//////////////////////////////
	if(tmp_rel.Num_members >0)
	{
		//////////DEBUG//////////////////////////////
		bool debuggs = true;
		//////////DEBUG//////////////////////////////
		if(tmp_rel.Num_members > 1)
		{
			//because this application is for 3 flock members, a simple linear solution the the nearest neighbor problem is fine. 
			//If this were implemented for a larger flock, more optimal solutions should be used, like a cover tree data structure
			Vector2l tmp_closest;
			uint8_t tmp_leader;
			for(int i = 0;i<tmp_rel.Num_members;i++)
			{
				uint8_t tmp_member1 = tmp_rel.Member_ids[i];
				Vector2l tmp_dist1;
				tmp_dist1.x = tmp_rel.dX[i];
				tmp_dist1.y = tmp_rel.dY[i];
				if(i==0) 
				{
					tmp_closest = tmp_dist1;
					tmp_leader = tmp_rel.Member_ids[i];
				}
				for(int j = i+1; j< tmp_rel.Num_members;j++)
				{
					uint8_t tmp_member2 = tmp_rel.Member_ids[j];
					Vector2l tmp_dist2;
					tmp_dist2.x = tmp_rel.dX[j];
					tmp_dist2.y = tmp_rel.dY[j];
					if(tmp_dist1.length_squared()<tmp_dist2.length_squared() && tmp_dist1.length_squared()<tmp_closest.length_squared())
					{
						tmp_closest = tmp_dist1;
						tmp_leader = tmp_rel.Member_ids[i];
					}
					else break;
				}
			}
			ac_flockmember.set_local_leader(tmp_leader);
			ac_flockmember.set_global_leader(false);
		}
		else
		{
			ac_flockmember.set_local_leader(tmp_rel.Member_ids[0]);
			ac_flockmember.set_global_leader(false);
		}
	}
	else 
	{
		ac_flockmember.set_global_leader(true);
	}
}

static void set_goal_WP(void)
{
    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP = goal_WP;

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    target_altitude_cm = current_loc.alt;
    offset_altitude_cm = next_WP.alt - prev_WP.alt;

    // this is handy for the groundstation
    wp_totalDistance        = get_distance(&current_loc, &next_WP);
    wp_distance             = wp_totalDistance;
    target_bearing_cd       = get_bearing_cd(&current_loc, &next_WP);

    // to check if we have missed the WP
    // ----------------------------
    old_target_bearing_cd = target_bearing_cd;

    // set a new crosstrack bearing
    // ----------------------------
    reset_crosstrack();
}
//member in view function provides a way to filter out members that are not within the pre-defined "view range" of the a/c flock member
static bool member_in_view(int32_t* p_lat, int32_t* p_lon, int32_t* p_alt)
{
	//based on imformation passed in about the flock member in question, a temporary location structure is created to use the AP_Math get_bearing function
	Location tmp_location;
	tmp_location.alt = *p_alt;
	tmp_location.lat = *p_lat;
	tmp_location.lng = *p_lon;
	
	//treating the two location structures as "waypoints" the get_bearing function returns the angle of the flock member wrt the a/c
	int32_t tmp_bearing = get_bearing_cd((const Location*)&current_loc,(const Location*)&tmp_location);
	//the "bearing" between the two members is subtracted from the heading of the a/c flock_member to find the relative bearing from the direction the a/c is pointed
	int32_t tmp_bearing_error = ahrs.yaw_sensor-tmp_bearing; 
	//in order to handle +/-360 degree outputs, the outputs are wrapped and then the absolute value is taken to see if the threshold has been surpassed 
	if (tmp_bearing_error > 18000) tmp_bearing -= 36000;
    if (tmp_bearing_error < -18000) tmp_bearing += 36000;
	//if the flock member in question is outside the threshold range, the function returns false
    if (abs(tmp_bearing_error)>((VIEW_RANGE*100)/2)) return false;
	//otherwise, the function returns true
	else return true;
}
static void check_formation_health()
{
	uint32_t right_now = millis();
#if(MAV_SYSTEM_ID!=HUEY_ID)
		if(ac_rollcall.Huey)
		{
			uint32_t last_contact = *huey.get_last_update_time();
			uint32_t time_out = right_now - last_contact;
			if(time_out>FLOCKMEMBER_TIME_OUT)
			{
				ac_flockmember.remove_member_in_view(HUEY_ID);
				ac_rollcall.Huey = false;				
			}
		}
#endif
#if(MAV_SYSTEM_ID!=DEWEY_ID)
		if(ac_rollcall.Dewey)
		{
			uint32_t last_contact = *dewey.get_last_update_time();
			uint32_t time_out = right_now - last_contact;
			if(time_out>FLOCKMEMBER_TIME_OUT)
			{
				ac_flockmember.remove_member_in_view(DEWEY_ID);
				ac_rollcall.Dewey = false;
			}
		}
#endif
#if(MAV_SYSTEM_ID!=LOUIE_ID)
		if(ac_rollcall.Louie)
		{
			uint32_t last_contact = *louie.get_last_update_time();
			uint32_t time_out = right_now - last_contact;
			if(time_out>FLOCKMEMBER_TIME_OUT)
			{
				ac_flockmember.remove_member_in_view(LOUIE_ID);
				ac_rollcall.Louie = false;
			}
		}
#endif
}
#endif