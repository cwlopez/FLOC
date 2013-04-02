// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//Formation flight high-level functions
//Christian Lopez
//1/18/13

#if FORMATION_FLIGHT

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}



static void update_ac_flockmember()
{
		int32_t rel_alt;
#if FF_BARO_ALT == ENABLED
	rel_alt = read_barometer(); //Use only the altimeter for relative altitude measurement
#else
	rel_alt = (current_loc.alt-home.alt); //Use the standard relative altitude measurement with corrected GPS and home altitude
#endif
	Matrix3f rot = ahrs.get_dcm_matrix(); // neglecting angle of attack for now
	mavlink_global_position_int_t packet;
	packet.time_boot_ms = millis();
	packet.lat = current_loc.lat;
	packet.lon = current_loc.lng;
	packet.alt = current_loc.alt; // [cm]
	packet.relative_alt = rel_alt; //[cm]
	packet.vx = g_gps->ground_speed * rot.a.x; //[cm/s] N
	packet.vy = g_gps->ground_speed * rot.b.x; //[cm/s] E
	packet.vz = g_gps->ground_speed * rot.c.x; //[cm/s] D
	packet.hdg = ahrs.yaw_sensor; // degrees *100

	ac_flockmember.set_state(packet.lat, packet.lon, packet.alt, packet.relative_alt, packet.vx, packet.vy, packet.vz, packet.hdg);
	ac_flockmember.update_rel();

	//Update the memberid bitmask
	uint32_t membermask = *ac_flockmember.get_membermask();
	uint32_t memberid_mask;
	if(ac_rollcall.Huey)
		memberid_mask = (1<<0);
	if(ac_rollcall.Dewey)
		memberid_mask = (1<<1);
	if(ac_rollcall.Louie)
		memberid_mask = (1<<2);

	membermask = membermask | memberid_mask;
	ac_flockmember.set_membermask(&membermask);

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

static void update_flock_leadership()
{
	//get relative state structure for ac
	Relative tmp_rel = *ac_flockmember.get_rel();
	//You only want to check this if there is at least one more member
	if(tmp_rel.Num_members >0)
	{
		//because this application is for 3 flock members, a simple linear solution the the nearest neighbor problem is fine. 
		//If this were implemented for a larger flock, more optimal solutions should be used, like a cover tree data structure
		int closer_to_goal[FLOCK_SIZE-1];
		Vector2f tmp_closest;
		uint8_t tmp_leader;
		int cntr = 0;
		for(int i = 0;i<tmp_rel.Num_members;i++)
		{

			int j = tmp_rel.Member_ids[i]-1;	//subtract 1 to start at offset ids (we start at 0 in C)
			flock_member tmp_flock_member = *ac_flockmember.get_member_pntr(j);

			if(*ac_flockmember.get_D2Goal()>*tmp_flock_member.get_D2Goal())
			{
				closer_to_goal[cntr]=i;
				cntr++;
			}
		}
		if(cntr == 0)
		{
			ac_flockmember.set_global_leader(true);
			return;
		}
		else if(cntr == 1)
		{
			tmp_leader = tmp_rel.Member_ids[closer_to_goal[0]];
			ac_flockmember.set_local_leader(tmp_leader);
			ac_flockmember.set_global_leader(false);
		}
		else
		{
			for(int k = 0; k<cntr; k++)
			{
				int l = closer_to_goal[k];
				uint8_t tmp_member1 = tmp_rel.Member_ids[l];
				Vector2f tmp_dist1;
				tmp_dist1.x = tmp_rel.dX[l]/100.0;
				tmp_dist1.y = tmp_rel.dY[l]/100.0;
				if(k==0) 
				{
					tmp_closest = tmp_dist1;
					tmp_leader = tmp_rel.Member_ids[l];
				}
				for(int m = k+1; m< tmp_rel.Num_members;m++)
				{
					int n = closer_to_goal[m];
					uint8_t tmp_member2 = tmp_rel.Member_ids[n];
					Vector2f tmp_dist2;
					tmp_dist2.x = tmp_rel.dX[n]/100.0;
					tmp_dist2.y = tmp_rel.dY[n]/100.0;
					if(tmp_dist1.length_squared()<tmp_dist2.length_squared() && tmp_dist1.length_squared()<tmp_closest.length_squared())
					{
						tmp_closest = tmp_dist1;
						tmp_leader = tmp_rel.Member_ids[l];
					}
				}
			}
			ac_flockmember.set_local_leader(tmp_leader);
			ac_flockmember.set_global_leader(false);
		}
	}
}

static void update_flock_side()
{
	//if ac is global leader, do nothing
	if(ac_flockmember.get_leader_status())
	{
		return;
	}
	else
	{
		Relative tmp_rel = *ac_flockmember.get_rel();
		//You only want to check this if there is at least one more member
		if(tmp_rel.Num_members >0)
		{
			//find out who is closer to the goal (aka "in front" of you)
			int closer_to_goal[FLOCK_SIZE-1];
			int cntr;
			for(int i = 0;i<tmp_rel.Num_members;i++)
			{
				int j = tmp_rel.Member_ids[i]-1;	//subtract 1 to start at offset ids (we start at 0 in C)
				flock_member tmp_flock_member = *ac_flockmember.get_member_pntr(j);

				if(*ac_flockmember.get_D2Goal()>*tmp_flock_member.get_D2Goal())
				{
					closer_to_goal[cntr]=i;
					cntr++;
				}
			}
			//if there is only one member "in front", base side on approach
			if(cntr == 1)
			{
				int32_t approach_l = -(tmp_rel.dXL)*sin(ToRad((*ac_flockmember.get_hdg())/100.0)) 
									+(tmp_rel.dYL)*cos(ToRad((*ac_flockmember.get_hdg())/100.0));
				if(approach_l<0)
					ac_pf_field.set_side(-1);
				else if (approach_l>0)
					ac_pf_field.set_side(1);
				else
					ac_pf_field.set_side(-1); //default to right side
			}
			//if there is more than one member "in front", base side on flock center
			else
			{
				//find the flock center in NED coordinates
				float dX_sum = 0;
				float dY_sum = 0;
				for(int k = 0; k<cntr; k++)
				{
					int m =closer_to_goal[k]; //subtract 1 to start at offset ids (we start at 0 in C) 
					dX_sum +=(tmp_rel.dX[m]/100.0);
					dY_sum +=(tmp_rel.dY[m]/100.0);
				}
				float flock_center_dX = dX_sum/(cntr-1);
				float flock_center_dY = dY_sum/(cntr-1);
				//translate coordinates into the Leader's local frame
				float flock_center_l = - flock_center_dX*sin(ToRad(tmp_rel.hdgL/100.0)) + flock_center_dY*cos(ToRad(tmp_rel.hdgL/100.0));
				//base side selection on y coordinate of the flock center in the Leader's local frame
				if(flock_center_l<0)
				{
					ac_pf_field.set_side(-1);
				}
				else if (flock_center_l>0)
				{
					ac_pf_field.set_side(1);
				}
				else //chose side based on approach (this is just to be safe in case of a miraculous alignment)
				{
					int32_t approach_l = -(tmp_rel.dXL)*sin(ToRad((*ac_flockmember.get_hdg())/100.0)) 
										+(tmp_rel.dYL)*cos(ToRad((*ac_flockmember.get_hdg())/100.0));
					if(approach_l<0)
						ac_pf_field.set_side(-1);
					else if (approach_l>0)
						ac_pf_field.set_side(1);
					else
						ac_pf_field.set_side(-1); //default to right side

				}//end if(flock_center_l>0)
			}//end if(cntr ==1)
		}//end if(tmp_rel.Num_members>0)
	}//end if(ac_flockmember.get_leader_status())
}//end update_flock_side()


static void set_goal_WP()
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

	nav_command_ID  = NO_COMMAND;
    non_nav_command_ID      = NO_COMMAND;
    // set a new crosstrack bearing
    // ----------------------------
    reset_crosstrack();
}

static void update_goal_wp_distance()
{
	goal_wp_distance = get_distance(&current_loc, &goal_WP);
}

static void update_goal_loiter()
{
    float power;

    if(goal_wp_distance <= GOAL_LOITER_R) {
        power = float(goal_wp_distance) / float(GOAL_LOITER_R);
        power = constrain(power, 0.5, 1);
        nav_bearing_cd += 9000.0 * (2.0 + power);
    } else if(goal_wp_distance < (GOAL_LOITER_R + LOITER_RANGE)) {
        power = -((float)(goal_wp_distance - GOAL_LOITER_R - GOAL_RANGE) / GOAL_RANGE);
        power = constrain(power, 0.5, 1);                               //power = constrain(power, 0, 1);
        nav_bearing_cd -= power * 9000;
    } else{
        update_crosstrack();

    }
    nav_bearing_cd = wrap_360_cd(nav_bearing_cd);
}

static void update_distance_to_goal()
{
    // wrap values
    if (loiter_delta > 180) loiter_delta -= 360;
    if (loiter_delta < -180) loiter_delta += 360;
	if(goal_wp_distance<(GOAL_LOITER_R+GOAL_MARGIN))
	{
		goal_loiter_sum += abs(loiter_delta);
	}
	goal_unwind = (2*GOAL_LOITER_R*PI*goal_loiter_sum/360)*100; //distance traveled around the goal (arc-length) in m
	int32_t d2goal = goal_wp_distance + (GOAL_WINDUP-goal_unwind)/100;
	ac_flockmember.set_D2Goal(&d2goal);
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
	/*
	This function is meant to manage the removal of a flock member from the UAV's 
	view if too much time has passed since receiving  a heartbeat packet, usually 
	signifying that the flock member has either left the formation or is simply 
	no longer broadcasting.
	*/
	uint32_t right_now = millis();
#if(MAV_SYSTEM_ID!=HUEY_ID)
		if(ac_rollcall.Huey)
		{
			uint32_t last_heartbeat = *huey.get_last_heartbeat();
			uint32_t time_out = right_now - last_heartbeat;
			if(time_out>FLOCKMEMBER_TIME_OUT)
			{
				ac_flockmember.remove_member_in_view(HUEY_ID);
				//Check to see if this was your leader
				if(ac_flockmember.get_local_leader() == HUEY_ID)
				{
					//if it was, set yourself as the global leader, and set the goal WP to be the next WP
					ac_flockmember.set_global_leader(true);
					set_goal_WP();
				}
				ac_rollcall.Huey = false;				
			}
		}
#endif
#if(MAV_SYSTEM_ID!=DEWEY_ID)
		if(ac_rollcall.Dewey)
		{
			uint32_t last_heartbeat = *dewey.get_last_heartbeat();
			uint32_t time_out = right_now - last_heartbeat;
			if(time_out>FLOCKMEMBER_TIME_OUT)
			{
				ac_flockmember.remove_member_in_view(DEWEY_ID);
				//Check to see if this was your leader
				if(ac_flockmember.get_local_leader() == DEWEY_ID)
				{
					//if it was, set yourself as the global leader, and set the goal WP to be the next WP
					ac_flockmember.set_global_leader(true);
					set_goal_WP();
				}
				ac_rollcall.Dewey = false;
			}
		}
#endif
#if(MAV_SYSTEM_ID!=LOUIE_ID)
		if(ac_rollcall.Louie)
		{
			uint32_t last_heartbeat = *louie.get_last_heartbeat();
			uint32_t time_out = right_now - last_heartbeat;
			if(time_out>FLOCKMEMBER_TIME_OUT)
			{
				ac_flockmember.remove_member_in_view(LOUIE_ID);
				//Check to see if this was your leader
				if(ac_flockmember.get_local_leader() == LOUIE_ID)
				{
					//if it was, set yourself as the global leader, and set the goal WP to be the next WP
					ac_flockmember.set_global_leader(true);
					set_goal_WP();
				}
				ac_rollcall.Louie = false;
			}
		}
#endif

}
#endif