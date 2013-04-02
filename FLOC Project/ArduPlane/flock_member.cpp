// 
// 
// 

#include "flock_member.h"
#include <GCS_MAVLink.h>


flock_member::flock_member()	//Constructor for this a/c
{
	//upon construction of a/c flock member, assign it the MAV_SYSTEM_ID given in the APM_Config file
	_sysid = MAV_SYSTEM_ID;
	//initialize the check for global leadership as false (this has yet to be determined by the swarming algorithm)
	_global_leader = true;
	//initialize the number of flock members as 0 (they have yet to be added to the flock)
	_num_members = 0;

	//initialize the update checks as false... the a/c has not been updated yet
	state_updated = false;
	rel_updated = false;
//----------------------------------------
}

flock_member::flock_member(uint8_t sysid) //constructor for a different flock member
{
	//given the sysid as an input, assign it to the flock member's private variable
	_sysid = sysid;
	//initialize the update check of the flock member as false... it hasn't been updated yet 
	state_updated = false;

}

//Sets the private state variables of a flock member based on information passed in from outside
void flock_member::set_state(int32_t &current_lat, int32_t &current_lon, int32_t &current_alt, int32_t &current_relative_alt, 
					int16_t &current_vx, int16_t &current_vy, int16_t &current_vz, uint16_t &current_hdg)
{
		_time_boot_ms = millis();
		_lat = current_lat;						//[e7]
		_lon = current_lon;						//[e7]
		_alt = current_alt;						//[m*100]
		_relative_alt = current_relative_alt;	//[m*100]
		_V.x = current_vx;						//[m/s*100]
		_V.y = current_vy;						//[m/s*100]
		_V.z = current_vz;						//[m/s*100]
		_hdg = current_hdg;						//cd

		_my_location.id = MAVLINK_MSG_ID_GLOBAL_POSITION_INT; //Eroneous ID at the moment...
		_my_location.options = MASK_OPTIONS_RELATIVE_ALT; //Sets options to signify relative altitude is used
		_my_location.alt = _alt; //[cm]
		_my_location.lat = _lat;
		_my_location.lng = _lon;

		//_last_location = _my_location;
		//_last_position_calc = _time_boot_ms;
		
		state_updated = true;
}

void flock_member::set_D2Goal(int32_t* p_D2Goal){
	_d2goal = *p_D2Goal;
}

void flock_member::set_last_heartbeat(uint32_t* timestamp){
	_last_heartbeat = *timestamp;
}

// Only applicable to a/c, not other formation members.
void flock_member::set_local_leader(uint8_t leader_sysid){
	_local_leader = leader_sysid;
}

void flock_member::set_global_leader(bool global_status){
	if(global_status)
		_local_leader = 0;

	_global_leader = global_status;
}

void flock_member::set_membermask(uint32_t* p_membermask)
{
	_membermask=*p_membermask;
}
//----------------------------------------------------
/*
// *Note, it is up to the calling function to signify whether or not to change the "updated" status after getting information 
#if HIL_MODE == HIL_MODE_DISABLED
const Location* flock_member::get_loc(AP_AHRS_DCM* ahrs, AP_IMU_INS* imu, AP_Airspeed* airspeed)
#endif
#if HIL_MODE == HIL_MODE_ATTITUDE
const Location* flock_member::get_loc(AP_AHRS_HIL* ahrs, AP_IMU_Shim* imu, AP_Airspeed* airspeed)
#endif
{
	if(state_updated) //New location information has been updated by GPS
	{
		Location* p_loc = &_my_location;
		return (const Location*)p_loc;
	}
	else	//Location information has not been updated by GPS, so we will use a dead-reckoning technique to approximate
	{
		Matrix3f dcm_matrix= ahrs->get_dcm_matrix();
		Vector3f wind = ahrs->wind_estimate();
		float deltat = millis()-_last_position_calc;
        float airspeed_ret;
        ahrs->airspeed_estimate(&airspeed_ret);
        // use airspeed to estimate our ground velocity in
        // earth frame by subtracting the wind
        Vector3f velocity = dcm_matrix.colx() * airspeed_ret;

        // add in wind estimate
        velocity += wind;

        _last_position_calc = millis();

        // update position delta for get_position()
        float position_offset_north = velocity.x * deltat;
        float position_offset_east  = velocity.y * deltat;
		location_offset(&_my_location, position_offset_north, position_offset_east);
		_last_position_calc = millis();
	}
}
*/
/*
const Location* flock_member::get_loc(){
	if(state_updated) //New location information has been updated by GPS
	{
		Location* p_loc = &_my_location;
		return (const Location*)p_loc;
	}
	else	//Location information has not been updated by GPS, so we will use a dead-reckoning technique to approximate
	{

		float deltat = millis()-_last_position_calc;
        const float* velocity = get_vel();

        // update position delta for get_position()
        float position_offset_north = velocity[0] * deltat;
        float position_offset_east  = velocity[1] * deltat;
		location_offset(&_my_location, position_offset_north, position_offset_east);
		_last_position_calc = millis();
	}
}
*/
const Location* flock_member::get_loc(){
	Location* p_loc = &_my_location;
	return (const Location*)p_loc;	
}

//Return pointer to velocity array in m/s*100
const int16_t* flock_member::get_vel(){
	float tmp_vel[3] = {_V.x, _V.y, _V.z};
	return (const int16_t*)tmp_vel;
}

const uint16_t* flock_member::get_hdg(){
	uint16_t* p_hdg = &_hdg;
	return (const uint16_t*)p_hdg;
}

const Relative* flock_member::get_rel(){
	Relative* p_current_relative;
	p_current_relative= &_my_relative;
	return (const Relative*)p_current_relative;
}

const int32_t* flock_member::get_D2Goal(){
	return (const int32_t*)&_d2goal;
}

void flock_member::update_rel(){
	if(_num_members == 0)
	{
		_my_relative.Num_members = _num_members;
	}
	else
	{
		for(int i = 0; i < _num_members; i++)
		{
			uint8_t j;				//new index to allow skipping ids not in view
			const Location* tmp_loc;//Temporary storage for flock member location pointer
			const Location* my_loc;	//Temporary storage for a/c location
			float tmp_distance;		//Temporary storage for distance magnitude
		
			j = _member_ids[i]-1;	//subtract 1 to start at offset ids (we start at 0 in C)

			tmp_loc = _member_ptrs[j]->get_loc();
			my_loc = get_loc();

			//Use the same lon correction technique used in Location.cpp
			float	tmp_lon_scale;
			static int32_t last_lat;
			static float scale = 1.0;
			if (labs(last_lat - tmp_loc->lat) < 1000) 
			{
			// we are within 0.001 degrees (about 1km) of the
			// same latitude. We can avoid the cos() and return
			// the same scale factor.
			tmp_lon_scale = scale;
			}
			else
			{
			tmp_lon_scale = cos((fabs((float)tmp_loc->lat)/1.0e7) * 0.0174532925);
			last_lat = tmp_loc->lat;
			}

			tmp_distance = get_distance(my_loc,tmp_loc); //[m]

			//Store Relative values in structure
			_dX[j].x = (int32_t)(100*De7ToM((tmp_loc->lat-my_loc->lat)));				//X distance of member wrt a/c (NED frame) [m*100]
			_dX[j].y = (int32_t)(De7ToM((tmp_loc->lng-my_loc->lng)*tmp_lon_scale)*100);	//Y distance of member wrt a/c (NED frame) [m*100]
			_dX[j].z = (int32_t)(tmp_loc->alt-my_loc->alt);								//Z distance of member wrt a/c (NED frame) [m*100]

			//Store Relative values in structure
			_my_relative.Num_members = _num_members;
			_my_relative.Member_ids[i]= _member_ids[i];
			_my_relative.dX[j] = _dX[j].x;
			_my_relative.dY[j] = _dX[j].y;
			_my_relative.dZ[j] = _dX[j].z;
		
			if(_member_ids[i] == _local_leader)
			{
				_dist_2_leader = (int32_t)(tmp_distance*100);
				const int16_t* tmp_leader_v = _member_ptrs[j]->get_vel();		//get velocity array from leader
				const uint16_t* tmp_p_hdg = _member_ptrs[j]->get_hdg();
				//calc relative velocity of leader wrt a/c in ft (NED frame) [m/s*100]
				_dV.x = (tmp_leader_v[0]-_V.x);													
				_dV.y = (tmp_leader_v[1]-_V.y);
				_dV.z = (tmp_leader_v[2]-_V.z);
				//Store Relative values in structure
				_my_relative.dVXL = _dV.x;
				_my_relative.dVYL = _dV.y;
				_my_relative.dVZL = _dV.z;
				_my_relative.dXL = _dX[j].x;
				_my_relative.dYL = _dX[j].y;
				_my_relative.dZL = _dX[j].z;
				_my_relative.d2L = _dist_2_leader;
				_my_relative.hdgL = *tmp_p_hdg;
			}
			//signal that the jth flock member's state has been used in a relative calc
			_member_ptrs[j]->state_updated = false;
		}
	}
	//note that the relative state of the a/c flock member
	rel_updated = true;
	//not that the absolute state of the a/c flock member has been used
	state_updated = false;
}

void flock_member::add_member_in_view(uint8_t sysid, flock_member* p_member){
	//this system will hopefully keep things easy to scale the number of members
	//new member ids are just added onto the end of the list, while their ptr fits into the slot corresponding to their id
	//the member_ids array will serve as an indexing guide (with the sysid system 1-N, where N is number of possible flock members)
	//that way, if a flock member pointer is in the member_ptrs array, it will get skipped over if it is not in the member_ids array
	
	//increment number of members (technically in view, if we want to implement a view restriction)
	_num_members++;
	//adjust the indexing to start at 0, instead of 1
	int i = _num_members-1; //index to use inside function
	//add the member sysid to the next available slot
	_member_ids[i] = sysid;
	//add the pointer to the slot corresponding with their sysid-1
	_member_ptrs[_member_ids[i]-1] = p_member;
	//signal that this member is in view (if we want to implement view restriction)
	p_member->in_view = true;
}

void flock_member::remove_member_in_view(uint8_t sysid){
	int ctr = 0;
//To keep this indexing system organized, when a member is removed, it loses its spot.
//the next member moves up into its index slot - this leaves the indexing system scalable
	for(int i=0;i<_num_members; i++){
		if(_member_ids[i]!=sysid){
			_member_ids[ctr]=_member_ids[i];
			ctr++;
		}
	}
	_num_members--;
	//remove from member mask
	uint32_t bits = (1<<(sysid-1));
	_membermask= _membermask & ~bits;
//We don't touch member pointers because they are indexed by member ids, not in joining order
}

const uint32_t* flock_member::get_last_update_time(){
	return (const uint32_t*)&_time_boot_ms;
}

const uint32_t* flock_member::get_last_heartbeat(){
	return (const uint32_t*)&_last_heartbeat;
}

uint8_t flock_member::get_local_leader()
{
	return _local_leader;
}

const flock_member* flock_member::get_member_pntr(uint8_t sysid)
{
	return (const flock_member*)_member_ptrs[sysid];
}
bool flock_member::get_leader_status()
{
	return _global_leader;
}

const uint32_t* flock_member::get_membermask()
{
	return (const uint32_t*)&_membermask;
}

uint8_t flock_member::get_members_iv()
{
	return _num_members;
}

flock_member FLOCK_MEMBER;