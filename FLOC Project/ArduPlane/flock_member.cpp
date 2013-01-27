// 
// 
// 

#include "flock_member.h"

flock_member::flock_member()	//Constructor for this a/c
{
	//upon construction of a/c flock member, assign it the SYSID given in the APM_Config file
	_sysid = SYSID;
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
void flock_member::set_state(int32_t* p_current_lat, int32_t* p_current_lon, int32_t* p_current_alt, int32_t* p_current_relative_alt, 
					int16_t* p_current_vx, int16_t* p_current_vy, int16_t* p_current_vz, uint16_t* p_current_hdg)
{
		_time_boot_ms = millis();
		_lat = *p_current_lat;
		_lon = *p_current_lon;
		_alt = *p_current_alt;
		_relative_alt = *p_current_relative_alt;
		_V.x = *p_current_vx;
		_V.y = *p_current_vy;
		_V.z = *p_current_vz;
		_hdg = *p_current_hdg;

		_my_location.id = FORMATION_CURRENT_STATE; //Eroneous ID at the moment...
		_my_location.options = MASK_OPTIONS_RELATIVE_ALT; //Sets options to signify relative altitude is used
		_my_location.alt = _alt;
		_my_location.lat = _lat;
		_my_location.lng = _lon;

		_alt_ft = _alt*3281/1000; //conversion from mm to ft*1000 (3 decimal precision)
		_relative_alt_ft = _alt*3281/1000; //conversion from mm to ft*1000 (3 decimal precision)
		_V_fps.x = _V.x*328/100; //conversion from cm/s to ft/s*100 (2 decimal precision)
		_V_fps.y = _V.y*328/100; //conversion from cm/s to ft/s*100 (2 decimal precision)
		_V_fps.z = _V.z*328/100; //conversion from cm/s to ft/s*100 (2 decimal precision)
		state_updated = true;
}

// Only applicable to a/c, not other formation members.
void flock_member::set_local_leader(uint8_t leader_sysid){
	_local_leader = leader_sysid;
}

void flock_member::set_global_leader(bool global_status){
	_global_leader = global_status;
}
//----------------------------------------------------

// *Note, it is up to the calling function to signify whether or not to change the "updated" status after getting information 
const Location* flock_member::get_loc(){
	Location* p_loc = &_my_location;
	return (const Location*)p_loc;
}

const uint8_t* flock_member::get_vel(){
	uint8_t tmp_vel[3] = {_V_fps.x, _V_fps.y, _V_fps.z};
	return (const uint8_t*)tmp_vel;
}

const Relative* flock_member::get_rel(){
	Relative* p_current_relative;
	p_current_relative= &_my_relative;
	return (const Relative*)p_current_relative;
}

void flock_member::update_rel(){
	for(int i = 0; i < _num_members; i++)
	{
		uint8_t j;				//new index to allow skipping ids not in view
		const Location* tmp_loc;//Temporary storage for flock member location pointer
		const Location* my_loc;	//Temporary storage for a/c location
		int32_t tmp_distance;	//Temporary storage for distance magnitude
		int32_t tmp_bearing;	//Temporary storage for bearing
		float tmp_distance_ft;	//Temporary storage for converted distance
		float tmp_bearing_rad;	//Temporary storage for converted bearing 
		
		j = _member_ids[i]-1;	//subtract 1 to start at offset ids (we start at 0 in C)

		tmp_loc = _member_ptrs[j]->get_loc();
		my_loc = get_loc();

		tmp_distance = get_distance_cm(my_loc,tmp_loc);
		tmp_bearing = get_bearing_cd(my_loc,tmp_loc);

		tmp_distance_ft = float(tmp_distance)/100.0*3.281;
		tmp_bearing_rad = float(tmp_bearing)/100.0*0.01745329252;

		_dX[j].x = int32_t(100*tmp_distance_ft*sin(PI/2-tmp_bearing_rad));	//X distance of member wrt a/c (NED frame)
		_dX[j].y = int32_t(100*tmp_distance_ft*cos(PI/2-tmp_bearing_rad));	//Y distance of member wrt a/c (NED frame)
		_dX[j].z = -(tmp_loc->alt - my_loc->alt)*328;						//Z distance of member wrt a/c (NED frame)
		//Store Relative values in structure
		_my_relative.Num_members = _num_members;
		_my_relative.Member_ids[i]= _member_ids[i];
		_my_relative.dX[j] = _dX[j].x;
		_my_relative.dY[j] = _dX[j].y;
		_my_relative.dZ[j] = _dX[j].z;
		
		if(_member_ids[i] == _local_leader)
		{
			_dist_2_leader = int32_t(100*tmp_distance_ft);
			const uint8_t* tmp_leader_v = _member_ptrs[j]->get_vel();		//get velocity array from leader 
			//calc relative velocity of leader wrt a/c in ft*100 (NED frame)
			_dV.x = tmp_leader_v[0]-_V_fps.x;													
			_dV.y = tmp_leader_v[1]-_V_fps.x;
			_dV.z = -(tmp_leader_v[2]-_V_fps.x);
			//Store Relative values in structure
			_my_relative.dvx = _dV.x;
			_my_relative.dvy = _dV.y;
			_my_relative.dvz = _dV.z;
			_my_relative.dXL = _dX[j].x;
			_my_relative.dYL = _dX[j].y;
			_my_relative.dZL = _dX[j].z;
			_my_relative.d2L = _dist_2_leader;
		}
		//signal that the jth flock member's state has been used in a relative calc
		_member_ptrs[j]->state_updated = false;
	}
	//note that the relative state of the a/c flock member
	rel_updated = true;
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
	_member_ptrs[_member_ids[i]] = p_member;
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
//We don't touch member pointers because they are indexed by member ids, not in joining order
}

const uint32_t* flock_member::get_last_update_time(){
	return (const uint32_t*)&_time_boot_ms;
}

bool flock_member::get_leader_status()
{
	return _global_leader;
}

flock_member FLOCK_MEMBER;