// AP_flock_member.h

#ifndef _AP_FLOCK_MEMBER_h
#define _AP_FLOCK_MEMBER_h

//including formation_common.h should include: APM_Config, AP_Math, AP_Common, Arduino.h, stdint.h, c++.h, and AP_Param.h
//also, allows for use of Vector3<T> and Matrix3<T>, Relative struct, and RollCall struct
#include "formation_common.h"

//-------------
class flock_member
{
 private:

	 //flock member state
	 uint8_t	_sysid;				//The sysid of the flock member (Current convention: Huey = 1, Dewey = 2, Louie = 3, N/A = 0)
	 Location	_my_location;		//Location structure: {id, options, p1, alt, lat, lng} *See AP_Common ln 210-217 for detail
	 Location	_last_location;		//Use for dead-reckoning referencing
	 uint32_t	_time_boot_ms;		//Timestamp (milliseconds since system boot)
	 uint32_t	_last_heartbeat;	//Timestamp of last heartbeat message received (milliseconds)
	 //uint32_t	_last_position_calc;//Timestamp of last position calculation (for dead-reckoning support)
	 int32_t	_lat;				//Latitude, expressed as *1E7
	 int32_t	_lon;				//Longitude, expressed as *1E7
	 int32_t	_alt;				//Altitude in meters, expressed as *1000 (mm), above MSL
	 int32_t	_relative_alt;		//Altitude above ground in meters, expressed as *1000 (mm)
	 Vector3i	_V;					//Ground Speed (GPS frame), expressed as m/s
	 uint16_t	_hdg;				//Compass heading in degrees *100, 0.0...359.99 degrees, if unknown, set to 65535

	 //flock Leadership
	 bool			_global_leader;				//True if a/c is the global leader, False if the a/c is following another flock member
	 uint8_t		_local_leader;				//The sysid of the local leader (Current convention: Huey = 1, Dewey = 2, Louie = 3, N/A = 0)
	 int32_t		_d2goal;					//The distance from current location to goal [m] (including distance traveled in loiter- ie arclength- for a goal that is "wound" about a point)

	 //flock membership
	 uint8_t		_num_members;				//The number of a/c systems in view default is 0
	 flock_member*	_member_ptrs[FLOCK_SIZE];	//array to hold the pointers to the members in view, for information retrieval purposes
	 uint8_t		_member_ids[FLOCK_SIZE];	//array to hold the sysid of the members in view, for indexing purposes
	 uint32_t		_membermask;				//bitmask for flock membership (supports 32 members)
	 
	 //relative positions
	 Vector3l	_dX[FLOCK_SIZE];	//Distance between a/c and other flock members (NED frame) in meters 	
	 
	 //velocities relative to leader
	 Vector3l	_dV;				//Ground speed (NED frame) difference between a/c and leader, expressed in meters
	 Relative	_my_relative;		//Structure containing all the relative S/A information
	 
	 //distance magnitude from leader
	 int32_t	_dist_2_leader;		//Magnitude of distance between a/c and leader, in meters
 public:

	flock_member();	//Constructor for this a/c

	flock_member(uint8_t sysid);	//Constructor function for other flock member

	//Publicly accessible status checks
	bool state_updated;				//True if the state of the flock member has been updated
	bool rel_updated;				//True if the relative state of the a/c flock member has been updated
	bool in_view;					//True if flock member can be seen by a/c (**may or may not be implemented**)

	//Functions to set protected variables 
	void set_state(int32_t &current_lat, int32_t &current_lon, int32_t &current_alt, int32_t &current_relative_alt, 
					int16_t &current_vx, int16_t &current_vy, int16_t &current_vz, uint16_t &current_hdg);
	void set_D2Goal(int32_t* p_D2Goal);				//set distance from current location to goal location
	void set_last_heartbeat(uint32_t* timestamp);	//set the time stamp of the last heartbeat recieved

	//// Only applicable to a/c flock member
	void set_local_leader(uint8_t leader_sysid);	// Assigns the sysid of the local leader, found through swarm algorithm
	void set_global_leader(bool global_status);		// If no leaders are available, assigns the role of global leader
	void set_membermask(uint32_t* p_membermask);	// Sets the member bitmask 

	//Functions to get protected variables
	const Location*	get_loc();				//returns a pointer to a copy of the private location structure (another flock member)
	const int16_t*	get_vel();				//returns a pointer to an array of the private velocity variables {_vx_fps, _vy_fps, _vz_fps}
	const uint16_t* get_hdg();				//returns a pointer to the flockmember heading in cd (deg*100)
	const Relative*	get_rel();				//returns a pointer to a copy of the private relative structure
	const uint32_t*	get_last_update_time();	//returns timestamp of last update
	const uint32_t* get_last_heartbeat();	//returns timestamp of last heartbeat
		  uint8_t get_local_leader();		//returns local leader sysid
		  bool get_leader_status();			//returns global leader status	
	const uint32_t* get_membermask();		//returns member bitmask
		  uint8_t get_members_iv();			//returns the number of members in view
	const int32_t* get_D2Goal();			//returns distance from current location to goal location
	const flock_member* get_member_pntr(uint8_t sysid);	//returns a pointer to the designated member

	//Function to update a/c flock member
	void update_rel();
	
	//Function to add/remove flock members from view
	void add_member_in_view(uint8_t sysid, flock_member* p_member);
	void remove_member_in_view(uint8_t sysid);
};
extern flock_member FLOCK_MEMBER;

#endif

