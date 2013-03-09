// pf_field.h

#ifndef _PF_FIELD_h
#define _PF_FIELD_h

//including formation_common.h should include: APM_Config, AP_Math, AP_Common, Arduino.h, stdint.h, c++.h, and AP_Param.h
//also, allows for use of Vector3<T> and Matrix3<T>, Relative struct, and RollCall struct
#include "formation_common.h"

//forward declaration of the flock_member class 
class flock_member;
#if HIL_MODE == HIL_MODE_DISABLED
class AP_AHRS_DCM;
#endif
#if HIL_MODE == HIL_MODE_ATTITUDE
class AP_AHRS_HIL;
#endif
class AP_Airspeed;
//forward declaration of the Location struct
struct Location;
//forward declaration of the Relative struct
struct Relative;

class pf_field
{
 private:

	 //near-field threshold
	 uint16_t	_chi;				//radius dividing the near-field and far-field regions [M]

	 //potential function shaping functions
	 uint16_t	_tau;				//repulsive potential sizing parameter
	 uint16_t	_zeta;				//repulsive potential sizing parameter
	 uint16_t	_sigma;				//attractive potential sizing parameter
	 Vector3ui	_lambda;			//attractive potential weight*100

	 //attractive potential offsets (+x out the nose, +y out the right wing, +z down out of the belly)
	 Vector3f	_pf_offset_b;		//attractive potential offset from theoretical center of a/c [M*100]
	 int8_t	_side;				//either +1 or -1
	 
	 //virtual Waypoint offset 
	 uint16_t	_VWP_offset;			//distance from current location to next VWP [M]

	 //potential function gains in near-field region (NED coordinate frame)
	 Vector3f	_k_phi_near;			//gain for normalized PFG *100
	 float		_k_V_near;				//gain for the velocity-match component *100
	 float		_k_alt_V_near;			//gain for the Altitude velocity match component
	 float		_k_phi_V_near;			//gain for the PFG component of velocity matching *100

	 //potential function gains in far-field region (NED coordinate frame)
	 Vector3f	_k_phi_far;				//gain for normalized PFG *100
	 float		_k_V_far;				//gain for the velocity-match component *100
	 float		_k_alt_V_far;			//gain for the Altitude velocity match component
	 float		_k_phi_V_far;			//gain for the PFG component of velocity matching *100

	 //potential function gradient values
	 Vector3f	_phi_NED;				//gradient value for the total potential function (NED frame)
	 Vector3f	_phi_b;					//gradient value for the total potential function (body frame)
	 Vector3f	_phi_c_NED;				//gradient value for the total potential function, corrected to insure waypoints are not placed behind the a/c (NED frame)
	 Vector3f	_Nphi_NED;				//Normalized gradient value for the total potential function (NED frame)
	 Vector3f	_Nphi_b;				//Normalized gradient value for the total potential function (body frame)
	 Vector3f	_Nphi_c_NED;			//Normalized gradient value for the total potential function, corrected to insure waypoints are not placed behind the a/c (NED frame)

	 //VWP, and airspeed commands
	 Location	_next_VWP;				//Location structure is the standard way to store WPs in ArduPlane
	 int32_t	_next_airspeed_com;		//Airspeed commands are int32_t, not uint32_t ... not sure why

public:
	//////////////DEBUG///////////////////////////
	float debug_dNorth_com;
	float debug_dEast_com;
	float debug_dspd_com;
	float debug_dalt_com;
	float debug_mag_phi;
	Vector3f debug_Nphi;
	Vector3f debug_r_phi;
	Vector3f debug_a_phi;
	Vector3f debug_tmp_dX;
	Vector3f debug_tmp_dL;
	Vector3f debug_tmp_pf_offset;
	/////////////////////////////////////////////


	//constructor: sets parameters and initializes some variables
	pf_field();

	//update the potential field: given the pointer to the a/c flock member object (this a/c), 
	//updates the potential function field based on the relative positions of the other flock members wrt the a/c 
#if HIL_MODE == HIL_MODE_DISABLED
	void update(flock_member* p_ac, AP_AHRS_DCM* p_ahrs, AP_Airspeed* p_airspeed);
#endif
#if HIL_MODE == HIL_MODE_ATTITUDE
	void update(flock_member* p_ac, AP_AHRS_HIL* p_ahrs, AP_Airspeed* p_airspeed);
#endif
	//retrieves current VWP calculated from pf_field object
	const Location* get_VWP();

	//retrieves current Airspeed command from pf_field object
	const int32_t* get_new_speed();

	//publicly accessible toggle to describe if the pf_field is updated
	bool updated;
};

extern pf_field PF_FIELD;

#endif

