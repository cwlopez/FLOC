// 
// 
// 

#include "pf_field.h"
//including flock_member.h allows us to use the flock_member class
#include "flock_member.h"
#include <AP_AHRS_DCM.h>
#include <AP_Airspeed.h>

//constructor
pf_field::pf_field()
{
	//set parameters with pre-defined constants
	_chi			= PFG_CHI;
	_tau			= PFG_TAU;
	_zeta			= PFG_ZETA;
#ifdef ATT_EXP
	_sigma			= PFG_SIGMA;
#endif
	_lambda.x		= PFG_X_LAMBDA/100.0;
	_lambda.y		= PFG_Y_LAMBDA/100.0;
	_lambda.z		= PFG_Z_LAMBDA/100.0;
	_pf_offset_b.x	= PFG_x_OFFSET/100.0;
	_pf_offset_b.y	= PFG_y_OFFSET/100.0;
	_pf_offset_b.z	= PFG_z_OFFSET/100.0;
	_VWP_offset		= PFG_VWP_OFFSET;
	_k_phi_near.x	= PFG_K_PHI_X_NEAR/100.0;
	_k_phi_near.y	= PFG_K_PHI_Y_NEAR/100.0;
	_k_phi_near.z	= PFG_K_PHI_Z_NEAR/100.0;
	_k_V_near		= PFG_K_V_NEAR/100.0;
	_k_alt_V_near	= PFG_K_ALT_V_NEAR/100.0;
	_k_phi_V_near	= PFG_K_PHI_V_NEAR/100.0;
	_k_phi_far.x	= PFG_K_PHI_X_FAR/100.0;
	_k_phi_far.y	= PFG_K_PHI_Y_FAR/100.0;
	_k_phi_far.z	= PFG_K_PHI_Z_FAR/100.0;
	_k_V_far		= PFG_K_V_FAR/100.0;
	_k_alt_V_far	= PFG_K_ALT_V_FAR/100.0;
	_k_phi_V_far	= PFG_K_PHI_V_FAR/100.0;
	_side			= PFG_DEFAULT_SIDE;

}

#if HIL_MODE == HIL_MODE_DISABLED
	void pf_field::update(flock_member* p_ac, AP_AHRS_DCM* ahrs, AP_Airspeed* airspeed)
{
#endif
#if HIL_MODE == HIL_MODE_ATTITUDE
	void pf_field::update(flock_member* p_ac, AP_AHRS_HIL* ahrs, AP_Airspeed* airspeed)
{
#endif
				const Relative* tmp_p_rel = p_ac->get_rel(); //Gets the current relative information from the ac object
				//Zero temporary vectors
				Vector3f tmp_dX;
				Vector3f tmp_dL;
				float tmp_d2L;
				Vector3f tmp_dL_b;
				Vector3f tmp_dV;
				Vector3f tmp_r_phi;
				Vector3f tmp_a_phi;
				float tmp_mag_phi;			
				float tmp_dNorth_com;
				float tmp_dEast_com;
				float tmp_dalt_com;	
				float tmp_dspd_com;
				float tmp_psi;
				Vector3f tmp_pf_offset_NED;
				//update offset with side correction
				Vector3f tmp_pf_offset = _pf_offset_b;
				tmp_pf_offset.y = tmp_pf_offset.y*_side;
				for(int i=0;i<tmp_p_rel->Num_members;i++)
				{
					int j = tmp_p_rel->Member_ids[i]-1; //indexing convention to organize member pointers
					tmp_dX.x=tmp_p_rel->dX[j];	//[M]
					tmp_dX.y=tmp_p_rel->dY[j];	//[M]
					tmp_dX.z=tmp_p_rel->dZ[j];	//[M]
					//repulsive function accumulation
					tmp_r_phi.x += (-(tmp_dX.x)*_tau/_zeta)*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/_zeta);
					tmp_r_phi.y += (-(tmp_dX.y)*_tau/_zeta)*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/_zeta);
					tmp_r_phi.z += (-(tmp_dX.z)*_tau/_zeta)*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/_zeta);
				}

				tmp_dL.x=tmp_p_rel->dXL;		//[M]
				tmp_dL.y=tmp_p_rel->dYL;		//[M]
				tmp_dL.z=tmp_p_rel->dZL;		//[M]
				tmp_d2L=tmp_p_rel->d2L;
				tmp_dV.x=tmp_p_rel->dvx;		//[M/s]
				tmp_dV.y=tmp_p_rel->dvy;		//[M/s]
				tmp_dV.z=tmp_p_rel->dvz;		//[M/s]


				//Calculate the attractive potential gradient components (quadratic PF)
				tmp_psi = tmp_p_rel->hdgL/100.00;		//Get the heading of the Leader
				//Rotate the NED distances to align with the leader's heading
				tmp_pf_offset_NED.x = tmp_pf_offset.x*cos(ToRad(tmp_psi)) - tmp_pf_offset.y*sin(ToRad(tmp_psi));
				tmp_pf_offset_NED.y = tmp_pf_offset.x*sin(ToRad(tmp_psi)) + tmp_pf_offset.y*cos(ToRad(tmp_psi));
				tmp_pf_offset_NED.z = tmp_pf_offset.z;
				tmp_dL -=tmp_pf_offset_NED;
				tmp_a_phi.x = tmp_dL.x;//_lambda.x*(tmp_dL.x);			//Calculate PF X gradient with weighting parameter
				tmp_a_phi.y = tmp_dL.y;//_lambda.y*(tmp_dL.y);			//Calculate PF Y gradient with weighting parameter
				tmp_a_phi.z = tmp_dL.z;//_lambda.z*(tmp_dL.z);			//Calculate PF Z gradient with weighting parameter

				//Calculate the total potential gradient components
				_phi_NED = tmp_a_phi + tmp_r_phi;
				//Calculate the magnitude of the total potential function gradient
				tmp_mag_phi = _phi_NED.length();
				
				//Divide-by-zero check (safety first)
				if(tmp_mag_phi == 0)
				{
					_Nphi_NED.zero();
					_Nphi_c_NED.zero();
				}
				else
				{
					//Calculate the normalized potential gradient components
					_Nphi_NED = _phi_NED.normalized(); 
					if(tmp_d2L<_chi) //Near-field consideration- We don't want the VWP behind the body
					{
						_Nphi_b.x = _Nphi_NED.x*cos(ToRad(*p_ac->get_hdg()/100.0)) + _Nphi_NED.y*sin(ToRad(*p_ac->get_hdg()/100.0));
						if(_Nphi_b.x<0)
						{
							_phi_b.x = _phi_NED.x*cos(ToRad(*p_ac->get_hdg()/100.0)) + _phi_NED.y*sin(ToRad(*p_ac->get_hdg()/100.0));
							_phi_b.y = - _phi_NED.x*sin(ToRad(*p_ac->get_hdg()/100.0)) + _phi_NED.y*cos(ToRad(*p_ac->get_hdg()/100.0));
							_phi_b.x = fabs(_phi_b.y); // forces the bearing to be +/-45
							_phi_c_NED.x= _phi_b.x*cos(ToRad(*p_ac->get_hdg()/100.0)) - _phi_b.y*sin(ToRad(*p_ac->get_hdg()/100.0));
							_phi_c_NED.y= _phi_b.x*sin(ToRad(*p_ac->get_hdg()/100.0)) + _phi_b.y*cos(ToRad(*p_ac->get_hdg()/100.0));
							_Nphi_c_NED = _phi_c_NED.normalized();
						}
					}
				}

				///////////////////////DEBUG
				debug_tmp_dX = tmp_dX;
				debug_r_phi = tmp_r_phi;
				debug_a_phi = tmp_a_phi;
				debug_Nphi = _Nphi_NED;
				debug_mag_phi = tmp_mag_phi;
				//////////////////////DEBUG
				if(tmp_d2L<_chi)
				{
					//Calculate the North and East Offset [m]
					tmp_dNorth_com	=	_VWP_offset*_k_phi_near.x*_Nphi_c_NED.x;
					tmp_dEast_com	=	_VWP_offset*_k_phi_near.y*_Nphi_c_NED.y;
					//Calculate the change in altitude [m]
					//k_alt_V is the equivalent of a derivative gain- set to 0 for simplicity. *Also, watch units- dV is ft/s*
					tmp_dalt_com	=	_VWP_offset*_k_phi_near.z*_Nphi_NED.z + _k_alt_V_near*tmp_dV.z;  
					//Calculate the change in airspeed [m/s]
					//k_phi_V_near is the equivalent of an integrator gain, while k_V_near is the equivalent of a proportional gain
					tmp_dspd_com=_k_phi_V_near*_Nphi_b.x + _k_V_near*tmp_dV.length(); //[m/s]
					//Convert to units used by WP and airspeed commands
					tmp_dspd_com= tmp_dspd_com*100; //[cm/s]
				}
				else
				{								//Far-Field Case
					tmp_dNorth_com	=	_VWP_offset*_k_phi_far.x*_Nphi_NED.x;
					tmp_dEast_com	=	_VWP_offset*_k_phi_far.y*_Nphi_NED.y;
					tmp_dalt_com	=	_VWP_offset*_k_phi_far.z*_Nphi_NED.z + _k_alt_V_far*tmp_dV.z;
					tmp_dspd_com	=	0;
				}
				
				////////////////////////DEBUG////////////////
				debug_dNorth_com = tmp_dNorth_com;
				debug_dEast_com = tmp_dEast_com;
				//debug_dalt_com = tmp_dalt_com;
				debug_dspd_com = tmp_dspd_com;
				////////////////////////////////////////////
					const Location* p_current_location = p_ac->get_loc();
					_next_VWP = *p_current_location;
					location_offset(&_next_VWP, tmp_dNorth_com, tmp_dEast_com);
					_next_VWP.alt-=(int32_t)100*tmp_dalt_com; //double-check sign here. I am subtracting to counter the + is down convention (units converted to cm)
					constrain(_next_VWP.alt,PFG_MIN_ALT,PFG_MAX_ALT);
					debug_dalt_com = _next_VWP.alt;
				if(tmp_d2L<_chi)
				{							//Near-Field Case
					_next_airspeed_com = airspeed->get_airspeed_cm()+tmp_dspd_com;
					constrain(_next_airspeed_com,PFG_MIN_AIRSPEED_CM,PFG_MAX_AIRSPEED_CM);
				}
				else
				{							//Far-Field Case
					_next_airspeed_com = PFG_MAX_AIRSPEED_CM; //In far-field, set airspeed to maximum allowable (to catch up)
				}
} 

const Location* pf_field::get_VWP(){
	return (const Location*)&_next_VWP;
}

const int32_t* pf_field::get_new_speed(){
	return (const int32_t*)&_next_airspeed_com;
}
pf_field PF_FIELD;

