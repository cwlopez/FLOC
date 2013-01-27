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
	_lambda.x		= PFG_X_LAMBDA;
	_lambda.y		= PFG_Y_LAMBDA;
	_lambda.z		= PFG_Z_LAMBDA;
	_pf_offset_b.x	= PFG_x_OFFSET/100.0;
	_pf_offset_b.y	= PFG_y_OFFSET/100.0;
	_pf_offset_b.z	= PFG_z_OFFSET/100.0;
	_VWP_offset		= PFG_VWP_OFFSET;
	_x_phi_near		= PFG_X_PHI_NEAR;
	_k_phi_near.x	= PFG_K_PHI_X_NEAR;
	_k_phi_near.y	= PFG_K_PHI_Y_NEAR;
	_k_phi_near.z	= PFG_K_PHI_Z_NEAR;
	_k_V_near		= PFG_K_V_NEAR;
	_k_alt_V_near	= PFG_K_ALT_V_NEAR;
	_k_phi_V_near	= PFG_K_PHI_V_NEAR;
	_k_phi_far.x	= PFG_K_PHI_X_FAR;
	_k_phi_far.y	= PFG_K_PHI_Y_FAR;
	_k_phi_far.z	= PFG_K_PHI_Z_FAR;
	_k_V_far		= PFG_K_V_FAR;
	_k_alt_V_far	= PFG_K_ALT_V_FAR;
	_k_phi_V_far	= PFG_K_PHI_V_FAR;
	_side			= PFG_DEFAULT_SIDE;

}

void pf_field::update(flock_member* p_ac, AP_AHRS_DCM* ahrs, AP_Airspeed* airspeed)
{
	//State Machine implemented to break calculation up into more managable chunks.
	//5 cases means full calc should take place at 10 Hz
	
	//State Machine counter for VWP placement
	static int VWP_calc_cntr; //Should be initialized as 0 upon creation

	while(!updated) //State Machine only runs when pf_field updated variable is "false"
	{				
		switch(VWP_calc_cntr)
		{
		//first block of calcs: Setting up
		case 0:
				if(p_ac->rel_updated)
				{
					tmp_p_rel = p_ac->get_rel(); //Gets the current relative information from the ac object
					//Zero temporary vectors
					tmp_dX.zero();
					tmp_dL.zero();
					tmp_dL_b.zero();
					tmp_dV.zero();
					tmp_r_phi.zero();
					tmp_a_phi.zero();
					tmp_mag_phi = 1;			
					tmp_bearing_com = 0;
					tmp_dalt_com = 0;	
					tmp_dspd_com = 0;

					//update offset with side correction
					tmp_pf_offset = _pf_offset_b;
					tmp_pf_offset.y = tmp_pf_offset.y*_side;

					VWP_calc_cntr++;			//Move to step 1
				}
				return;
		//Second block of calcs: accumulate repulsive potential functions from flock members
		case 1:
				for(int i=0;i<tmp_p_rel->Num_members;i++)
				{
					int j = tmp_p_rel->Member_ids[i]-1; //indexing convention to organize member pointers
					tmp_dX.x=tmp_p_rel->dX[j]/100.0;	//Converted to float [ft]
					tmp_dX.y=tmp_p_rel->dY[j]/100.0;	//Converted to float [ft]
					tmp_dX.z=tmp_p_rel->dZ[j]/100.0;	//Converted to float [ft]
					//repulsive function accumulation
					tmp_r_phi.x += (-(tmp_dX.x)*_tau/_zeta)*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/_zeta);
					tmp_r_phi.y += (-(tmp_dX.y)*_tau/_zeta)*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/_zeta);
					tmp_r_phi.z += (-(tmp_dX.z)*_tau/_zeta)*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/_zeta);
				}
				
				VWP_calc_cntr++;		//Move to  step 2
				return;
		//Third block of calcs: calculate attractive potential function, total potential gradient, and normalized potential gradient
		case 2:
				tmp_dL.x=tmp_p_rel->dXL/100.0;		//Converted to float [ft]
				tmp_dL.y=tmp_p_rel->dYL/100.0;		//Converted to float [ft]
				tmp_dL.z=tmp_p_rel->dZL/100.0;		//Converted to float [ft]
				tmp_d2L=tmp_p_rel->d2L/100.0;
				tmp_dV.x=tmp_p_rel->dvx/100.0;		//Converted to float [ft/s]
				tmp_dV.y=tmp_p_rel->dvy/100.0;		//Converted to float [ft/s]
				tmp_dV.z=tmp_p_rel->dvz/100.0;		//Converted to float [ft/s]

				//Calculate the attractive potential gradient components (quadratic PF)
				tmp_rot = ahrs->get_dcm_matrix();	//Get the rotation matrix from body to NED
				tmp_dL_b = tmp_rot.mul_transpose(tmp_dL);	//Get distances in body frame
				tmp_dL_b -=tmp_pf_offset;				//Adjust distances with body frame offsets
				tmp_dL = tmp_rot*tmp_dL_b;					//Transpose distances back into NED frame
				tmp_a_phi.x = _lambda.x*(tmp_dL.x);		//Calculate PF X gradient with weighting parameter
				tmp_a_phi.y = _lambda.y*(tmp_dL.y);		//Calculate PF X gradient with weighting parameter
				tmp_a_phi.z = _lambda.z*(tmp_dL.z);		//Calculate PF X gradient with weighting parameter


				//Calculate the total potential gradient components
				_phi_NED = tmp_a_phi + tmp_r_phi;
				_phi_b = tmp_rot.mul_transpose(_phi_NED);
				if(tmp_d2L<_chi)						//Near field consideration (we don't want wp behind body)
				{
					_phi_b.x = _x_phi_near;
					_phi_c_NED = tmp_rot*_phi_b;
				}

				//Calculate the magnitude of the total potential function gradient
				////tmp_mag_phi = safe_fsqrt(square(_phi.x)+square(_phi.y)+square(_phi.z));
				tmp_mag_phi = _phi_NED.length();
				//Divide-by-zero check (safety first)
				if(tmp_mag_phi != 0.0)
				{
					//Calculate the normalized potential gradient components
					_Nphi_NED = _phi_NED.normalized();
					_Nphi_b = tmp_rot.mul_transpose(_Nphi_NED);
					if(tmp_d2L<_chi)
					{
						_Nphi_c_NED = _phi_c_NED.normalized();
					}
				}
				else
				{
					_Nphi_NED.zero();
					_Nphi_c_NED.zero();
					_Nphi_b.zero();
				}
		
				VWP_calc_cntr++;				//Move to step 3
				return;
		//Fourth block of calcs: calculate commanded bearing, change in altitude, and change in speed	
		case 3:
				if(tmp_d2L<_chi)
				{
					//Calculate the bearing [deg*100] from unit vector components (0= N, 90 = E, 180 = S, 270 = W)
					//k_phi_X and k_phi_Y should be set to 1 for now
					tmp_bearing_com = 180/PI*(PI/2-atan2(((_k_phi_near.x/100.0)*_Nphi_c_NED.x),((_k_phi_near.y/100.0)*_Nphi_c_NED.y)));
					//Calculate the change in altitude [ft] + is down
					//k_phi_Z should be set to 1 for now
					//k_V_Z_far acts as the D gain of a PD controller, set to 0 for simplicity, but can be used to reduce altitude overshoot
					tmp_dalt_com=((_VWP_offset/100.0)*(_k_phi_near.z/100.0)*(_Nphi_NED.z)+((_k_alt_V_near/100.0)*tmp_dV.z));
					//Calculate the change in airspeed [ft/s]
					//k_phi_V_near is the equivalent of an integrator gain, while k_V_near is the equivalent of a proportional gain
					tmp_dspd_com=((_k_phi_V_near/100.0)*(_Nphi_b.x)+(_k_V_near/100.0)*tmp_dV.length());
					//Convert to units used by WP and airspeed commands
					tmp_dalt_com=3.281*tmp_dalt_com; //[m]
					tmp_dspd_com=30.48*tmp_dspd_com; //[cm/s]
				}
				else
				{								//Far-Field Case
					//Calculate the bearing [deg*100] from unit vector components (0= N, 90 = E, 180 = S, 270 = W)
					//k_phi_X and k_phi_Y should be set to 1 for now
					tmp_bearing_com = 180/PI*(PI/2-atan2(((_k_phi_far.x/100.0)*_Nphi_NED.x),((_k_phi_far.y/100.0)*_Nphi_NED.y)));
					//Calculate the change in altitude [ft] + is down
					//k_phi_Z should be set to 1 for now
					//k_V_Z_far acts as the D gain of a PD controller, set to 0 for simplicity, but can be used to reduce altitude overshoot
					tmp_dalt_com=(_VWP_offset/100.0)*(_k_phi_far.z/100.0)*(_Nphi_NED.z)+((_k_alt_V_far)*tmp_dV.z); 
					tmp_dspd_com=0;
					//Convert to units used by WP
					tmp_dalt_com=3.281*tmp_dalt_com; //[m]
				}
				
				VWP_calc_cntr++;			//Move to step 4
				return;
		//Fifth block of calcs: set VWP and Airspeed command
		case 4:
					const Location* p_current_location = p_ac->get_loc();
					_next_VWP = *p_current_location;
					location_update(&_next_VWP,tmp_bearing_com,_VWP_offset/100.0*3.281);
					_next_VWP.alt-=tmp_dalt_com; //double-check sign here. I am subtracting to counter the + is down convention
					constrain(_next_VWP.alt,PFG_MIN_ALT,PFG_MAX_ALT);
				if(tmp_d2L<_chi)
				{							//Near-Field Case
					_next_airspeed_com = airspeed->get_airspeed_cm()+tmp_dspd_com;
					constrain(_next_airspeed_com,PFG_MIN_AIRSPEED_CM,PFG_MAX_AIRSPEED_CM);
				}
				else
				{							//Far-Field Case
					_next_airspeed_com = PFG_MAX_AIRSPEED_CM; //In far-field, set airspeed to maximum allowable (to catch up)
				}
				
				VWP_calc_cntr=0;
				updated = true;
				return;
			} //End Switch
	} // End While
	return;
}

const Location* pf_field::get_VWP(){
	return (const Location*)&_next_VWP;
}

const int32_t* pf_field::get_new_speed(){
	return (const int32_t*)&_next_airspeed_com;
}
pf_field PF_FIELD;

