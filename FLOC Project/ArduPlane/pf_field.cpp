// 
// 
// 

#include "pf_field.h"
//including flock_member.h allows us to use the flock_member class
#include "flock_member.h"
#include <AP_AHRS_DCM.h>
#include <AP_AHRS_HIL.h>
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
	_pf_offset_l.x	= PFG_x_OFFSET;
	_pf_offset_l.y	= PFG_y_OFFSET;
	_pf_offset_l.z	= PFG_z_OFFSET;
	_VWP_offset		= PFG_VWP_XY_OFFSET;
	_VWP_Z_offset	= PFG_VWP_Z_OFFSET;
	_k_V_near		= PFG_K_V_NEAR;
	_k_alt_V_near	= PFG_K_ALT_V_NEAR;
	_k_phi_V_near	= PFG_K_PHI_V_NEAR;
	_k_V_far		= PFG_K_V_FAR;
	_k_alt_V_far	= PFG_K_ALT_V_FAR;
	_k_phi_V_far	= PFG_K_PHI_V_FAR;
	_side			= PFG_DEFAULT_SIDE;

}

void pf_field::set_side(int8_t side)
{
	_side = side;
}

#if HIL_MODE == HIL_MODE_DISABLED
	void pf_field::update(local_member* p_ac, AP_AHRS_DCM* ahrs, AP_Airspeed* airspeed)
{
#endif
#if HIL_MODE == HIL_MODE_ATTITUDE
	void pf_field::update(local_member* p_ac, AP_AHRS_HIL* ahrs, AP_Airspeed* airspeed)
{
#endif
				const Relative* tmp_p_rel = p_ac->get_rel(); //Gets the current relative information from the ac object
				uint16_t tmp_psi_ac = *p_ac->get_hdg();	//Get our heading (deg)
				//Calculate total repulsive function from i flock members
				Vector3f tmp_r_phi; //initialize vector (NED components)
				//Sum repulsive functions from i flock members
				for(int i=0;i<tmp_p_rel->Num_members;i++)
				{
					//indexing convention to organize member pointers
					int j = tmp_p_rel->Member_ids[i]-1; 
					//Relative distance to flock members (NED components)
					Vector3f tmp_dX;	//initialize vector each iteration
					tmp_dX.x=tmp_p_rel->dX[j]/100.0;	//[m]
					tmp_dX.y=tmp_p_rel->dY[j]/100.0;	//[m]
					tmp_dX.z=tmp_p_rel->dZ[j]/100.0;	//[m]

					//Calculate repulsive parameters based on relative states

					//Gaussian repulsive function accumulation
					tmp_r_phi.x += (-2*(tmp_dX.x)*(_tau/100.0)/(_zeta/100.0))*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/(_zeta/100.0));
					tmp_r_phi.y += (-2*(tmp_dX.y)*(_tau/100.0)/(_zeta/100.0))*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/(_zeta/100.0));
					tmp_r_phi.z += (-2*(tmp_dX.z)*(_tau/100.0)/(_zeta/100.0))*exp(-(square(tmp_dX.x)+square(tmp_dX.y)+square(tmp_dX.z))/(_zeta/100.0));
				}
				_phi_r = tmp_r_phi;		//Repulsive potential gradient vector (NED component)

				//Relative distance vector to leader (NED components)
				Vector3i tmp_dL;	//initialize vector	(NED components)
				tmp_dL.x=tmp_p_rel->dXL;		//[m*100]
				tmp_dL.y=tmp_p_rel->dYL;		//[m*100]
				tmp_dL.z=tmp_p_rel->dZL;		//[m*100]
				//Relative distance magnitude to leader
				int32_t tmp_d2L=tmp_p_rel->d2L;	//[m*100]
				//Relative velocity to leader (NED components)
				Vector3i tmp_dV;	//initialize vector (NED components)
				int16_t tmp_dVaspd;	//initialize dVaspd (Local Navigation x component)
				tmp_dV.x=tmp_p_rel->dVXL;		//[m/s*100]
				tmp_dV.y=tmp_p_rel->dVYL;		//[m/s*100]
				tmp_dV.z=tmp_p_rel->dVZL;		//[m/s*100]


				//Calculate the attractive potential gradient (NED components)
					/*Note:
					Attractive potential is comprised of two types of potential functions
					-In the far-field regime we use a linear potential field
					-In the near-field regime we use a quadratic potential field
					These fields must be matched at _chi, the near-field, far-field regime cut-off such that
					-Gradient of quadratic at _chi = gradient of linear at _chi

					In far-field, we are guided towards the leader's position (no offset) - this should improve convergence
					In near-field, we are guided towards the offset position, the side of which is dictated by swarm algorithm
				*/
					Vector3i tmp_pf_offset_NED;						//Initialize vector
					uint16_t tmp_psi_L = tmp_p_rel->hdgL;		//Get the heading of the Leader
					//set offset (Leader's Local frame)
					Vector3i tmp_pf_offset = _pf_offset_l;
					//update offset with side correction
					tmp_pf_offset.y = tmp_pf_offset.y*_side;
					//Rotate the offset distances (Leader's Local Navigation Frame) to NED frame
					tmp_pf_offset_NED.x = tmp_pf_offset.x*cos(ToRad(tmp_psi_L/100.0)) - tmp_pf_offset.y*sin(ToRad(tmp_psi_L/100.0));
					tmp_pf_offset_NED.y = tmp_pf_offset.x*sin(ToRad(tmp_psi_L/100.0)) + tmp_pf_offset.y*cos(ToRad(tmp_psi_L/100.0));
					tmp_pf_offset_NED.z = tmp_pf_offset.z;
				//Subtract offset distance vector from leader distance vector to get modified relative distance vector to leader
				if(tmp_d2L<(_chi*100))	//Near-field regime condition
				{
					tmp_dL -=tmp_pf_offset_NED;
					_phi_a.x = 2*(_lambda.x/100.0)*(tmp_dL.x/100.0);			//Calculate Quad PF X gradient with weighting parameter (NED frame)
					_phi_a.y = 2*(_lambda.y/100.0)*(tmp_dL.y/100.0);			//Calculate Quad PF Y gradient with weighting parameter (NED frame)
					_phi_a.z = 2*(_lambda.z/100.0)*(tmp_dL.z/100.0);			//Calculate Quad PF Z gradient with weighting parameter (NED frame)
				}
				else
				{
					float tmp_M =2*safe_sqrt(square(_lambda.x/100.0)+square(_lambda.y/100.0)+square(_lambda.z/100.0))*(_chi);		//Slope of linear potential field (Magnitude of gradient) = nominal magnitude of quadratic gradient at chi
					float tmp_R = safe_sqrt(square((_lambda.x/100.0)*(tmp_dL.x/100.0))+square((_lambda.y/100.0)*(tmp_dL.y/100.0))+square((_lambda.z/100.0)*(tmp_dL.z/100)));
					_phi_a.x = tmp_M*square(_lambda.x/100.0)*(tmp_dL.x/100.0)/tmp_R;			//Calculate Lin PF X gradient with weighting parameter (NED frame)
					_phi_a.y = tmp_M*square(_lambda.y/100.0)*(tmp_dL.y/100.0)/tmp_R;			//Calculate Lin PF Y gradient with weighting parameter (NED frame)
					_phi_a.z = tmp_M*square(_lambda.z/100.0)*(tmp_dL.z/100.0)/tmp_R;			//Calculate Lin PF Z gradient with weighting parameter (NED frame)
				}
				//Calculate the total potential gradient components
				_phi_NED = _phi_a + _phi_r;

				

				//Calculate the magnitude of the total potential function gradient
				if(_phi_NED.length() > 0) //divide-by-zero check (safety first)
				{
					//Calculate the normalized potential gradient components
					_Nphi_NED = _phi_NED.normalized(); 
					if(tmp_d2L<_chi) //near-field consideration- We don't want the VWP behind the body
					{
						_regime_mask = 0x01;	//Make first bit in regime mask 1 to reflect that we're in near-field regime
						//Find X component in Local Navigation frame by rotating NED vector about heading
						_Nphi_l.x = _Nphi_NED.x*cos(ToRad(tmp_psi_ac/100.0)) + _Nphi_NED.y*sin(ToRad(tmp_psi_ac/100.0));
						//Correct normalized vector for VWP in near-field
						if(_Nphi_l.x<0)		//Vector is pointing behind ac in Local Navigation frame
						{
							//Rotate NED pf gradiend components into Local Navigation components
							_phi_l.x = _phi_NED.x*cos(ToRad(tmp_psi_ac/100.0)) + _phi_NED.y*sin(ToRad(tmp_psi_ac/100.0));
							_phi_l.y = - _phi_NED.x*sin(ToRad(tmp_psi_ac/100.0)) + _phi_NED.y*cos(ToRad(tmp_psi_ac/100.0));
							//Correct potential function to point forward
							/*	Note:
								Corrected by using airspeed instead of relative distance... almost a predictor of where the goal "will" be in a second
							*/
							float airspeed_est;
							if(ahrs->airspeed_estimate(&airspeed_est))
							{
								_phi_l.x = 2*(_lambda.x/100.0)*(airspeed_est);
							}
							else
							{
								_phi_l.x = 2*(_lambda.x/100.0)*10; //Not the most elegant way to do this... 10 is an average airspeed for the skysurfer
							}
							//Corrected potential rotated back into NED frame from Local Navigation frame
							_phi_c_NED.x= _phi_l.x*cos(ToRad(tmp_psi_ac/100.0)) - _phi_l.y*sin(ToRad(tmp_psi_ac/100.0));
							_phi_c_NED.y= _phi_l.x*sin(ToRad(tmp_psi_ac/100.0)) + _phi_l.y*cos(ToRad(tmp_psi_ac/100.0));
							//Normalize the corrected potential gradient vector
							_Nphi_c_NED = _phi_c_NED.normalized();
							//Modify regime mask to reflect that both the first bit and the second bit are 1 (near-field regime, and using a corrected 
							_regime_mask = 0x03;
						}
						//Make sure regime mask reflects that only 
						else _regime_mask = 0x01;
					}
					else
					{
						//Regime mask should reflect that we are not in the near-field, and we therefore aren't using a corrected potential field
						_regime_mask = 0x00;
					}
				}
				else
				{
					//If the magnitude of the gradient is not greater than 0, make the norm 0 to avoid divide by 0 problems 
					_Nphi_NED.zero();
					_Nphi_c_NED.zero();
				}

				float tmp_dNorth_com;
				float tmp_dEast_com;
				float tmp_dalt_com;
				float tmp_dspd_com;


				if(tmp_d2L<_chi)
				{
					tmp_dVaspd = tmp_dV.x*cos(ToRad(tmp_psi_ac)) + tmp_dV.y*sin(ToRad(tmp_psi_ac));
					//Calculate the North and East Offset [m]
					tmp_dNorth_com	=	_VWP_offset*_Nphi_c_NED.x;
					tmp_dEast_com		=	_VWP_offset*_Nphi_c_NED.y;
					//Calculate the change in altitude [m]
					//k_alt_V is the equivalent of a derivative gain- set to 0 for simplicity.
					//tmp_dalt_com	=	_VWP_Z_offset*_Nphi_NED.z;  
					tmp_dalt_com = tmp_dL.z/100.0;
					//Calculate the change in airspeed [m/s]
					//k_phi_V_near is the equivalent of an integrator gain, while k_V_near is the equivalent of a proportional gain
					tmp_dspd_com=(_k_phi_V_near/100.0)*_Nphi_l.x + (_k_V_near/100.0)*tmp_dVaspd; //[m/s]
					//Convert to units used by WP and airspeed commands
					tmp_dspd_com= tmp_dspd_com*100; //[cm/s]
					_phi_norm = _Nphi_c_NED;
				}
				else
				{								//Far-Field Case
					tmp_dNorth_com	=	_VWP_offset*_Nphi_NED.x;
					tmp_dEast_com	=	_VWP_offset*_Nphi_NED.y;
					//tmp_dalt_com	=	_VWP_Z_offset*_Nphi_NED.z;
					tmp_dalt_com = tmp_dL.z/100.0;
					tmp_dspd_com	=	0;
					_phi_norm = _Nphi_NED;
				}
				
					const Location* p_current_location = p_ac->get_loc();
					_next_VWP = *p_current_location;
					location_offset(&_next_VWP, tmp_dNorth_com, tmp_dEast_com);
					_next_VWP.alt+=(int32_t)(tmp_dalt_com*100); //double-check sign here.
					constrain(_next_VWP.alt,PFG_MIN_ALT,PFG_MAX_ALT);

				if(tmp_d2L<_chi)
				{							//Near-Field Case
					float airspeed_ret;
					ahrs->airspeed_estimate(&airspeed_ret);
					_next_airspeed_com = (uint16_t)(airspeed_ret*100+tmp_dspd_com); //converted to uint16_t with units of (cm/s)
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

const uint16_t* pf_field::get_new_speed(){
	return (const uint16_t*)&_next_airspeed_com;
}

const Vector3f* pf_field::get_pfg_att(){
	return (const Vector3f*)&_phi_a;
}

const Vector3f* pf_field::get_pfg_rep(){
	return (const Vector3f*)&_phi_r;
}

const Vector3f* pf_field::get_pfg_norm(){
	return (const Vector3f*)&_phi_norm;
}

uint8_t pf_field::get_regime_mask(){
	return _regime_mask;
}

pf_field PF_FIELD;

