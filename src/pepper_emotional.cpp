// ----------------------------------------------------------------------------------------------------------------------------
//
// Code with the emotional function
// 
//     Copyright (C) <year>  <name of author>
// 
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
// 
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
// 
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// For comments, questions, suggestions and bugs, contact the author: 
//	Josep-Arnau Claret Robert (joseparnau81@gmail.com)
//
// ----------------------------------------------------------------------------------------------------------------------------


#include <pepper_emotional.hpp>


void initialize_emotional_data(emotionalInitData *emo_data){

//   double cont_cfg[PEPPER_TOTAL_N_JOINTS_SIMP], ext_cfg[PEPPER_TOTAL_N_JOINTS_SIMP], KAmp[PEPPER_TOTAL_N_JOINTS_SIMP];
//   for (unsigned int i=0; i<3; i++)		cont_cfg[i] = ext_cfg[i] = KAmp[i] = 0.0;
  
  double Kw = 1.0;
  
  // Extension ------------------------------------------------------------
  emo_data->w_estension  =  Kw * (2.0*PI)/2.25;
  // EXTENT = 0 
  unsigned int n_jnt = PLATFORM_JNT_TORSO_1;
  //   Torso
  emo_data->cont_cfg[n_jnt++] =  0.1;
  emo_data->cont_cfg[n_jnt++] =  0.1;
  emo_data->cont_cfg[n_jnt++] =  0.0;
  //   Head
  emo_data->cont_cfg[n_jnt++] =  0.0;
  emo_data->cont_cfg[n_jnt++] =  0.7;
  //   Right arm
  emo_data->cont_cfg[n_jnt++] =  0.9;
  emo_data->cont_cfg[n_jnt++] = -0.3;
  emo_data->cont_cfg[n_jnt++] =  0.0;
  emo_data->cont_cfg[n_jnt++] =  1.1;
  emo_data->cont_cfg[n_jnt++] =  1.0;
  //   Left arm
  emo_data->cont_cfg[n_jnt++] =  0.9;
  emo_data->cont_cfg[n_jnt++] =  0.3;
  emo_data->cont_cfg[n_jnt++] =  0.0;
  emo_data->cont_cfg[n_jnt++] = -1.1;
  emo_data->cont_cfg[n_jnt++] = -1.0;  
  
  // EXTENT = 1
  n_jnt = PLATFORM_JNT_TORSO_1;
  //   Torso
  emo_data->ext_cfg[n_jnt++]  =  0.0;
  emo_data->ext_cfg[n_jnt++]  = -0.05;
  emo_data->ext_cfg[n_jnt++]  =  0.0;
  //   Head
  emo_data->ext_cfg[n_jnt++]  =  0.0;
  emo_data->ext_cfg[n_jnt++]  = -0.1;
  //   Right arm
  emo_data->ext_cfg[n_jnt++]  =  1.57;
  emo_data->ext_cfg[n_jnt++]  = -0.85;
  emo_data->ext_cfg[n_jnt++]  =  1.50;
  emo_data->ext_cfg[n_jnt++]  =  0.1;
  emo_data->ext_cfg[n_jnt++]  =  1.15;
  //   Left arm
  emo_data->ext_cfg[n_jnt++]  =  1.57;
  emo_data->ext_cfg[n_jnt++]  =  0.85;
  emo_data->ext_cfg[n_jnt++]  = -1.50;
  emo_data->ext_cfg[n_jnt++]  = -0.1;
  emo_data->ext_cfg[n_jnt++]  = -1.15;  
  
  // AMPLITUDE
  n_jnt = PLATFORM_JNT_TORSO_1;
  //   Torso
  emo_data->KAmp[n_jnt++] =  0.0;
  emo_data->KAmp[n_jnt++] =  0.0;
  emo_data->KAmp[n_jnt++] =  0.15;
  //   Head
  emo_data->KAmp[n_jnt++] =  0.45;
  emo_data->KAmp[n_jnt++] =  0.0;
  //   Right arm			// Experiments lab	// Original
  emo_data->KAmp[n_jnt++] =  0.0;
  emo_data->KAmp[n_jnt++] =  0.45;	//0.35; //0.65;
  emo_data->KAmp[n_jnt++] =  0.7;
  emo_data->KAmp[n_jnt++] =  0.3;
  emo_data->KAmp[n_jnt++] = -0.2;
  //   Left arm				// Experiments lab	// Original
  emo_data->KAmp[n_jnt++] =  0.0;
  emo_data->KAmp[n_jnt++] =  0.45;	// 0.35; //-0.65;
  emo_data->KAmp[n_jnt++] = -0.7;
  emo_data->KAmp[n_jnt++] = -0.3;
  emo_data->KAmp[n_jnt++] =  0.2;   
  
  // Jerkiness ------------------------------------------------------------
						// Experiments lab	// Original
  emo_data->jerky_w_sin = Kw * 2.0*(2*PI)/1.0;	// Kw * 2.0*(2*PI)/1.0;	// Kw * (2*PI)/1.0;
  emo_data->jerky_w_cos = Kw * 2.5*(2*PI)/1.0;	// Kw * 2.5*(2*PI)/1.0;	// Kw * 1.5*(2*PI)/1.0;
  emo_data->jerky_A_sin = 1.5/6.0;		// 1.0/6.0;		// 1.0/6.0;
  emo_data->jerky_A_cos = 1.5/6.0;		// 1.0/6.0;		// 1.0/4.0;
  
  // Symmetry -------------------------------------------------------------
  emo_data->sym_A_min  = 1.5;
  emo_data->sym_A_span = 1.5;
  emo_data->sym_T_min  = 6.0;
  emo_data->sym_T_span = 4.0;
  
  // Hands ----------------------------------------------------------------
  emo_data->hand_jnt_val_min  = 0.2;
  emo_data->hand_jnt_val_span = 0.6;  
  
  return;
}



void compute_emotional_configuration(double* current_configuration, double* current_poses,
				     KDL::ChainFkSolverPos_recursive** fk_solvers, 
				     double* emotion_configuration, double* desired_poses, double &righthand_jnt_val, double &lefthand_jnt_val, double *desired_gazed_position, 
				     const double period, const double alg_time,
				     const double extension, const double jerkiness, const double symmetry, const double dominance,
				     double* sym_sin, double* sym_sin_past, double* sym_A, double* sym_w
				    )
{
  
   // ****************************************************************************************************************************************************************************************
  // EMOTION CONVEYANCE *********************************************************************************************************************************************************************
  
  // Initialization --------------------------------------------------------------------------------------------------------------------------------------

  // Init emotional data
  emotionalInitData emo_data;
  initialize_emotional_data(&emo_data);

//   // Symmetry 
//   double sym_A[5], sym_w[5];
//   for (unsigned int i=0; i<5; i++){
//     sym_A[i] = emo_data.sym_A_min + emo_data.sym_A_span/2.0;
//     sym_w[i] = emo_data.sym_T_min + emo_data.sym_T_span/2.0;
//   }
  
//   double  sym_sin[5], sym_sin_past[5];
//   for (unsigned int i=0; i<5; i++)	sym_sin[i] = sym_sin_past[i] = 0.0;
  
  // Emotion function ------------------------------------------------------------------------------------------------------------------------------------
  for (unsigned int i=PLATFORM_JNT_TORSO_1; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)
  {
//     unsigned int i_sym = 0;			// Non-symmetric parts
//     if      ( i == PLATFORM_JNT_TORSO_3 )							i_sym = 1;	// Torso
//     else if ( i == PLATFORM_JNT_HEAD_1 )							i_sym = 2;	// Neck
//     else if ( (i >= PLATFORM_JNT_RIGHT_ARM_1) && (i <= PLATFORM_JNT_RIGHT_ARM_5) )		i_sym = 3;	// Right arm
//     else if ( (i >= PLATFORM_JNT_LEFT_ARM_1)  && (i <= PLATFORM_JNT_LEFT_ARM_5) )		i_sym = 4;	// Left arm
//     
//     // Symmetry +++++++++++++++++++++++++++++++++++++++++++++++++++	// TODO TODO TODO TODO TODO TODO TODO TODO
//     sym_sin[i_sym] += period;
//     if (i == PLATFORM_JNT_HEAD_1){
// //       std::cout << " ----------------------------------------------------------------------------------- " << std::endl;
// //       std::cout << i << " / " << i_sym << " t: " << sym_sin_past[i_sym] << " / " << sym_sin[i_sym] << std::endl;
// //       std::cout << "s: " << sin(sym_w[i_sym]*sym_sin_past[i_sym]) << "  / " << sin(sym_w[i_sym]*sym_sin[i_sym]) << std::endl;
//     }
//     if (  ( sin(sym_w[i_sym]*sym_sin_past[i_sym]) <=  0.0)  &&  ( sin(sym_w[i_sym]*sym_sin[i_sym]) > 0.0)  )
//     {
//       sym_A[i_sym]   =            emo_data.sym_A_min + emo_data.sym_A_span * rnd0to1();
//       sym_w[i_sym]   = (2.0*PI)/( 3.0 );//emo_data.sym_T_min + emo_data.sym_T_span * rnd0to1() );
// //       sym_sin[i_sym] = 0.0;
//       if (i == PLATFORM_JNT_HEAD_1){
// // 	std::cout << i << " / " << i_sym << std::endl;
// // 	std::cout << "AAAA" << std::endl;
// 	std::cout << i << " / " << i_sym 
// 		  << " -  t: " << sym_sin_past[i_sym] << " / " << sym_sin[i_sym] 
// 		  << " -  s: " << sin(sym_w[i_sym]*sym_sin_past[i_sym]) << " / " << sin(sym_w[i_sym]*sym_sin[i_sym]) 
// 		  << " -  sA: " << sym_A[2] << "    sw: " << sym_w[2] << std::endl;
//       }
//     }
//     std::cout << "s2: " << sym_sin[2] << std::endl; 
//     sym_sin_past[i_sym] = sym_sin[i_sym];
//     double sym_offset = (1.0 - symmetry) * sym_A[i_sym] * sin( sym_w[i_sym] * sym_sin[i_sym] );
    
    
    unsigned int i_sym = 0;			// Non-symmetric parts
    double sym_offset = 0.0;
    if ( i == PLATFORM_JNT_HEAD_1 ){
      i_sym = 2;	// Neck

      // Symmetry +++++++++++++++++++++++++++++++++++++++++++++++++++	// TODO TODO TODO TODO TODO TODO TODO TODO
      sym_sin[i_sym] += period;
//       std::cout << i << " / " << i_sym << " -  t: " << sym_sin_past[i_sym] << " / " << sym_sin[i_sym]
// 		     << " -  w: " << sym_w[i_sym]
// 		     << " -  s: " << sin(sym_w[i_sym]*sym_sin_past[i_sym]) << " / " << sin(sym_w[i_sym]*sym_sin[i_sym]) << std::endl;
//       std::cout << " ----------------------------------------------------------------------------------- " << std::endl;
//       std::cout << i << " / " << i_sym << " t: " << sym_sin_past[i_sym] << " / " << sym_sin[i_sym] << std::endl;
//       std::cout << "s: " << sin(sym_w[i_sym]*sym_sin_past[i_sym]) << "  / " << sin(sym_w[i_sym]*sym_sin[i_sym]) << std::endl;
      if (  ( sin(sym_w[i_sym]*sym_sin_past[i_sym]) <= 0.0)  &&  ( sin(sym_w[i_sym]*sym_sin[i_sym]) > 0.0)  )
      {
	std::cout << " ----------------------------------------------------------------------------------- " << std::endl;
	std::cout << "T: " << alg_time
		  << " -  t: " << sym_sin_past[i_sym] << " / " << sym_sin[i_sym] 
		  << " -  s: " << sin(sym_w[i_sym]*sym_sin_past[i_sym]) << " / " << sin(sym_w[i_sym]*sym_sin[i_sym]) 
		  << std::endl;
	sym_A[i_sym] =            emo_data.sym_A_min + emo_data.sym_A_span * rnd0to1();
	sym_w[i_sym] = (2.0*PI)/( emo_data.sym_T_min + emo_data.sym_T_span * rnd0to1() );
	sym_sin[i_sym] = period;
// 	std::cout << i << " / " << i_sym << std::endl;
// 	std::cout << "AAAA" << std::endl;
	std::cout << " -  sA: " << sym_A[i_sym] << "    sw: " << (2.0*PI)/sym_w[i_sym] << std::endl;

      }
//       std::cout << "s2: " << sym_sin[2] << std::endl; 
      sym_sin_past[i_sym] = sym_sin[i_sym];
      sym_offset = (1.0 - symmetry) * sym_A[i_sym] * sin( sym_w[i_sym] * sym_sin[i_sym] );
    }

    
    // Jerkiness +++++++++++++++++++++++++++++++++++++++++++++++++++
    double jerky_offset = jerkiness * ( emo_data.jerky_A_sin * sin( emo_data.jerky_w_sin * alg_time + sym_offset) + emo_data.jerky_A_cos * cos( emo_data.jerky_w_cos * alg_time  + sym_offset) );
    
    // Extension +++++++++++++++++++++++++++++++++++++++++++++++++++
#if PEPPER_PLATFORM_N_DOF == 3
//     // Symmetry in the extension
//     if ((i == 5) || (i == 6))	emotion_configuration[i] = ( 1.0 - extension ) * emo_data.cont_cfg[i]  +  extension * ( emo_data.ext_cfg[i] + (1.0 - symmetry) * emo_data.KAmp[i] * sin( emo_data.w_estension * alg_time + jerky_offset + sym_offset ) );
//     else			emotion_configuration[i] = ( 1.0 - extension ) * emo_data.cont_cfg[i]  +  extension * ( emo_data.ext_cfg[i] +                    emo_data.KAmp[i] * sin( emo_data.w_estension * alg_time + jerky_offset + sym_offset ) );
    // Symmetry in the jerkyness and the head only
    if ((i == 5) || (i == 6))	
		emotion_configuration[i] = ( 1.0 - extension ) * emo_data.cont_cfg[i]  +  extension * ( emo_data.ext_cfg[i] + (1.0 - symmetry) * emo_data.KAmp[i] * sin( emo_data.w_estension * alg_time + jerky_offset + sym_offset ) );
    else	emotion_configuration[i] = ( 1.0 - extension ) * emo_data.cont_cfg[i]  +  extension * ( emo_data.ext_cfg[i] +                    emo_data.KAmp[i] * sin( emo_data.w_estension * alg_time + jerky_offset ) );
//     std::cout << "i = " << i << " -  cfg = " << emo_data.cont_cfg[i] << std::endl;
#elif PEPPER_PLATFORM_N_DOF == 0
    //  Symmetry in the jerkyness and the head only
    if ((i == PLATFORM_JNT_HEAD_1) || (i == PLATFORM_JNT_HEAD_2)){
      jerky_offset = jerkiness * ( emo_data.jerky_A_sin * sin( emo_data.jerky_w_sin * alg_time + sym_offset) + emo_data.jerky_A_cos * cos( emo_data.jerky_w_cos * alg_time  + sym_offset) );
      emotion_configuration[i] = ( 1.0 - extension ) * emo_data.cont_cfg[i]  +  extension * ( emo_data.ext_cfg[i] + (1.0 - symmetry) * emo_data.KAmp[i] * sin( emo_data.w_estension * alg_time + jerky_offset + sym_offset ) );
    }
    else{
      jerky_offset = jerkiness * ( emo_data.jerky_A_sin * sin( emo_data.jerky_w_sin * alg_time)              + emo_data.jerky_A_cos * cos( emo_data.jerky_w_cos * alg_time) );
      emotion_configuration[i] = ( 1.0 - extension ) * emo_data.cont_cfg[i]  +  extension * ( emo_data.ext_cfg[i] +                    emo_data.KAmp[i] * sin( emo_data.w_estension * alg_time + jerky_offset ) );
    }
#endif
  }
  righthand_jnt_val = lefthand_jnt_val = emo_data.hand_jnt_val_min + emo_data.hand_jnt_val_span * extension;
  
  
  // Gaze ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Eigen::Matrix3d domHeadRotMat;
  domHeadRotMat.col(0) << Eigen::Vector3d(desired_gazed_position[0] - current_poses[0], desired_gazed_position[1] - current_poses[1], desired_gazed_position[2] - current_poses[2]).normalized();  
  domHeadRotMat.col(1) << Eigen::Vector3d::UnitZ().cross(domHeadRotMat.col(0));
  domHeadRotMat.col(2)  = domHeadRotMat.col(0).cross(domHeadRotMat.col(1));
  Eigen::Quaternion<double> domGazeRot = Eigen::Quaternion<double>(domHeadRotMat);
  
  double pepper_pose_aux[7];
  compute_pose_from_config(fk_solvers, emotion_configuration, pepper_pose_aux, 0);
  
  Eigen::Quaternion<double> gaze_quat = Eigen::Quaternion<double>(pepper_pose_aux[6], pepper_pose_aux[3], pepper_pose_aux[4], pepper_pose_aux[5]).slerp(dominance, Eigen::Quaternion<double>(domHeadRotMat));
  desired_poses[3] = gaze_quat.x();
  desired_poses[4] = gaze_quat.y();
  desired_poses[5] = gaze_quat.z();
  desired_poses[6] = gaze_quat.w();
 
    
  return;
}