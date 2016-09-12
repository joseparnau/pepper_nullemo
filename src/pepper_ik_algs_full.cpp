// ----------------------------------------------------------------------------------------------------------------------------
//
// Code with the task priority formulations
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



#include <pepper_ik_algs_full.hpp>
#include <pepper_trajectories_data.hpp>
#include <pepper_emotional.hpp>

#include <sstream>
#include <iostream>
#include <iomanip>

#include <Eigen/SparseCore>


void compute_next_configuration(double* current_configuration, 
				unsigned int* dimension_selection, double* dimension_selection_frames,
				Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, KDL::Jacobian* KDL_jacobians,
				double Km, double* emotion_configuration, double* past_emotion_configuration,
				const unsigned int priority_distribution_id,
				const unsigned int n_it, const double period,
				Eigen::VectorXd &configuration_inc,
				const bool gather_task_error,
				double* desired_poses,
				KDL::ChainFkSolverPos_recursive** fk_solvers,
				Eigen::MatrixXd &task_priority_errors, Eigen::MatrixXd &task_priority_norm_errors, Eigen::MatrixXd &pose_errors_norm_errors, double *emotion_errors,
				Eigen::MatrixXd &singular_values,
				double* next_configuration,
				const unsigned int ik_tp_alg_formulation
			       ){
   
  switch(ik_tp_alg_formulation)
  {
	case 0:	// Siciliano
	{
	  compute_next_configuration_siciliano_taskpriority(current_configuration, dimension_selection, dimension_selection_frames, alg_error_p, alg_error_w, KDL_jacobians, Km, emotion_configuration, past_emotion_configuration,
							    priority_distribution_id, n_it, configuration_inc, gather_task_error, task_priority_errors, task_priority_norm_errors, pose_errors_norm_errors, emotion_errors, singular_values);
	  break;
	}
	
	case 1:	// Siciliano Recursive
	{
	  compute_next_configuration_siciliano_recursive_taskpriority(current_configuration, dimension_selection, dimension_selection_frames, alg_error_p, alg_error_w, KDL_jacobians, Km, emotion_configuration, past_emotion_configuration,
								      priority_distribution_id, n_it, configuration_inc, gather_task_error, task_priority_errors, task_priority_norm_errors, pose_errors_norm_errors, emotion_errors, singular_values);
	  break;
	}
	
	case 2:	// Chiaverini
	{
	  compute_next_configuration_chiaverini_taskpriority(current_configuration, dimension_selection, dimension_selection_frames, alg_error_p, alg_error_w, KDL_jacobians, Km, emotion_configuration, past_emotion_configuration,
							     priority_distribution_id, n_it, configuration_inc, gather_task_error, task_priority_errors, task_priority_norm_errors, pose_errors_norm_errors, emotion_errors, singular_values);
	  break;
	}
  }
  
  // Boundary checking ********************************************************************************************************************************************
  saturate_joint_velocity(configuration_inc, period);
  
//   if (n_it == 1)	past_configuration_inc = configuration_inc;  
//   saturate_joint_acceleration(configuration_inc, past_configuration_inc, period);
//   past_configuration_inc = configuration_inc;

  //configuration_inc(0) = configuration_inc(1) = configuration_inc(2) = 0.0;
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)    	next_configuration[i] = current_configuration[i] + configuration_inc(i);
  saturate_joint_angles(next_configuration);
  
  // DK Error computation *****************************************************************************************************************************************
  if (gather_task_error){
    
    double pose_aux[7];
    switch(priority_distribution_id)
    {
	case 3:
	{
	  // Wave
	  //   TP 1
	  compute_pose_from_config(fk_solvers, next_configuration, pose_aux, 1);	// RIGHT ARM
	  pose_errors_norm_errors(0,0) = sqrt( (desired_poses[7]-pose_aux[0])*(desired_poses[7]-pose_aux[0]) + 
					       (desired_poses[8]-pose_aux[1])*(desired_poses[8]-pose_aux[1]) + 
					       (desired_poses[9]-pose_aux[2])*(desired_poses[9]-pose_aux[2]) );
	  double angle_prov = Eigen::AngleAxis<double>(Eigen::Quaternion<double>(desired_poses[13], desired_poses[10], desired_poses[11], desired_poses[12]).toRotationMatrix() * 
						       Eigen::Quaternion<double>(pose_aux[6],       pose_aux[3],       pose_aux[4],       pose_aux[5]      ).toRotationMatrix().transpose()).angle();
	  while (angle_prov < -PI)	angle_prov += 2*PI;
	  while (angle_prov >= PI)	angle_prov -= 2*PI;
	  pose_errors_norm_errors(0,1) = angle_prov;

	  // Gaze
	  
	  // Emotion configuration error
	  emotion_errors[1] = 0.0;
	  double ang_error = 0.0;
	  for (unsigned int i=3; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++){
	    ang_error = emotion_configuration[i] - current_configuration[i];
	    while (ang_error >= PI)	ang_error -= 2*PI;
	    while (ang_error < -PI)	ang_error += 2*PI;
	    emotion_errors[1] += ang_error*ang_error;
	  }
	  emotion_errors[1] = sqrt(emotion_errors[1]);
	  
	  break;
	}
    }
  }
  
  
  return;
}



void compute_next_configuration_siciliano_taskpriority(double* current_configuration, 
						       unsigned int* dimension_selection, double* dimension_selection_frames,
						       Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, KDL::Jacobian* KDL_jacobians,
						       double Km, double* emotion_configuration, double* past_emotion_configuration,
						       const unsigned int priority_distribution_id,
						       const unsigned int n_it,
						       Eigen::VectorXd &configuration_inc,
						       const bool gather_task_error,
						       Eigen::MatrixXd &task_priority_errors, Eigen::MatrixXd & task_priority_norm_errors, Eigen::MatrixXd &pose_errors_norm_errors, double *emotion_errors,
						       Eigen::MatrixXd &singular_values
						      )
{
//  current_configuration
//     PLATFORM
//       0:      'Pos X'
//       1:      'Pos Y'
//       2:      'Rotation around Z axis'
//     PEPPER UPPER & TORSO
//       3:      'KneePitch',
//       4-5:    'HipRoll', 'HipPitch',
//     HEAD
//       6-7:    'HeadYaw', 'HeadPitch',
//     RIGHT ARM
//       8-12:   'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'
//     LEFT ARM
//       13-17:  'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'
// 
//  desired_poses: [{P_head, q_head}, {P_RightArm, q_RightArm}, {P_LeftArm, q_LeftArm}] = [ {px, py, pz, qx, qy, qz, qw} ]

  
 
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //  PRIORITY LEVEL - Start
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Eigen::VectorXd sing_values = singular_values.row(0);
  Eigen::VectorXd sing_values_2 = singular_values.row(0);
  
  // Siciliano
  switch(priority_distribution_id)
  {
	case 0:	// 1: HEAD, RIGHT ARM, LEFT ARM --------------------------------------------------------------------------------------------------------------------------------------------
	{
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)   = KDL_jacobians[0].data.block<6,6>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = KDL_jacobians[0].data.block<6,2>(0,6);		// HEAD
		jacobian.block<6,6>(6,0)   = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,8)   = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(12,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
															    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		// Inverse Kinematics
		configuration_inc = pinv( task_jacobian , 0.001, sing_values) * pose_error;
		singular_values.row(0) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.row(0) = pose_error - task_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();
		  task_priority_norm_errors(0,1) = ( task_priority_errors.block<1,3>(0,9) ).norm();
		}
		
		
		break;
	}
	
	
	case 1:	// 1: HEAD, RIGHT ARM, LEFT ARM; 2: Optimization function ------------------------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();

		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)   = KDL_jacobians[0].data.block<6,6>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = KDL_jacobians[0].data.block<6,2>(0,6);		// HEAD
		jacobian.block<6,6>(6,0)   = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,8)   = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(12,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM

		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
															    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];

		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;

		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;

		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level - OPTIMIZATION FUNCTION ++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd optfunc_error(PEPPER_TOTAL_N_JOINTS_SIMP);
		if (n_it == 1)		optfunc_error.setZero();
		else			for (unsigned int i=3; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	optfunc_error(i) = (emotion_configuration[i] - past_emotion_configuration[i]) + Km * ( emotion_configuration[i] - current_configuration[i] );
		optfunc_error(0) = optfunc_error(1) = optfunc_error(2) = 0.0;

		// Jacobian
		Eigen::MatrixXd optfunc_jacobian = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP);

// 		configuration_inc = configuration_inc + pinv( optfunc_jacobian * ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian ) , 0.001, sing_values) * ( optfunc_error - optfunc_jacobian * configuration_inc );
		configuration_inc = configuration_inc + pinv( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian, 0.001, sing_values) * ( optfunc_error - configuration_inc );	// pinv(optfunc_jacobian = Id	
		singular_values.row(1) = sing_values;


		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.row(0) = pose_error    - task_jacobian    * configuration_inc;
		  task_priority_errors.row(1) = optfunc_error - optfunc_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();
		  task_priority_norm_errors(0,1) = 0.5 * ( ( task_priority_errors.block<1,3>(0,3) ).norm() + ( task_priority_errors.block<1,3>(0,9) ).norm() );
		  task_priority_norm_errors(1,0) = ( task_priority_errors.block<1,15>(0,3) ).norm();
		}		
		
		
		break;
	}	
	
	
	case 2:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation -----------------------------------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  
		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();

		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)  = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,8)  = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(6,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM

		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
														    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];

		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;

		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;

		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::Vector3d pose_error_2(alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2));
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd task_jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_2.setZero();
		task_jacobian_2.block<3,8>(0,0) = KDL_jacobians[0].data.block<3,8>(3,0);
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		configuration_inc = configuration_inc + pinv( task_jacobian_2 * ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian ) , 0.001, sing_values) * ( pose_error_2 - task_jacobian_2 * configuration_inc );
		singular_values.row(1) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.block<1,6*2>(0,6) = pose_error   - task_jacobian   * configuration_inc;
		  task_priority_errors.block<1,3>(1,3)   = pose_error_2 - task_jacobian_2 * configuration_inc;
		}	
		
		
		break;
	}
	
	
	case 3:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation; 3: Optimization function ---------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)  = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,8)  = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(6,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
														    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;
		
		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::Vector3d pose_error_2(alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2));
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd task_jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_2.setZero();
		task_jacobian_2.block<3,8>(0,0) = KDL_jacobians[0].data.block<3,8>(3,0);
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		configuration_inc = configuration_inc + pinv( task_jacobian_2 * ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian ) , 0.001, sing_values) * ( pose_error_2 - task_jacobian_2 * configuration_inc );
		singular_values.row(1) = sing_values;


		// 3rd Priority Level - OPTIMIZATION FUNCTION +++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd optfunc_error(PEPPER_TOTAL_N_JOINTS_SIMP);
		if (n_it == 1)		optfunc_error.setZero();
		else			for (unsigned int i=3; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	optfunc_error(i) = (emotion_configuration[i] - past_emotion_configuration[i]) + Km * ( emotion_configuration[i] - current_configuration[i] );
		optfunc_error(0) = optfunc_error(1) = optfunc_error(2) = 0.0;
		
		// Jacobian
		Eigen::MatrixXd optfunc_jacobian = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP, PEPPER_TOTAL_N_JOINTS_SIMP);
		
		Eigen::MatrixXd task_jacobian_jacobian_1_2(15,PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_jacobian_1_2.block<12,PEPPER_TOTAL_N_JOINTS_SIMP>(0,0) = task_jacobian;
		task_jacobian_jacobian_1_2.block<3,PEPPER_TOTAL_N_JOINTS_SIMP>(12,0) = task_jacobian_2;
		
// 		configuration_inc = configuration_inc + pinv(  optfunc_jacobian * ( Eigen::MatrixXd::Identity(18,18) - pinv(task_jacobian_jacobian_1_2,0.001, sing_values) * task_jacobian_jacobian_1_2  ) , 0.001, sing_values_2) 
// 								* ( optfunc_error - optfunc_jacobian * configuration_inc );
		configuration_inc = configuration_inc + pinv( Eigen::MatrixXd::Identity(18,18) - pinv(task_jacobian_jacobian_1_2,0.001, sing_values) * task_jacobian_jacobian_1_2, 0.001, sing_values_2) 
								* ( optfunc_error - configuration_inc );	// pinv(optfunc_jacobian = Id	
									
		singular_values.row(3) = sing_values;
		singular_values.row(4) = sing_values_2;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.block<1,6*2>(0,6) = pose_error    - task_jacobian    * configuration_inc;
		  task_priority_errors.block<1,3>(1,3)   = pose_error_2  - task_jacobian_2  * configuration_inc;
		  task_priority_errors.row(2)            = optfunc_error - optfunc_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();	// Right arm Position Error Velocity
		  task_priority_norm_errors(0,1) = ( task_priority_errors.block<1,3>(0,9) ).norm();	// Right arm Orientation Error Velocity
		  task_priority_norm_errors(1,0) = ( task_priority_errors.block<1,3>(1,3) ).norm();	// Head Orientation Error Velocity
		  task_priority_norm_errors(1,0) =   task_priority_errors.row(2) 	   .norm();	// Emotion configuration velocity
		}
		
		
		break;
	}
  }

 
  return;
}



void compute_next_configuration_siciliano_recursive_taskpriority(double* current_configuration, 
								 unsigned int* dimension_selection, double* dimension_selection_frames,
								 Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, KDL::Jacobian* KDL_jacobians,
								 double Km, double* emotion_configuration, double* past_emotion_configuration,
								 const unsigned int priority_distribution_id,
								 const unsigned int n_it,
								 Eigen::VectorXd &configuration_inc,
								 const bool gather_task_error,
								 Eigen::MatrixXd &task_priority_errors, Eigen::MatrixXd & task_priority_norm_errors, Eigen::MatrixXd &pose_errors_norm_errors, double *emotion_errors,
								 Eigen::MatrixXd &singular_values
								)
{
//  current_configuration
//     PLATFORM
//       0:      'Pos X'
//       1:      'Pos Y'
//       2:      'Rotation around Z axis'
//     PEPPER UPPER & TORSO
//       3:      'KneePitch',
//       4-5:    'HipRoll', 'HipPitch',
//     HEAD
//       6-7:    'HeadYaw', 'HeadPitch',
//     RIGHT ARM
//       8-12:   'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'
//     LEFT ARM
//       13-17:  'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'
// 
//  desired_poses: [{P_head, q_head}, {P_RightArm, q_RightArm}, {P_LeftArm, q_LeftArm}] = [ {px, py, pz, qx, qy, qz, qw} ]

  
 
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //  PRIORITY LEVEL - Start
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Eigen::VectorXd sing_values = singular_values.row(0);
  
  // Siciliano Recursive
  switch(priority_distribution_id)
  {
	case 0:	// 1: HEAD, RIGHT ARM, LEFT ARM --------------------------------------------------------------------------------------------------------------------------------------------
	{
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)   = KDL_jacobians[0].data.block<6,6>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = KDL_jacobians[0].data.block<6,2>(0,6);		// HEAD
		jacobian.block<6,6>(6,0)   = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,8)   = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(12,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
															    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		// Inverse Kinematics
		configuration_inc = pinv( task_jacobian , 0.001, sing_values) * pose_error;
		singular_values.row(0) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.row(0) = pose_error - task_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();
		  task_priority_norm_errors(0,1) = ( task_priority_errors.block<1,3>(0,9) ).norm();
		}
		
		
		break;
	}
	
	
	case 1:	// 1: HEAD, RIGHT ARM, LEFT ARM; 2: Optimization function ------------------------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();

		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)   = KDL_jacobians[0].data.block<6,6>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = KDL_jacobians[0].data.block<6,2>(0,6);		// HEAD
		jacobian.block<6,6>(6,0)   = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,8)   = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(12,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM

		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
															    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];

		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;

		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;

		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level - OPTIMIZATION FUNCTION ++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd optfunc_error(PEPPER_TOTAL_N_JOINTS_SIMP);
		if (n_it == 1)		optfunc_error.setZero();
		else			for (unsigned int i=3; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	optfunc_error(i) = (emotion_configuration[i] - past_emotion_configuration[i]) + Km * ( emotion_configuration[i] - current_configuration[i] );
		optfunc_error(0) = optfunc_error(1) = optfunc_error(2) = 0.0;

		// Jacobian
		Eigen::MatrixXd optfunc_jacobian = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP);

// 		configuration_inc = configuration_inc + pinv( optfunc_jacobian * ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian ) , 0.001, sing_values) * ( optfunc_error - optfunc_jacobian * configuration_inc );
		configuration_inc = configuration_inc + pinv( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian, 0.001, sing_values) * ( optfunc_error - configuration_inc );	// pinv(optfunc_jacobian = Id
		singular_values.row(1) = sing_values;


		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.row(0) = pose_error    - task_jacobian    * configuration_inc;
		  task_priority_errors.row(1) = optfunc_error - optfunc_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();
		  task_priority_norm_errors(0,1) = 0.5 * ( ( task_priority_errors.block<1,3>(0,3) ).norm() + ( task_priority_errors.block<1,3>(0,9) ).norm() );
		  task_priority_norm_errors(1,0) = ( task_priority_errors.block<1,15>(0,3) ).norm();
		}		
		
		
		break;
	}	
	
	
	case 2:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation -----------------------------------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  
		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();

		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)  = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,8)  = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(6,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM

		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
														     dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];

		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;

		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;

		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::Vector3d pose_error_2(alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2));
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd task_jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_2.setZero();
		task_jacobian_2.block<3,8>(0,0) = KDL_jacobians[0].data.block<3,8>(3,0);
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		configuration_inc = configuration_inc + pinv( task_jacobian_2 * ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian ) , 0.001, sing_values) * ( pose_error_2 - task_jacobian_2 * configuration_inc );
		singular_values.row(1) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.block<1,6*2>(0,6) = pose_error   - task_jacobian   * configuration_inc;
		  task_priority_errors.block<1,3>(1,3)   = pose_error_2 - task_jacobian_2 * configuration_inc;
		}	
		
		
		break;
	}
	
	
	case 3:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation; 3: Optimization function ---------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)  = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,8)  = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(6,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
														     dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;
		
		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;

		Eigen::MatrixXd task_projection = Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian;

		
		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::Vector3d pose_error_2(alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2));
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd task_jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_2.setZero();
		task_jacobian_2.block<3,8>(0,0) = KDL_jacobians[0].data.block<3,8>(3,0);
		
		Eigen::MatrixXd task_jacobian_2_projection_1      = task_jacobian_2 * task_projection;
		Eigen::MatrixXd task_jacobian_2_projection_1_pinv = pinv( task_jacobian_2_projection_1 , 0.001, sing_values);
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		configuration_inc = configuration_inc + task_jacobian_2_projection_1_pinv * ( pose_error_2 - task_jacobian_2 * configuration_inc );
		singular_values.row(1) = sing_values;


		// 3rd Priority Level - OPTIMIZATION FUNCTION +++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd optfunc_error(PEPPER_TOTAL_N_JOINTS_SIMP);
		if (n_it == 1)		optfunc_error.setZero();
		else			for (unsigned int i=3; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	optfunc_error(i) = (emotion_configuration[i] - past_emotion_configuration[i]) + Km * ( emotion_configuration[i] - current_configuration[i] );
		optfunc_error(0) = optfunc_error(1) = optfunc_error(2) = 0.0;
		
		// Jacobian
		Eigen::MatrixXd optfunc_jacobian = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP, PEPPER_TOTAL_N_JOINTS_SIMP);
		
// 		configuration_inc = configuration_inc + pinv(  optfunc_jacobian * (task_projection - task_jacobian_2_projection_1_pinv * task_jacobian_2_projection_1) , 0.001, sing_values) * ( optfunc_error - optfunc_jacobian * configuration_inc );
		configuration_inc = configuration_inc + pinv(  task_projection - task_jacobian_2_projection_1_pinv * task_jacobian_2_projection_1 , 0.001, sing_values) * ( optfunc_error - configuration_inc );	// pinv(optfunc_jacobian = Id
		singular_values.row(3) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.block<1,6*2>(0,6) = pose_error    - task_jacobian    * configuration_inc;
		  task_priority_errors.block<1,3>(1,3)   = pose_error_2  - task_jacobian_2  * configuration_inc;
		  task_priority_errors.row(2)            = optfunc_error - optfunc_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();	// Right arm Position Error Velocity
		  task_priority_norm_errors(0,1) = ( task_priority_errors.block<1,3>(0,9) ).norm();	// Right arm Orientation Error Velocity
		  task_priority_norm_errors(1,0) = ( task_priority_errors.block<1,3>(1,3) ).norm();	// Head Orientation Error Velocity
		  task_priority_norm_errors(1,0) =   task_priority_errors.row(2) 	   .norm();	// Emotion configuration velocity
		}
		
		
		break;
	}
  }

 
  return;
}


void compute_next_configuration_chiaverini_taskpriority(double* current_configuration, 
							unsigned int* dimension_selection, double* dimension_selection_frames,
							Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, KDL::Jacobian* KDL_jacobians,
							double Km, double* emotion_configuration, double* past_emotion_configuration,
							const unsigned int priority_distribution_id,
							const unsigned int n_it,
							Eigen::VectorXd &configuration_inc,
							const bool gather_task_error,
							Eigen::MatrixXd &task_priority_errors, Eigen::MatrixXd & task_priority_norm_errors, Eigen::MatrixXd &pose_errors_norm_errors, double *emotion_errors,
							Eigen::MatrixXd &singular_values
						       )
{
//  current_configuration
//     PLATFORM
//       0:      'Pos X'
//       1:      'Pos Y'
//       2:      'Rotation around Z axis'
//     PEPPER UPPER & TORSO
//       3:      'KneePitch',
//       4-5:    'HipRoll', 'HipPitch',
//     HEAD
//       6-7:    'HeadYaw', 'HeadPitch',
//     RIGHT ARM
//       8-12:   'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'
//     LEFT ARM
//       13-17:  'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'
// 
//  desired_poses: [{P_head, q_head}, {P_RightArm, q_RightArm}, {P_LeftArm, q_LeftArm}] = [ {px, py, pz, qx, qy, qz, qw} ]

 
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //  PRIORITY LEVEL - Start
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Eigen::VectorXd sing_values = singular_values.row(0);
  
  // Siciliano Recursive
  switch(priority_distribution_id)
  {

	case 0:	// 1: HEAD, RIGHT ARM, LEFT ARM --------------------------------------------------------------------------------------------------------------------------------------------
	{
#if PEPPER_PLATFORM_N_DOF == 3
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)   = KDL_jacobians[0].data.block<6,6>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = KDL_jacobians[0].data.block<6,2>(0,6);		// HEAD
		jacobian.block<6,6>(6,0)   = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,8)   = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(12,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
															    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		// Inverse Kinematics
		configuration_inc = pinv( task_jacobian , 0.001, sing_values) * pose_error;
		singular_values.row(0) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.row(0) = pose_error - task_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();
		  task_priority_norm_errors(0,1) = ( task_priority_errors.block<1,3>(0,9) ).norm();
		}
#elif PEPPER_PLATFORM_N_DOF == 0
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,3>(0,0)   = KDL_jacobians[0].data.block<6,3>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = KDL_jacobians[0].data.block<6,2>(0,3);		// HEAD
		jacobian.block<6,3>(6,0)   = KDL_jacobians[1].data.block<6,3>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,5)   = KDL_jacobians[1].data.block<6,5>(0,3);		// RIGHT ARM
		jacobian.block<6,3>(12,0)  = KDL_jacobians[2].data.block<6,3>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,10) = KDL_jacobians[2].data.block<6,5>(0,3);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
															    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		// Inverse Kinematics
		configuration_inc = pinv( task_jacobian , 0.001, sing_values) * pose_error;
		singular_values.row(0) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.row(0) = pose_error - task_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();
		  task_priority_norm_errors(0,1) = ( task_priority_errors.block<1,3>(0,9) ).norm();
		}

#endif
		break;
	}

	
	
	case 1:	// 1: HEAD, RIGHT ARM, LEFT ARM; 2: Optimization function ------------------------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();

		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)   = KDL_jacobians[0].data.block<6,6>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = KDL_jacobians[0].data.block<6,2>(0,6);		// HEAD
		jacobian.block<6,6>(6,0)   = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,8)   = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(12,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM

		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>(dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
															    dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];

		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;

		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;

		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level - OPTIMIZATION FUNCTION ++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd optfunc_error(PEPPER_TOTAL_N_JOINTS_SIMP);
		if (n_it == 1)		optfunc_error.setZero();
		else			for (unsigned int i=3; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	optfunc_error(i) = (emotion_configuration[i] - past_emotion_configuration[i]) + Km * ( emotion_configuration[i] - current_configuration[i] );
		optfunc_error(0) = optfunc_error(1) = optfunc_error(2) = 0.0;

		// Jacobian
		Eigen::MatrixXd optfunc_jacobian = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP);

// 		configuration_inc = configuration_inc + ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian ) * pinv( optfunc_jacobian, 0.001, sing_values) * optfunc_error;
		configuration_inc = configuration_inc + ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian ) * optfunc_error;	// pinv(optfunc_jacobian = Id
		singular_values.row(1) = sing_values;


		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.row(0) = pose_error    - task_jacobian    * configuration_inc;
		  task_priority_errors.row(1) = optfunc_error - optfunc_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();
		  task_priority_norm_errors(0,1) = 0.5 * ( ( task_priority_errors.block<1,3>(0,3) ).norm() + ( task_priority_errors.block<1,3>(0,9) ).norm() );
		  task_priority_norm_errors(1,0) = ( task_priority_errors.block<1,15>(0,3) ).norm();
		}		
		
		
		break;
	}	
	
	
	case 2:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation -----------------------------------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  
		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();

		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)  = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,8)  = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(6,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM

		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
														     dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];

		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;

		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;

		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::Vector3d pose_error_2(alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2));
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd task_jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_2.setZero();
		task_jacobian_2.block<3,8>(0,0) = KDL_jacobians[0].data.block<3,8>(3,0);
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		configuration_inc = configuration_inc + ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian ) * pinv( task_jacobian_2, 0.001, sing_values) * pose_error_2;
		singular_values.row(1) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.block<1,6*2>(0,6) = pose_error   - task_jacobian   * configuration_inc;
		  task_priority_errors.block<1,3>(1,3)   = pose_error_2 - task_jacobian_2 * configuration_inc;
		}	
		
		
		break;
	}
	
#if PEPPER_PLATFORM_N_DOF == 3
	case 3:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation; 3: Optimization function ---------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)  = KDL_jacobians[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,8)  = KDL_jacobians[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(6,0)  = KDL_jacobians[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,13) = KDL_jacobians[2].data.block<6,5>(0,6);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
														     dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;
		
		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;

		Eigen::MatrixXd task_projection = Eigen::MatrixXd::Identity(18,18) - task_jacobian_pinv * task_jacobian;
		
		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::Vector3d pose_error_2(alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2));
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd task_jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_2.setZero();
		task_jacobian_2.block<3,8>(0,0) = KDL_jacobians[0].data.block<3,8>(3,0);
		Eigen::MatrixXd task_jacobian_2_pinv = pinv( task_jacobian_2 , 0.001, sing_values);
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		configuration_inc = configuration_inc + task_projection * task_jacobian_2_pinv * pose_error_2;
		singular_values.row(1) = sing_values;

		// 3rd Priority Level - OPTIMIZATION FUNCTION +++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd optfunc_error(PEPPER_TOTAL_N_JOINTS_SIMP);
		if (n_it == 1)		optfunc_error.setZero();
		else			for (unsigned int i=3; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	
		  optfunc_error(i) = (emotion_configuration[i] - past_emotion_configuration[i]) + Km * ( emotion_configuration[i] - current_configuration[i] );
// 		  optfunc_error(i) = Km * ( emotion_configuration[i] - current_configuration[i] );
		optfunc_error(0) = optfunc_error(1) = optfunc_error(2) = 0.0;
		
		// Jacobian
		Eigen::MatrixXd optfunc_jacobian = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP, PEPPER_TOTAL_N_JOINTS_SIMP);
		
// 		configuration_inc = configuration_inc + task_projection * ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_2_pinv * task_jacobian_2 ) * pinv(optfunc_jacobian, 0.001, sing_values) * optfunc_error;
		configuration_inc = configuration_inc + task_projection * ( Eigen::MatrixXd::Identity(18,18) - task_jacobian_2_pinv * task_jacobian_2 ) * optfunc_error;	// pinv(optfunc_jacobian = Id
		singular_values.row(3) = sing_values;

		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.block<1,6*2>(0,6) = pose_error    - task_jacobian    * configuration_inc;
		  task_priority_errors.block<1,3>(1,3)   = pose_error_2  - task_jacobian_2  * configuration_inc;
		  task_priority_errors.row(2)            = optfunc_error - optfunc_jacobian * configuration_inc;
		  
		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();	// Right arm Position Error Velocity
		  task_priority_norm_errors(0,1) = ( task_priority_errors.block<1,3>(0,9) ).norm();	// Right arm Orientation Error Velocity
		  task_priority_norm_errors(1,0) = ( task_priority_errors.block<1,3>(1,3) ).norm();	// Head Orientation Error Velocity
		  task_priority_norm_errors(1,0) =   task_priority_errors.row(2) 	   .norm();	// Emotion configuration velocity
		}
		
		break;
	}
	
#elif PEPPER_PLATFORM_N_DOF == 0
	case 3:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation; 3: Optimization function ---------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,3>(0,0)  = KDL_jacobians[1].data.block<6,3>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,5)  = KDL_jacobians[1].data.block<6,5>(0,3);		// RIGHT ARM
		jacobian.block<6,3>(6,0)  = KDL_jacobians[2].data.block<6,3>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,10)  = KDL_jacobians[2].data.block<6,5>(0,3);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
														     dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001, sing_values);
		singular_values.row(0) = sing_values;
		
		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;

		Eigen::MatrixXd task_projection = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP) - task_jacobian_pinv * task_jacobian;

		
		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::Vector3d pose_error_2(alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2));
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd task_jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_2.setZero();
		task_jacobian_2.block<3,5>(0,0) = KDL_jacobians[0].data.block<3,5>(3,0);
		Eigen::MatrixXd task_jacobian_2_pinv = pinv( task_jacobian_2 , 0.001, sing_values);
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		configuration_inc = configuration_inc + task_projection * task_jacobian_2_pinv * pose_error_2;
		singular_values.row(1) = sing_values;


		// 3rd Priority Level - OPTIMIZATION FUNCTION +++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::VectorXd optfunc_error(PEPPER_TOTAL_N_JOINTS_SIMP);
		if (n_it == 1)		optfunc_error.setZero();
		else			
		  for (unsigned int i=PEPPER_PLATFORM_N_DOF; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	
		    optfunc_error(i) = (emotion_configuration[i] - past_emotion_configuration[i]) + Km * ( emotion_configuration[i] - current_configuration[i] );
// 		    optfunc_error(i) = Km * ( emotion_configuration[i] - current_configuration[i] );
		optfunc_error(0) = optfunc_error(1) = optfunc_error(2) = 0.0;

		// Jacobian
		Eigen::MatrixXd optfunc_jacobian = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP, PEPPER_TOTAL_N_JOINTS_SIMP);
		
// 		configuration_inc = configuration_inc + task_projection * ( Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP) - task_jacobian_2_pinv * task_jacobian_2 ) * pinv(optfunc_jacobian, 0.001, sing_values) * optfunc_error;
// 		configuration_inc = configuration_inc + task_projection * ( Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP) - task_jacobian_2_pinv * taks_jacobian_2 ) * optfunc_error;	// pinv(optfunc_jacobian = Id
		configuration_inc = configuration_inc + ( task_projection - pinv(task_jacobian_2*task_projection, 0.001, sing_values) * (task_jacobian_2*task_projection)) * optfunc_error;
		singular_values.row(3) = sing_values;
		
		// Error gathering -----------------------------------------------------------------------------
		if (gather_task_error){
		  task_priority_errors.block<1,6*2>(0,6) = pose_error    - task_jacobian    * configuration_inc;
		  task_priority_errors.block<1,3>(1,3)   = pose_error_2  - task_jacobian_2  * configuration_inc;
		  task_priority_errors.row(2)            = optfunc_error - optfunc_jacobian * configuration_inc;

		  // Wave
		  task_priority_norm_errors(0,0) = ( task_priority_errors.block<1,3>(0,6) ).norm();	// Right arm Position Error Velocity
		  task_priority_norm_errors(0,1) = ( task_priority_errors.block<1,3>(0,9) ).norm();	// Right arm Orientation Error Velocity
		  task_priority_norm_errors(1,0) = ( task_priority_errors.block<1,3>(1,3) ).norm();	// Head Orientation Error Velocity
		  task_priority_norm_errors(1,0) =   task_priority_errors.row(2) 	   .norm();	// Emotion configuration velocity
		}
		
		break;
	}
#endif
	
  }


 
  return;
}