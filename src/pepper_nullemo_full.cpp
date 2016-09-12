// ----------------------------------------------------------------------------------------------------------------------------
//
// Program to compute the inverse kinematics of Pepper for the wrists and the head, without modeling the hands
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


#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <visualization_msgs/MarkerArray.h>

#include <pepper_nullemo/geometry_msgs_pose_vector.h>

#include <pepper_core.hpp>
#include <pepper_chain_init.hpp>
#include <pepper_ik_algs_full.hpp>
#include <pepper_trajectories_data.hpp>
#include <pepper_nullemo_tasks.hpp>
#include <pepper_emotional.hpp>


#define RECURSIVE_MOVING_AVERAGE 2
#define MEAN_WINDOW_SIZE 1
#define N_RECURSIVE_LAYERS 1

#define LOOP_FREQUENCY 100
#define REAL_PEPPER_LOOP_FREQUENCY 4.45


using namespace Eigen;



// ROS Subscribers --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool motion_initiated = false;
unsigned int trajectory_id = 0;
void PepperGeneratePosesTrajectoryCallback(const std_msgs::Int32 msg){
  
  trajectory_id = msg.data;
  
  motion_initiated = true;
  
  ROS_INFO("pepper_emonull: Pepper poses trajectory selected. Trajectory Id = %d", trajectory_id);
  
  return;
}


double desired_pepper_poses[PEPPER_N_CHAINS*7], past_desired_pepper_poses[PEPPER_N_CHAINS*7]; // Position & Quaternion
// [pos_head, quat_head, pos_rightarm, quat_rightarm, pos_leftarm, quat_leftarm] = [ {px, py, pz, qx, qy, qz, qw} ]
unsigned int dimension_selection[PEPPER_N_CHAINS*6];
// void dimensionSelectionCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
double dimension_selection_frames[PEPPER_N_CHAINS*8];
// 3 (head, right arm, left arm) x 2 (position, orientation) x [qx qy qz qw]
double desired_gazed_position[3];


int main(int argc, char** argv){
  
  // Initialize random seed:
  srand (time(NULL));
  
  
  double period = 1.0/(double)LOOP_FREQUENCY;
  
  double all_time = GetTimeMs64();
  double alg_time = 0;    
  
  
  // ROS -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  ros::init(argc, argv, "pepper_emonull");

  ros::NodeHandle node;
  ros::Rate loop_rate(LOOP_FREQUENCY);

  //   Topics
  ros::Subscriber poses_trajectories_sub = node.subscribe("pepper_generate_poses_trajectory", 10, &PepperGeneratePosesTrajectoryCallback);
  
  ros::Publisher pepper_platform_2Dpose_pub = node.advertise<geometry_msgs::Vector3>("pepper_platform_2Dpose", 1);
  ros::Publisher pepper_joint_pub = node.advertise<sensor_msgs::JointState>("pepper_joint_angles", 1);
  ros::Publisher pepper_pose_trajectory_pub = node.advertise<pepper_nullemo::geometry_msgs_pose_vector>("pepper_poses_trajectory", 1);

  //   Markers
  ros::Publisher marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("pepper_object_marker_array", 1);  
  
  
  // Variables ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  load_pepper_joint_values();
  
  
  // Robot variables
  Eigen::VectorXd past_configuration_inc(PEPPER_TOTAL_N_JOINTS_SIMP);
  
  double pepper_platform_2Dpose[3], past_pepper_platform_2Dpose[3];
  //   0:      'Pos X'
  //   1:      'Pos Y'
  //   2:      'Rotation around Z axis'
  for (unsigned int i=0; i<3; i++)		pepper_platform_2Dpose[i] = past_pepper_platform_2Dpose[i] = 0.0;
  
  double pepper_joints[PEPPER_TOTAL_N_JOINTS];	// rad
  //   0-1:    'HeadYaw', 'HeadPitch',
  //   2-3:    'HipRoll', 'HipPitch',
  //   4:      'KneePitch',
  //   5-10:   'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand',
  //   11-16:  'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 
  //   17-22:  'RFinger41', 'LFinger42', 'RFinger11', 'RFinger12', 'LFinger33', 'RFinger32', 
  //   23-28:  'LFinger21', 'RFinger43', 'LFinger13', 'LFinger32', 'LFinger11', 'RFinger22',
  //   29-34:  'LFinger41', 'RFinger13', 'LFinger22', 'RFinger21', 'LFinger12', 'RFinger23', 
  //   35-40:  'LFinger23', 'LFinger43', 'RFinger42', 'LFinger31', 'RFinger33', 'RFinger31',
  //   41-44:  'LThumb1', 'RThumb1', 'RThumb2', 'LThumb2',
  //   45-47:  'WheelFL', 'WheelB', 'WheelFR'
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)		pepper_joints[i] = 0.0;
  
  double real_pepper_joints[17];	// rad
  //   0-1:    'HeadYaw', 'HeadPitch',
  //   2-3:    'HipRoll', 'HipPitch',
  //   4:      'KneePitch',
  //   5-10:   'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand',
  //   11-16:  'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 
  for (unsigned int i=0; i<17; i++)		real_pepper_joints[i] = 0.0;
  
  double real_pepper_platform[3];
  // X, Y, Theta, ROBOT_FRAME
  real_pepper_platform[0] =   real_pepper_platform[1] =   real_pepper_platform[2] = 0.0;

  double current_configuration[PEPPER_TOTAL_N_JOINTS_SIMP];
  //   0:'Pos X', 1:'Pos Y', 2:'Rotation around Z axis'
  //   Joint angles
  double past_current_configuration[PEPPER_TOTAL_N_JOINTS_SIMP], past_past_current_configuration[PEPPER_TOTAL_N_JOINTS_SIMP];
  
  Eigen::VectorXd configuration_inc(PEPPER_TOTAL_N_JOINTS_SIMP), configuration_inc_aux(PEPPER_TOTAL_N_JOINTS_SIMP);  
  
  double righthand_jnt_val, lefthand_jnt_val;
  righthand_jnt_val = lefthand_jnt_val = 0.5;
  
  double current_poses[PEPPER_N_CHAINS*7];
  for (unsigned int i=0; i<PEPPER_N_CHAINS*7; i++)	current_poses[i] = 0.0; 
  

  // Task variables
  for (unsigned int i=0; i<PEPPER_N_CHAINS*7; i++)	desired_pepper_poses[i] = past_desired_pepper_poses[i] = 0.0;  
  
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	dimension_selection_frames[i] = 0.0;
  dimension_selection_frames[3]  = dimension_selection_frames[7]  = 1.0;	// Head
  dimension_selection_frames[11] = dimension_selection_frames[15] = 1.0;	// Right arm
  dimension_selection_frames[19] = dimension_selection_frames[23] = 1.0;	// Left arm
  
  desired_gazed_position[0] = desired_gazed_position[1] = desired_gazed_position[2] = -1.0;  // Default gaze  
  
  
  // PAD model variables
  double PAD[3];	// PAD = [P, A, D]: PAD[i] \in [-1,1]
  PAD[0] = PAD[1] = PAD[2] = 0.0;
  
  
  // Emotional conveyance variables
  double emotion_configuration[PEPPER_TOTAL_N_JOINTS_SIMP], past_emotion_configuration[PEPPER_TOTAL_N_JOINTS_SIMP];
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	emotion_configuration[i] = past_emotion_configuration[i] = 0.0;
  double extension, jerkiness, symmetry, dominance;
  extension = jerkiness = symmetry = dominance = 0.5;

  
  // Init emotional data
  emotionalInitData emo_data;
  initialize_emotional_data(&emo_data);

  // Symmetry 
  double sym_A[5], sym_w[5];
  for (unsigned int i=0; i<5; i++){
    sym_A[i] = emo_data.sym_A_min + emo_data.sym_A_span/2.0;
    sym_w[i] = emo_data.sym_T_min + emo_data.sym_T_span/2.0;
  }
  double sym_sin[5], sym_sin_past[5];
  for (unsigned int i=0; i<5; i++)	sym_sin[i] = 0.0;
  for (unsigned int i=0; i<5; i++)	sym_sin_past[i] = 0.0;

  
  // IK algorithm constants
  double Kp[2];
  Kp[0] = Kp[1] = 1.0;
  double Km = 1.0;
    
  
  // ROS
  geometry_msgs::Vector3 rosPepperPlatform2DPose;
  rosPepperPlatform2DPose.x = 0.0;
  rosPepperPlatform2DPose.y = 0.0;
  rosPepperPlatform2DPose.z = 0.0;

  sensor_msgs::JointState rosPepperJoints;
  rosPepperJoints.position.resize(PEPPER_TOTAL_N_JOINTS);
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)		rosPepperJoints.position[i] = 0.0;

  //   Markers
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  
  
  // KDL
  KDL::JntArray* config_chains = new KDL::JntArray[PEPPER_N_CHAINS];
  config_chains[0] = KDL::JntArray(PEPPER_HEAD_CHAIN_JOINTS);
  config_chains[1] = KDL::JntArray(PEPPER_RIGHTARM_CHAIN_JOINTS);
  config_chains[2] = KDL::JntArray(PEPPER_LEFTARM_CHAIN_JOINTS);
  config_chains[3] = KDL::JntArray(PEPPER_TORSO_CHAIN_JOINTS);
  config_chains[4] = KDL::JntArray(PEPPER_RIGHTELBOW_CHAIN_JOINTS);
  config_chains[5] = KDL::JntArray(PEPPER_LEFTELBOW_CHAIN_JOINTS);
  
  KDL::Chain* pepper_tree_chains = new KDL::Chain[PEPPER_N_CHAINS];
  initialize_kinematic_chains(pepper_tree_chains);
  
  //   Forward kinematics solvers
  KDL::ChainFkSolverPos_recursive** fk_solvers = new KDL::ChainFkSolverPos_recursive*[PEPPER_N_CHAINS];
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	  fk_solvers[i] = new KDL::ChainFkSolverPos_recursive(pepper_tree_chains[i]);
  
  //   Jacobian computation
  KDL::Jacobian* KDL_jacobians = new KDL::Jacobian[PEPPER_N_CHAINS];
  KDL_jacobians[0] = KDL::Jacobian(PEPPER_HEAD_CHAIN_JOINTS);
  KDL_jacobians[1] = KDL::Jacobian(PEPPER_RIGHTARM_CHAIN_JOINTS);
  KDL_jacobians[2] = KDL::Jacobian(PEPPER_LEFTARM_CHAIN_JOINTS);
  KDL_jacobians[3] = KDL::Jacobian(PEPPER_TORSO_CHAIN_JOINTS);
  KDL_jacobians[4] = KDL::Jacobian(PEPPER_RIGHTELBOW_CHAIN_JOINTS);
  KDL_jacobians[5] = KDL::Jacobian(PEPPER_LEFTELBOW_CHAIN_JOINTS);  
  
  KDL::ChainJntToJacSolver** jacobian_solvers = new KDL::ChainJntToJacSolver*[PEPPER_N_CHAINS];
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	  jacobian_solvers[i] = new KDL::ChainJntToJacSolver(pepper_tree_chains[i]);

  // Error computation
  unsigned int error_alg_id = 0;;
  Eigen::Vector3d* alg_error_p = new Eigen::Vector3d[PEPPER_N_TCP];
  Eigen::Vector3d* alg_error_w = new Eigen::Vector3d[PEPPER_N_TCP]; 
    
 
  // Task
  double task_pose[2];
  task_pose[0] = task_pose[1] = 0.0;
  
  
  // Task error
  bool gather_task_error = false;

  unsigned int n_ik_tp_alg = 0;  
  
  Eigen::MatrixXd *pose_errors_norm_errors = new Eigen::MatrixXd[n_ik_tp_alg];
  for (int i = 0; i < n_ik_tp_alg; i++)		pose_errors_norm_errors[i]= Eigen::MatrixXd::Zero(3,2);		// Position & Orientation
  
  Eigen::MatrixXd *task_priority_errors = new Eigen::MatrixXd[n_ik_tp_alg];
  for (int i = 0; i < n_ik_tp_alg; i++)		task_priority_errors[i]= Eigen::MatrixXd::Zero(3,18);		// Errors in N x SE(3) or q_m
  Eigen::MatrixXd *task_priority_norm_errors = new Eigen::MatrixXd[n_ik_tp_alg];
  for (int i = 0; i < n_ik_tp_alg; i++)		task_priority_norm_errors[i]= Eigen::MatrixXd::Zero(3,2);	// Position & Orientation
  
  double emotion_errors[2];
  
  //   Singular singular_values
  //     0 - First  task jacobian
  //     1 - Second task jacobian
  //     2 - Second task projected jacobian
  //     3 - Third task jacobian
  //     4 - Third task projected jacobian
  Eigen::MatrixXd singular_values(5,PEPPER_TOTAL_N_JOINTS_SIMP);
  singular_values.setZero();

  
  // Statistical data
  double time_init = 0.0;  
  statAccount timeLoad[4];
  double alg_joint[n_ik_tp_alg];
  for (unsigned int i=0; i<n_ik_tp_alg; i++)	alg_joint[i] = 0.0;
  
  
  // File data
  bool write_in_files = false; 
  std::ofstream file_exp("/home/joseparnau/nullemo_data/nullemo.dat", std::ios::out);
  std::ofstream file_alg_times("/home/joseparnau/nullemo_data/ne_alg_times.dat", std::ios::out);
  //   Joint trajectories and derivatives
  std::ofstream file_algs_jnt("/home/joseparnau/nullemo_data/algs_joint.dat", std::ios::out);
  std::ofstream file_jnt_pos("/home/joseparnau/nullemo_data/joint_pos.dat", std::ios::out);
  std::ofstream file_jnt_vel("/home/joseparnau/nullemo_data/joint_vel.dat", std::ios::out);
  std::ofstream file_jnt_acc("/home/joseparnau/nullemo_data/joint_acc.dat", std::ios::out);
  // Errors
  std::ofstream file_errors_tp1("/home/joseparnau/nullemo_data/errors_tp1.dat", std::ios::out);
  std::ofstream file_task_smooth("/home/joseparnau/nullemo_data/task_smooth.dat", std::ios::out);
  std::ofstream file_emot_errors("/home/joseparnau/nullemo_data/task_emotional_errors.dat", std::ios::out);
  
  std::ofstream file_jnt_csv("/home/joseparnau/nullemo_data/angleLists.csv", std::ios::out);
  std::ofstream file_time_csv("/home/joseparnau/nullemo_data/timeLists.csv", std::ios::out);
  std::ofstream file_platform_csv("/home/joseparnau/nullemo_data/platformLists.csv", std::ios::out);
  
  bool write_to_csv = false;
  unsigned int n_csv_written = 0;
  double csv_period = 1.0/(double)REAL_PEPPER_LOOP_FREQUENCY;
  
  // Initialization -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  unsigned int priority_distribution_id = 0;

  
  // Set initial configuration
  set_home_configuration(current_configuration);
  righthand_jnt_val = lefthand_jnt_val = 0.5;  
  
  // DATA xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  write_in_files = false;  
  gather_task_error = false;
  
  write_to_csv = true;
  
  error_alg_id = 1;
  priority_distribution_id = 3;
  
  PAD[0] = 0.0;		// P
  PAD[1] = 0.0;		// A
  PAD[2] = 0.0;		// D
  
  // Emotional conveyance variables
//   extension   = 0.5 - 0.5*cos(0.5*emo_data.w_estension*alg_time);
  extension = 1.0;
  jerkiness = 0.0;
//   symmetry  = 0.3 + 0.4 * extension;
  symmetry  = 0.0;
  dominance = 0.0;

  // IK algorithm constants
  Kp[0] = 1.0;
  Kp[1] = 1.0;
  Km = 0.1;
  
  unsigned int i_alg_joint = 8;
  // DATA xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  
  
  
  // Loop ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  unsigned int n_it = 0;
  while (ros::ok()){
    
    if (motion_initiated){
      
      double time0 = GetTimeMs64();	// IK Time computation
      double time_aux = 0.0;
      
      // Update
      n_it++;
      if (n_it == 1)	alg_time = 0;
      else		alg_time += (GetTimeMs64() - all_time)/1000.0;
      all_time  =  GetTimeMs64();	// ms

      
      // Set initial configurations *******************************************************************************************************************************************
      if (n_it == 1)
      {
	time_init = GetTimeMs64();
	
	if        ( trajectory_id == RIGHTARM_WAVING )
	{
	  // Left arm
#if PEPPER_PLATFORM_N_DOF == 3
	  current_configuration[13] = PI/2;
	  current_configuration[14] = PI/8;
	  current_configuration[16] = -PI/16;
#elif PEPPER_PLATFORM_N_DOF == 0
	  current_configuration[1] = PI/16;
	  current_configuration[6] = -PI/8;
	  current_configuration[7] = 5*PI/8;
	  current_configuration[8] = 3*PI/8; 
	  current_configuration[9] = -3*PI/8;	  
	  current_configuration[10] = PI/2;
	  current_configuration[11] = PI/8;
	  current_configuration[13] = -PI/16; 
#endif
	}	
	else if ( ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR ) || 
	          ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY ) )
	{
	  // Left arm( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR )
	  current_configuration[13] = 54.0*(PI/180.0);
	  current_configuration[14] = 0.0;
	  current_configuration[15] = -PI/2;
	  current_configuration[16] = -PI/4;
	  current_configuration[17] = -PI/2;;
	}
      }
      if (n_it == 1)	for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		past_past_current_configuration[i] = current_configuration[i];
      else		for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		past_past_current_configuration[i] = past_current_configuration[i];
      for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)					past_current_configuration[i] = current_configuration[i];
      for (unsigned int i=0; i<PEPPER_N_TCP*7; i++)						past_desired_pepper_poses[i] = desired_pepper_poses[i];     
      
    
      
      // Direct kinematics ********************************************************************************************************************************************
      compute_poses_from_config(fk_solvers, current_configuration, current_poses);
      
      // New task poses and/or gaze ***********************************************************************************************************************************
      compute_next_task(current_poses, desired_pepper_poses, dimension_selection, dimension_selection_frames, desired_gazed_position, alg_time, trajectory_id);
      task_pose[0] = sqrt( desired_pepper_poses[0]*desired_pepper_poses[0] + desired_pepper_poses[1]*desired_pepper_poses[1] + desired_pepper_poses[2]*desired_pepper_poses[2] );
      task_pose[1] = sqrt( desired_pepper_poses[0]*desired_pepper_poses[0] + desired_pepper_poses[1]*desired_pepper_poses[1] + desired_pepper_poses[2]*desired_pepper_poses[2] );
      
      // Compute emotional next configuration
      for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		past_emotion_configuration[i] = emotion_configuration[i];
      compute_emotional_configuration(current_configuration, current_poses, fk_solvers, emotion_configuration, desired_pepper_poses, righthand_jnt_val, lefthand_jnt_val, desired_gazed_position, 
				      period, alg_time, extension, jerkiness, symmetry, dominance, sym_sin, sym_sin_past, sym_A, sym_w);
      if (n_it == 1)	for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		past_emotion_configuration[i] = emotion_configuration[i];
      
     
      // Error computation ********************************************************************************************************************************************
      compute_errors_from_poses(current_poses, desired_pepper_poses, past_desired_pepper_poses, Kp, alg_error_p, alg_error_w, period, error_alg_id);
      
      // Jacobians ****************************************************************************************************************************************************
      compute_jacobians_from_config(config_chains, jacobian_solvers, current_configuration, KDL_jacobians);
      
      // Inverse kinematics *******************************************************************************************************************************************
      double next_configuration[PEPPER_TOTAL_N_JOINTS_SIMP];
     
//       //   SICILIANO
//       time_aux = GetTimeMs64();
//       compute_next_configuration(current_configuration, dimension_selection, dimension_selection_frames, alg_error_p, alg_error_w, KDL_jacobians, Km, emotion_configuration, past_emotion_configuration, priority_distribution_id, n_it, period, 
// 				 configuration_inc, gather_task_error, desired_pepper_poses, fk_solvers, task_priority_errors[0], task_priority_norm_errors[0], pose_errors_norm_errors[0], emotion_errors, singular_values, next_configuration, 0);
//       timeLoad[0+1].computeNextStatistics(GetTimeMs64() - time_aux);
//       alg_joint[0] = next_configuration[i_alg_joint];
      
//       //   SICILIANO RECURSIVE
//       time_aux = GetTimeMs64();
//       compute_next_configuration(current_configuration, dimension_selection, dimension_selection_frames, alg_error_p, alg_error_w, KDL_jacobians, Km, emotion_configuration, past_emotion_configuration, priority_distribution_id, n_it, period, 
// 				 configuration_inc, gather_task_error, desired_pepper_poses, fk_solvers, task_priority_errors[1], task_priority_norm_errors[1], pose_errors_norm_errors[1], emotion_errors, singular_values, next_configuration, 1);      
//       timeLoad[1+1].computeNextStatistics(GetTimeMs64() - time_aux);
//       alg_joint[1] = next_configuration[i_alg_joint];

      
//       for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		emotion_configuration[i] = 0;
//       emotion_configuration[10] = PI/2;
//       emotion_configuration[11] = PI/8;
//       emotion_configuration[13] = -PI/16;
      
      //   CHIAVERINI
      time_aux = GetTimeMs64();
      compute_next_configuration(current_configuration, dimension_selection, dimension_selection_frames, alg_error_p, alg_error_w, KDL_jacobians, Km, emotion_configuration, past_emotion_configuration, priority_distribution_id, n_it, period, 
				 configuration_inc, gather_task_error, desired_pepper_poses, fk_solvers, task_priority_errors[2], task_priority_norm_errors[2], pose_errors_norm_errors[2], emotion_errors, singular_values, next_configuration, 2);       
      timeLoad[2+1].computeNextStatistics(GetTimeMs64() - time_aux);
      alg_joint[2] = next_configuration[i_alg_joint];
      
      for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		current_configuration[i] = next_configuration[i];
      
      
      // Data **********************************************************************************************************************************************************
      
      // IK Time computation
      timeLoad[0].computeNextStatistics(GetTimeMs64() - time0);
//       if (write_in_files)	
	file_exp << GetTimeMs64() - time0 << std::endl;
      if (write_in_files){
	for (unsigned int i=0; i<1+n_ik_tp_alg; i++)	file_alg_times << std::setw(15) << timeLoad[i].get_last_value();	file_alg_times << std::endl;
      }
      
      // Joint trajectories
      if (write_in_files){
	for (unsigned int i=0; i<n_ik_tp_alg; i++)	file_algs_jnt << std::setw(15) << alg_joint[i];		file_algs_jnt << std::endl;
	
	for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	file_jnt_pos << std::setw(15) << current_configuration[i] << " ";      												file_jnt_pos << std::endl;
	for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	file_jnt_vel << std::setw(15) << (current_configuration[i]-past_current_configuration[i])/period << " ";							file_jnt_vel << std::endl;
	for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	file_jnt_acc << std::setw(15) << (current_configuration[i]-2.0*past_current_configuration[i]+past_past_current_configuration[i])/(period*period) << " ";	file_jnt_acc << std::endl; 
      }
      
      // Errors
      if (write_in_files){
	// Main task
	for (unsigned int i=0; i<n_ik_tp_alg; i++)	file_errors_tp1 << std::setw(15) << pose_errors_norm_errors[i](0,0) << std::setw(15) << pose_errors_norm_errors[i](0,1);	file_errors_tp1 << std::endl;
	// Gaze
	
	// Extent & Jerkiness: H
	file_emot_errors << std::setw(15) << emotion_errors[1] << std::endl;	
      }
      
      // Task
      if (write_in_files){
	for (unsigned int i=0; i<n_ik_tp_alg; i++)	file_task_smooth << std::setw(15) << task_pose[0] << std::setw(15) << task_pose[1];	file_task_smooth << std::endl;
      }
    }

    //   To ROS format
    for (unsigned int i=0; i<3; i++)	past_pepper_platform_2Dpose[i] = pepper_platform_2Dpose[i];
    pepper3ChainModelConfig_to_RosConfig(current_configuration, pepper_platform_2Dpose, pepper_joints);


    // Pepper hands  ---------------------------------------------------------------------------------------------------------------------------------------
    if      ( (trajectory_id == RIGHTARM_WAVING) || 
              (trajectory_id == RIGHTARM_WAVING_RELAXED) || 
              (trajectory_id == RIGHTARM_WAVING_RELAXED_LOOK) )				righthand_jnt_val = 0.85;
    
    else if ( (trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS) || 
	      (trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS_XY) )			righthand_jnt_val = lefthand_jnt_val = 0.5;
    
    else if ( (trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR) || 
	      (trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY) )		lefthand_jnt_val = 0.05;
    
    else if ( (trajectory_id == RIGHTHAND_SHAKE) || 
	      (trajectory_id == RIGHTHAND_SHAKE_POS_Z) || 
	      (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y) || 
	      (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_RIGHTARM_YZ) || 
	      (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ) || 
	      (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR) || 
	      (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WX_WY_RIGHTARM_YZ_OR) || 
	      (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ_HEAD_WY_WZ) )		righthand_jnt_val = 0.625;
    
    else if ( (trajectory_id == OBJECT_BOTH_HANDS) ||
	      (trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD) ||
	      (trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD_HANDS_POS_GRASPCOORD) )	righthand_jnt_val = lefthand_jnt_val = 0.85;

    //   Right hand
    pepper_joints[17] = pepper_joints[19] = pepper_joints[20] = pepper_joints[22] = pepper_joints[24] = pepper_joints[28] = pepper_joints[30] = 
    pepper_joints[32] = pepper_joints[34] = pepper_joints[37] = pepper_joints[39] = pepper_joints[40] = pepper_joints[42] = pepper_joints[43] = righthand_jnt_val;
    //   Left hand
    pepper_joints[18] = pepper_joints[21] = pepper_joints[23] = pepper_joints[25] = pepper_joints[26] = pepper_joints[27] = pepper_joints[29] = 
    pepper_joints[31] = pepper_joints[33] = pepper_joints[35] = pepper_joints[36] = pepper_joints[38] = pepper_joints[41] = pepper_joints[44] = lefthand_jnt_val;

    
    // Markers
    marker_array.markers.resize(1);
    
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time::now();
    marker.ns = "pepper_objects";
    marker.lifetime = ros::Duration();
    
    marker.id = 0;
    marker.header.frame_id = "/platf_coordX_link";;
    marker.type = visualization_msgs::Marker::SPHERE;     
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.pose.position.x = desired_gazed_position[0];
    marker.pose.position.y = desired_gazed_position[1];
    marker.pose.position.z = desired_gazed_position[2];
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = (  0.0f/255.0f);
    marker.color.g = (255.0f/255.0f);
    marker.color.b = (255.0f/255.0f);
    marker.color.a = 1.0;
    marker_array.markers[0] = marker; 
    
    
    
//     // Set configuration
//     //   0-1:    'HeadYaw', 'HeadPitch',
//     pepper_joints[0] = 0.0;
//     pepper_joints[1] = 0.0;
//     //   2-3:    'HipRoll', 'HipPitch',
//     pepper_joints[2] = 0.0;
//     pepper_joints[3] = 0.0;
//     //   4:      'KneePitch',
//     pepper_joints[4] = 0.0;
//     //   5-10:   'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand',
//     pepper_joints[5] = PI/2;
//     pepper_joints[6] = PI/8;
//     pepper_joints[7] = 0.0;
//     pepper_joints[8] = -PI/16;
//     pepper_joints[9] = 0.0;
//     pepper_joints[10] = 0.5;
//     //   11-16:  'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand', 
//     pepper_joints[11] = 0.0 + PI/16*sin(150.0*(PI/180.0) * alg_time);
//     pepper_joints[12] = -PI/8 - PI/32*sin(150.0*(PI/180.0) * alg_time);
//     pepper_joints[13] = PI/2 + PI/4*sin(150.0*(PI/180.0) * alg_time);
//     pepper_joints[14] = PI/2-PI/16;// - PI/8*sin(150.0*(PI/180.0) * alg_time);
//     pepper_joints[15] = -PI/2+PI/8 + PI/8*sin(150.0*(PI/180.0) * alg_time);
//     pepper_joints[16] = 1.0;
//     //   17-22:  'RFinger41', 'LFinger42', 'RFinger11', 'RFinger12', 'LFinger33', 'RFinger32', 
//     //   23-28:  'LFinger21', 'RFinger43', 'LFinger13', 'LFinger32', 'LFinger11', 'RFinger22',
//     //   29-34:  'LFinger41', 'RFinger13', 'LFinger22', 'RFinger21', 'LFinger12', 'RFinger23', 
//     //   35-40:  'LFinger23', 'LFinger43', 'RFinger42', 'LFinger31', 'RFinger33', 'RFinger31',
//     //   41-44:  'LThumb1', 'RThumb1', 'RThumb2', 'LThumb2',
//     pepper_joints[17] = pepper_joints[19] = pepper_joints[20] = pepper_joints[22] = pepper_joints[24] = pepper_joints[28] = pepper_joints[30] = 
//     pepper_joints[32] = pepper_joints[34] = pepper_joints[37] = pepper_joints[39] = pepper_joints[40] = pepper_joints[42] = pepper_joints[43] = pepper_joints[16];
//     pepper_joints[18] = pepper_joints[21] = pepper_joints[23] = pepper_joints[25] = pepper_joints[26] = pepper_joints[27] = pepper_joints[29] = 
//     pepper_joints[31] = pepper_joints[33] = pepper_joints[35] = pepper_joints[36] = pepper_joints[38] = pepper_joints[41] = pepper_joints[44] = pepper_joints[10];
//     //   45-47:  'WheelFL', 'WheelB', 'WheelFR'
//     pepper_joints[45] = pepper_joints[46] = pepper_joints[47] = 0.0;
//     // Platform
//     pepper_platform_2Dpose[0]= pepper_platform_2Dpose[1] = pepper_platform_2Dpose[2] = 0.0;
    
    
    
    // Real Pepper ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //   Joints
    for (unsigned int i=0; i<17; i++)	real_pepper_joints[i] = pepper_joints[i];
    real_pepper_joints[16] = righthand_jnt_val;
    real_pepper_joints[10] = 0.5;
    //   Platform
    double theta = past_pepper_platform_2Dpose[2];
    Eigen::Matrix3d platform_rotation_world2robotframe = Eigen::Matrix3d::Identity();
    platform_rotation_world2robotframe(0,0) = platform_rotation_world2robotframe(1,1) = cos(theta);
    platform_rotation_world2robotframe(0,1) = -sin(theta);
    platform_rotation_world2robotframe(1,0) = sin(theta);
    platform_rotation_world2robotframe(0,2) = past_pepper_platform_2Dpose[0];
    platform_rotation_world2robotframe(1,2) = past_pepper_platform_2Dpose[1];
    Eigen::Vector3d platform_nextpose_worldframe(pepper_platform_2Dpose[0], pepper_platform_2Dpose[1], 1.0);
    Eigen::Vector3d platform_nextpose_robotframe = platform_rotation_world2robotframe.inverse() * platform_nextpose_worldframe;
    real_pepper_platform[0] = (platform_nextpose_robotframe(0))/period;
    real_pepper_platform[1] = (platform_nextpose_robotframe(1))/period;
    real_pepper_platform[2] = (pepper_platform_2Dpose[2] - past_pepper_platform_2Dpose[2])/period;
//     while (real_pepper_platform[2] > PI)	real_pepper_platform[2] -= 2.0*PI;
//     while (real_pepper_platform[2] <= PI)	real_pepper_platform[2] += 2.0*PI;    
       
    

    // Publish -----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    //   Pepper joints
    //     Platform
    rosPepperPlatform2DPose.x = pepper_platform_2Dpose[0];
    rosPepperPlatform2DPose.y = pepper_platform_2Dpose[1];
    rosPepperPlatform2DPose.z = pepper_platform_2Dpose[2];
    pepper_platform_2Dpose_pub.publish(rosPepperPlatform2DPose);
    //     Joints
    rosPepperJoints.header.stamp = ros::Time::now();
    for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)	rosPepperJoints.position[i] = pepper_joints[i];
    pepper_joint_pub.publish(rosPepperJoints);

    //   Pepper poses
    pepper_nullemo::geometry_msgs_pose_vector rosPepperPoses;
    for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)
    {
      geometry_msgs::Pose ROSPoseAux;
      ROSPoseAux.position.x = desired_pepper_poses[7*i + 0];
      ROSPoseAux.position.y = desired_pepper_poses[7*i + 1];
      ROSPoseAux.position.z = desired_pepper_poses[7*i + 2];
      ROSPoseAux.orientation.x = desired_pepper_poses[7*i + 3];
      ROSPoseAux.orientation.y = desired_pepper_poses[7*i + 4];
      ROSPoseAux.orientation.z = desired_pepper_poses[7*i + 5];
      ROSPoseAux.orientation.w = desired_pepper_poses[7*i + 6];
      rosPepperPoses.pose_vector.push_back(ROSPoseAux);
    }
    pepper_pose_trajectory_pub.publish(rosPepperPoses);    
    
    //   Markers
    marker_array_pub.publish(marker_array);
      
    
    ros::spinOnce(); 
    loop_rate.sleep();
    
    
    if ( (n_it > 1) && ( (GetTimeMs64() - time_init)/1000.0 > 30 ) )		break;
    
    // CSV files
    if ( write_to_csv && motion_initiated ){
//       std::cout << n_csv_written << "/" << n_csv_written*csv_period << " - ";
      if ( trunc(alg_time/csv_period) > n_csv_written ){
	n_csv_written++;
//  	std::cout << "t = " << alg_time << ": trunc = " << trunc(alg_time/csv_period) <<" - n_w = " << n_csv_written << std::endl;
	// Time
	if (n_csv_written > 1)		file_time_csv << ",";
	file_time_csv << n_csv_written*csv_period;      
	// Joints
	for (unsigned int i=0; i<17; i++){
	  if ( (i == 2) || (i == 3) || (i == 4) )	file_jnt_csv << -real_pepper_joints[i];
	  else						file_jnt_csv << real_pepper_joints[i];
	  if (i < 17-1)	file_jnt_csv << ",";
	}
	file_jnt_csv << std::endl;
	// Platform 2D pose
	file_platform_csv << real_pepper_platform[0] << "," << real_pepper_platform[1] << "," << real_pepper_platform[2] << std::endl;
      }
    }
  }
  
  
  std::cout << "ST: " << n_it << " :  " << (GetTimeMs64() - time_init)/1000.0 << " s   /   " << timeLoad[0].get_average() << "  -  " 
	    << timeLoad[0].get_stdev() << " ms"<< std::endl;  
  for (unsigned int i=1; i<4; i++)	std::cout << "S" << i+1 << ": " << timeLoad[i].get_average() << "  -  " << timeLoad[i].get_stdev() << " ms"<< std::endl; 
  
  file_exp.close();
  file_alg_times.close();
  file_algs_jnt.close();
  file_jnt_pos.close();
  file_jnt_vel.close();
  file_jnt_acc.close();
  file_errors_tp1.close();
  file_task_smooth.close();
  file_emot_errors.close();
  file_jnt_csv.close();
  file_time_csv.close();
  file_platform_csv.close();
  
  return 0;
}