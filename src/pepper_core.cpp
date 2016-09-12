// ----------------------------------------------------------------------------------------------------------------------------
//
// Code with the basic functionalities of Pepper
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


#include <pepper_core.hpp>
#include <pepper_chain_init.hpp>
#include <pepper_nullemo_aux.hpp>

#include <stdlib.h>


void load_pepper_joint_values(void){
  
#if PEPPER_PLATFORM_N_DOF == 3
  // Joint limits ---------------------------------------------------------------------------------------------------------------------------------------
  
  //   MINIMUM
  //   Platform - No limits
  pepper_joint_limits_min[0]  = 0.0;
  pepper_joint_limits_min[1]  = 0.0;
  pepper_joint_limits_min[2]  = 0.0;
  //   Torso
  pepper_joint_limits_min[3]  = -0.05; //-0.514872;
  pepper_joint_limits_min[4]  = -0.05; //-1.03847;
  pepper_joint_limits_min[5]  = -0.514872;
  //   Head
  pepper_joint_limits_min[6]  = -2.08567;	// 0.0;// 
  pepper_joint_limits_min[7]  = -0.706858;
  //   Right Arm
  pepper_joint_limits_min[8]  = -2.08567;
  pepper_joint_limits_min[9]  = -1.56207;
  pepper_joint_limits_min[10] = -2.08567;
  pepper_joint_limits_min[11] = 0.00872665;
  pepper_joint_limits_min[12] = -1.82387;
  //   Left Arm
  pepper_joint_limits_min[13] = -2.08567;
  pepper_joint_limits_min[14] = 0.00872665;
  pepper_joint_limits_min[15] = -2.08567;
  pepper_joint_limits_min[16] = -1.56207;
  pepper_joint_limits_min[17] = -1.82387;

  //   MAXIMUM
  //   Platform - No limits
  pepper_joint_limits_max[0]  = 0.0;
  pepper_joint_limits_max[1]  = 0.0;
  pepper_joint_limits_max[2]  = 0.0;
  //   Torso
  pepper_joint_limits_max[3]  = 0.25; //0.514872;
  pepper_joint_limits_max[4]  = 0.25; //1.03847;
  pepper_joint_limits_max[5]  = 0.514872;
  //   Head
  pepper_joint_limits_max[6]  = 2.08567;		// 0.0;// 
  pepper_joint_limits_max[7]  = 0.706858;
  //   Right Arm
  pepper_joint_limits_max[8]  = 2.08567;
  pepper_joint_limits_max[9]  = -0.00872665;
  pepper_joint_limits_max[10] = 2.08567;
  pepper_joint_limits_max[11] = 1.56207;
  pepper_joint_limits_max[12] = 1.82387;
  //   Left Arm
  pepper_joint_limits_max[13] = 2.08567;
  pepper_joint_limits_max[14] = 1.56207;
  pepper_joint_limits_max[15] = 2.08567;
  pepper_joint_limits_max[16] = -0.00872665;
  pepper_joint_limits_max[17] = 1.82387;


  // Joint velocity limits ---------------------------------------------------------------------------------------------------------------------------------
  double Kv = 1.0;
  //   Platform
  pepper_joint_velocity_limits[0]  = Kv * 1.0;			// TODO - Find out the real values
  pepper_joint_velocity_limits[1]  = Kv * 1.0;			// TODO - Find out the real values
  pepper_joint_velocity_limits[2]  = Kv * 30.0*(PI/180.0);	// TODO - Find out the real values
  //   Torso
  pepper_joint_velocity_limits[3]  = Kv * 2.93276;
  pepper_joint_velocity_limits[4]  = Kv * 2.93276;
  pepper_joint_velocity_limits[5]  = Kv * 2.27032;
  //   Head
  pepper_joint_velocity_limits[6]  = Kv * 7.33998;
  pepper_joint_velocity_limits[7]  = Kv * 9.22756;
  //   Right Arm
  pepper_joint_velocity_limits[8]  = Kv * 7.33998;
  pepper_joint_velocity_limits[9]  = Kv * 9.22756;
  pepper_joint_velocity_limits[10] = Kv * 7.33998;
  pepper_joint_velocity_limits[11] = Kv * 9.22756;
  pepper_joint_velocity_limits[12] = Kv * 17.3835;
  //   Left Arm
  pepper_joint_velocity_limits[13] = Kv * 7.33998;
  pepper_joint_velocity_limits[14] = Kv * 9.22756;
  pepper_joint_velocity_limits[15] = Kv * 7.33998;
  pepper_joint_velocity_limits[16] = Kv * 9.22756;
  pepper_joint_velocity_limits[17] = Kv * 17.3835;
  
 
  // Joint acceleration limits ---------------------------------------------------------------------------------------------------------------------------------
  double Ka = 7.0;
  //   Platform
  pepper_joint_acceleration_limits[0]  = Ka * 1.0;
  pepper_joint_acceleration_limits[1]  = Ka * 1.0;
  pepper_joint_acceleration_limits[2]  = Ka * 90.0*(PI/180.0);
  //   Torso
  pepper_joint_acceleration_limits[3]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[4]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[5]  = Ka * 90.0*(PI/180.0);
  //   Head
  pepper_joint_acceleration_limits[6]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[7]  = Ka * 90.0*(PI/180.0);
  //   Right Arm
  pepper_joint_acceleration_limits[8]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[9]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[10] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[11] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[12] = Ka * 90.0*(PI/180.0);
  //   Left Arm
  pepper_joint_acceleration_limits[13] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[14] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[15] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[16] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[17] = Ka * 90.0*(PI/180.0);
  
  
#elif PEPPER_PLATFORM_N_DOF == 0
  // Joint limits ---------------------------------------------------------------------------------------------------------------------------------------
  
  //   MINIMUM
  //   Platform - No limits
  pepper_joint_limits_min[PLATFORM_JNT_X]  = 0.0;
  pepper_joint_limits_min[PLATFORM_JNT_Y]  = 0.0;
  pepper_joint_limits_min[PLATFORM_THETA]  = 0.0;
  //   Torso
  pepper_joint_limits_min[PLATFORM_JNT_TORSO_1]  = -0.05; //-0.514872;
  pepper_joint_limits_min[PLATFORM_JNT_TORSO_2]  = -0.05; //-1.03847;
  pepper_joint_limits_min[PLATFORM_JNT_TORSO_3]  = -0.514872;
  //   Head
  pepper_joint_limits_min[PLATFORM_JNT_HEAD_1]  = -2.08567;	// 0.0;// 
  pepper_joint_limits_min[PLATFORM_JNT_HEAD_2]  = -0.706858;
  //   Right Arm
  pepper_joint_limits_min[PLATFORM_JNT_RIGHT_ARM_1]  = -2.08567;
  pepper_joint_limits_min[PLATFORM_JNT_RIGHT_ARM_2]  = -1.56207;
  pepper_joint_limits_min[PLATFORM_JNT_RIGHT_ARM_3] = -2.08567;
  pepper_joint_limits_min[PLATFORM_JNT_RIGHT_ARM_4] = 0.00872665;
  pepper_joint_limits_min[PLATFORM_JNT_RIGHT_ARM_5] = -1.82387;
  //   Left Arm
  pepper_joint_limits_min[PLATFORM_JNT_LEFT_ARM_1] = -2.08567;
  pepper_joint_limits_min[PLATFORM_JNT_LEFT_ARM_2] = 0.00872665;
  pepper_joint_limits_min[PLATFORM_JNT_LEFT_ARM_3] = -2.08567;
  pepper_joint_limits_min[PLATFORM_JNT_LEFT_ARM_4] = -1.56207;
  pepper_joint_limits_min[PLATFORM_JNT_LEFT_ARM_5] = -1.82387;

  //   MAXIMUM
  //   Platform - No limits
  pepper_joint_limits_max[PLATFORM_JNT_X]  = 0.0;
  pepper_joint_limits_max[PLATFORM_JNT_Y]  = 0.0;
  pepper_joint_limits_max[PLATFORM_THETA]  = 0.0;
  //   Torso
  pepper_joint_limits_max[PLATFORM_JNT_TORSO_1]  = 0.25; //0.514872;
  pepper_joint_limits_max[PLATFORM_JNT_TORSO_2]  = 0.25; //1.03847;
  pepper_joint_limits_max[PLATFORM_JNT_TORSO_3]  = 0.514872;
  //   Head
  pepper_joint_limits_max[PLATFORM_JNT_HEAD_1]  = 2.08567;		// 0.0;// 
  pepper_joint_limits_max[PLATFORM_JNT_HEAD_2]  = 0.706858;
  //   Right Arm
  pepper_joint_limits_max[PLATFORM_JNT_RIGHT_ARM_1]  = 2.08567;
  pepper_joint_limits_max[PLATFORM_JNT_RIGHT_ARM_2]  = -0.00872665;
  pepper_joint_limits_max[PLATFORM_JNT_RIGHT_ARM_3] = 2.08567;
  pepper_joint_limits_max[PLATFORM_JNT_RIGHT_ARM_4] = 1.56207;
  pepper_joint_limits_max[PLATFORM_JNT_RIGHT_ARM_5] = 1.82387;
  //   Left Arm
  pepper_joint_limits_max[PLATFORM_JNT_LEFT_ARM_1] = 2.08567;
  pepper_joint_limits_max[PLATFORM_JNT_LEFT_ARM_2] = 1.56207;
  pepper_joint_limits_max[PLATFORM_JNT_LEFT_ARM_3] = 2.08567;
  pepper_joint_limits_max[PLATFORM_JNT_LEFT_ARM_4] = -0.00872665;
  pepper_joint_limits_max[PLATFORM_JNT_LEFT_ARM_5] = 1.82387;


  // Joint velocity limits ---------------------------------------------------------------------------------------------------------------------------------
  double Kv = 1.0;
  //   Platform
  pepper_joint_velocity_limits[PLATFORM_JNT_X]  = Kv * 1.0;			// TODO - Find out the real values
  pepper_joint_velocity_limits[PLATFORM_JNT_Y]  = Kv * 1.0;			// TODO - Find out the real values
  pepper_joint_velocity_limits[PLATFORM_THETA]  = Kv * 30.0*(PI/180.0);	// TODO - Find out the real values
  //   Torso
  pepper_joint_velocity_limits[PLATFORM_JNT_TORSO_1]  = Kv * 2.93276;
  pepper_joint_velocity_limits[PLATFORM_JNT_TORSO_2]  = Kv * 2.93276;
  pepper_joint_velocity_limits[PLATFORM_JNT_TORSO_3]  = Kv * 2.27032;
  //   Head
  pepper_joint_velocity_limits[PLATFORM_JNT_HEAD_1]  = Kv * 7.33998;
  pepper_joint_velocity_limits[PLATFORM_JNT_HEAD_2]  = Kv * 9.22756;
  //   Right Arm
  pepper_joint_velocity_limits[PLATFORM_JNT_RIGHT_ARM_1]  = Kv * 7.33998;
  pepper_joint_velocity_limits[PLATFORM_JNT_RIGHT_ARM_2]  = Kv * 9.22756;
  pepper_joint_velocity_limits[PLATFORM_JNT_RIGHT_ARM_3] = Kv * 7.33998;
  pepper_joint_velocity_limits[PLATFORM_JNT_RIGHT_ARM_4] = Kv * 9.22756;
  pepper_joint_velocity_limits[PLATFORM_JNT_RIGHT_ARM_5] = Kv * 17.3835;
  //   Left Arm
  pepper_joint_velocity_limits[PLATFORM_JNT_LEFT_ARM_1] = Kv * 7.33998;
  pepper_joint_velocity_limits[PLATFORM_JNT_LEFT_ARM_2] = Kv * 9.22756;
  pepper_joint_velocity_limits[PLATFORM_JNT_LEFT_ARM_3] = Kv * 7.33998;
  pepper_joint_velocity_limits[PLATFORM_JNT_LEFT_ARM_4] = Kv * 9.22756;
  pepper_joint_velocity_limits[PLATFORM_JNT_LEFT_ARM_5] = Kv * 17.3835;
  
 
  // Joint acceleration limits ---------------------------------------------------------------------------------------------------------------------------------
  double Ka = 7.0;
  //   Platform
  pepper_joint_acceleration_limits[PLATFORM_JNT_X]  = Ka * 1.0;
  pepper_joint_acceleration_limits[PLATFORM_JNT_Y]  = Ka * 1.0;
  pepper_joint_acceleration_limits[PLATFORM_THETA]  = Ka * 90.0*(PI/180.0);
  //   Torso
  pepper_joint_acceleration_limits[PLATFORM_JNT_TORSO_1]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_TORSO_2]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_TORSO_3]  = Ka * 90.0*(PI/180.0);
  //   Head
  pepper_joint_acceleration_limits[PLATFORM_JNT_HEAD_1]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_HEAD_2]  = Ka * 90.0*(PI/180.0);
  //   Right Arm
  pepper_joint_acceleration_limits[PLATFORM_JNT_RIGHT_ARM_1]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_RIGHT_ARM_2]  = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_RIGHT_ARM_3] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_RIGHT_ARM_4] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_RIGHT_ARM_5] = Ka * 90.0*(PI/180.0);
  //   Left Arm
  pepper_joint_acceleration_limits[PLATFORM_JNT_LEFT_ARM_1] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_LEFT_ARM_2] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_LEFT_ARM_3] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_LEFT_ARM_4] = Ka * 90.0*(PI/180.0);
  pepper_joint_acceleration_limits[PLATFORM_JNT_LEFT_ARM_5] = Ka * 90.0*(PI/180.0);
#endif
}


void set_home_configuration(double* home_configuration){
  
  // Platform
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF; i++)		home_configuration[i] = 0.0;
  
  // Joints
  //   Torso
  home_configuration[PLATFORM_JNT_TORSO_1] = 0.0;
  home_configuration[PLATFORM_JNT_TORSO_2] = 0.0;
  home_configuration[PLATFORM_JNT_TORSO_3] = 0.0;
  //   Head
  home_configuration[PLATFORM_JNT_HEAD_1] = 0.0;
  home_configuration[PLATFORM_JNT_HEAD_2] = 0.0;
  //   Right arm
  home_configuration[PLATFORM_JNT_RIGHT_ARM_1] = 0.0;
  home_configuration[PLATFORM_JNT_RIGHT_ARM_2] = -0.00872665;
  home_configuration[PLATFORM_JNT_RIGHT_ARM_3] = 0.0;
  home_configuration[PLATFORM_JNT_RIGHT_ARM_4] = -0.00872665;
  home_configuration[PLATFORM_JNT_RIGHT_ARM_5] = 0.0;
  //   Left arm
  home_configuration[PLATFORM_JNT_LEFT_ARM_1] = 0.0;
  home_configuration[PLATFORM_JNT_LEFT_ARM_2] = -0.00872665;
  home_configuration[PLATFORM_JNT_LEFT_ARM_3] = 0.0;
  home_configuration[PLATFORM_JNT_LEFT_ARM_4] = -0.00872665;
  home_configuration[PLATFORM_JNT_LEFT_ARM_5] = 0.0;  
}


void saturate_joint_velocity(Eigen::VectorXd &configuration_inc, const double time_increment){
    
  
  // Search for the velocity:
  // V = K * Vmax
  // with Vmax the maximum joint velocity
  // such that K = max(k[i]) > 0
  Eigen::VectorXd configuration_velocity = configuration_inc/time_increment;
  
  double Kmax = 0.0;
  unsigned int i_max = 0;
#if PEPPER_PLATFORM_N_DOF == 3
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)
  {
    double Ki = fabs(configuration_velocity(i)) / pepper_joint_velocity_limits[i];
    if ( Ki > Kmax )
    {
      i_max = i;
      Kmax = Ki;
    }
  }
  // Scale velocities if needed
  if ( Kmax > 1.0 )	configuration_velocity = configuration_velocity / Kmax;
  
#elif PEPPER_PLATFORM_N_DOF == 0
  // ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM 
  //  This code only suits the waving motion !!!!!!!! TEST TEST TEST TEST TEST TEST TEST
  // ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM ALARM 
  for (unsigned int i=0; i<10; i++)
  {
    double Ki = fabs(configuration_velocity(i)) / pepper_joint_velocity_limits[i];
    if ( Ki > Kmax )
    {
      i_max = i;
      Kmax = Ki;
    }
  }
  // Scale velocities if needed
  if ( Kmax > 1.0 )	for (unsigned int i=0; i<10; i++) 	configuration_velocity(i) = configuration_velocity(i) / Kmax;
  
  Kmax = 0.0;
  for (unsigned int i=10; i<15; i++)
  {
    double Ki = fabs(configuration_velocity(i)) / pepper_joint_velocity_limits[i];
    if ( Ki > Kmax )
    {
      i_max = i;
      Kmax = Ki;
    }
  }
  // Scale velocities if needed
  if ( Kmax > 1.0 )	for (unsigned int i=10; i<15; i++)	configuration_velocity(i) = configuration_velocity(i) / Kmax;
#endif
    
  configuration_inc = configuration_velocity * time_increment;
  
  return;
}


void saturate_joint_acceleration(Eigen::VectorXd &configuration_inc, Eigen::VectorXd past_configuration_inc, const double time_increment){
    
  // Search for the velocity:
  // A = A * Vmax
  // with Amax the maximum joint acceleration
  // such that K = max(k[i]) > 0
  
  Eigen::VectorXd current_velocity(configuration_inc.size()), past_velocity(configuration_inc.size());
  current_velocity = configuration_inc/time_increment;
  past_velocity = past_configuration_inc/time_increment;
  
  Eigen::VectorXd velocity_inc(configuration_inc.size());
  velocity_inc = current_velocity - past_velocity;
  
  double Kmax = 0.0;
  unsigned int i_max = 0;
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)
  {
    double Ki = fabs(velocity_inc(i)/time_increment) / pepper_joint_acceleration_limits[i];
    if ( Ki > Kmax )
    {
      i_max = i;
      Kmax = Ki;
    }
  }
  
  // Scale velocities if needed
  if ( Kmax > 1.0 )	velocity_inc = velocity_inc / Kmax;
  
  configuration_inc = past_configuration_inc + velocity_inc * time_increment;
  
  return;
}


void saturate_joint_angles(double* configuration){
  
  // Platform - Check Z axis rotation angle between [0, 2PI]
#if PEPPER_PLATFORM_N_DOF == 3
  while (configuration[2] < 0.0)		configuration[2] += 2*PI;
  while (configuration[2] >= 2*PI)		configuration[2] -= 2*PI;
#endif
  // Pepper
  for (unsigned int i=PEPPER_PLATFORM_N_DOF; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)
  {
    if		( configuration[i] < pepper_joint_limits_min[i] )		configuration[i] = pepper_joint_limits_min[i];
    else if	( configuration[i] > pepper_joint_limits_max[i] )		configuration[i] = pepper_joint_limits_max[i];
  }
  
  return;
}


void generate_pepper_random_configuration(double* random_configuration){
  
  // Platform X, Y positions and axis Z rotation are left as is
  for (unsigned int i=PEPPER_PLATFORM_N_DOF; i<PEPPER_PLATFORM_N_DOF+PEPPER_N_JOINTS; i++)
  {
    random_configuration[i] = pepper_joint_limits_min[i] + ((double)rand() / (double)(RAND_MAX)) * (pepper_joint_limits_max[i] - pepper_joint_limits_min[i]);
    
    while (  ( random_configuration[i] < pepper_joint_limits_min[i] )  ||  ( random_configuration[i] > pepper_joint_limits_max[i] )  )
      random_configuration[i] = pepper_joint_limits_min[i] + ((double)rand() / (double)(RAND_MAX)) * (pepper_joint_limits_max[i] - pepper_joint_limits_min[i]);
  }
  
  return;
}

void add_random_offset_to_pepper_configuration(double* configuration, const double offset){
  // Platform 2D pose remains equal
  // Offset \in [0 , 1] : for each joint new_joint[i] = old_joint[i] + rand_value_from_interval(1,-1) * Ooffset * (joint_max[i] - joint_min[i])
  
  for (unsigned int i=PEPPER_PLATFORM_N_DOF; i<PEPPER_PLATFORM_N_DOF+PEPPER_N_JOINTS; i++)
    configuration[i] = configuration[i] + (2.0*((double)rand() / (double)(RAND_MAX)) - 1.0) * offset * (pepper_joint_limits_max[i] - pepper_joint_limits_min[i]);

  saturate_joint_angles(configuration);
  
  return;
}


// compute_poses_from_config
void compute_poses_from_config(const double* configuration, double* poses){
  
  KDL::Chain* pepper_chains = new KDL::Chain[PEPPER_N_CHAINS];
  initialize_kinematic_chains(pepper_chains);
  
  compute_poses_from_config(pepper_chains, configuration, poses);
  
  return;
}

void compute_poses_from_config(KDL::Chain* pepper_chains, const double* configuration, double* poses){
  
  KDL::ChainFkSolverPos_recursive** fk_solvers = new KDL::ChainFkSolverPos_recursive*[PEPPER_N_CHAINS];
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	  fk_solvers[i] = new KDL::ChainFkSolverPos_recursive(pepper_chains[i]);
  
  compute_poses_from_config(fk_solvers, configuration, poses);
  
  return;
}

void compute_poses_from_config(KDL::ChainFkSolverPos_recursive** fk_solvers, const double* configuration, double* poses){

  // KDL CHAINS CONFIGURATIONS *****************************************************************************************************************************
  
#if PEPPER_PLATFORM_N_DOF == 3  
  //   HEAD
  KDL::JntArray config_head_chain = KDL::JntArray(PEPPER_HEAD_CHAIN_JOINTS);
  config_head_chain(0) = configuration[0];
  config_head_chain(1) = configuration[1]; 
  config_head_chain(2) = configuration[2]; 
  config_head_chain(3) = configuration[3]; 
  config_head_chain(4) = configuration[4]; 
  config_head_chain(5) = configuration[5]; 
  config_head_chain(6) = configuration[6];
  config_head_chain(7) = configuration[7];
  
  //   RIGHT ARM
  KDL::JntArray config_rightarm_chain = KDL::JntArray(PEPPER_RIGHTARM_CHAIN_JOINTS);
  config_rightarm_chain(0)  = configuration[0];
  config_rightarm_chain(1)  = configuration[1]; 
  config_rightarm_chain(2)  = configuration[2]; 
  config_rightarm_chain(3)  = configuration[3]; 
  config_rightarm_chain(4)  = configuration[4]; 
  config_rightarm_chain(5)  = configuration[5]; 
  config_rightarm_chain(6)  = configuration[8];
  config_rightarm_chain(7)  = configuration[9];
  config_rightarm_chain(8)  = configuration[10];
  config_rightarm_chain(9)  = configuration[11];
  config_rightarm_chain(10) = configuration[12];
  
  //   LEFT ARM  
  KDL::JntArray config_leftarm_chain = KDL::JntArray(PEPPER_LEFTARM_CHAIN_JOINTS);
  config_leftarm_chain(0)  = configuration[0];
  config_leftarm_chain(1)  = configuration[1]; 
  config_leftarm_chain(2)  = configuration[2]; 
  config_leftarm_chain(3)  = configuration[3]; 
  config_leftarm_chain(4)  = configuration[4]; 
  config_leftarm_chain(5)  = configuration[5]; 
  config_leftarm_chain(6)  = configuration[13];
  config_leftarm_chain(7)  = configuration[14];
  config_leftarm_chain(8)  = configuration[15];
  config_leftarm_chain(9)  = configuration[16];
  config_leftarm_chain(10) = configuration[17];
  
  //   TORSO
  KDL::JntArray config_torso_chain = KDL::JntArray(PEPPER_TORSO_CHAIN_JOINTS);
  config_torso_chain(0) = configuration[0];
  config_torso_chain(1) = configuration[1]; 
  config_torso_chain(2) = configuration[2]; 
  config_torso_chain(3) = configuration[3]; 
  config_torso_chain(4) = configuration[4]; 
  config_torso_chain(5) = configuration[5]; 
  
  //   RIGHT ELBOW
  KDL::JntArray config_rightelbow_chain = KDL::JntArray(PEPPER_RIGHTELBOW_CHAIN_JOINTS);
  config_rightelbow_chain(0) = configuration[0];
  config_rightelbow_chain(1) = configuration[1]; 
  config_rightelbow_chain(2) = configuration[2]; 
  config_rightelbow_chain(3) = configuration[3]; 
  config_rightelbow_chain(4) = configuration[4]; 
  config_rightelbow_chain(5) = configuration[5]; 
  config_rightelbow_chain(6) = configuration[8];
  config_rightelbow_chain(7) = configuration[9];
  
  //   LEFT ELBOW
  KDL::JntArray config_leftelbow_chain = KDL::JntArray(PEPPER_LEFTELBOW_CHAIN_JOINTS);
  config_leftelbow_chain(0) = configuration[0];
  config_leftelbow_chain(1) = configuration[1]; 
  config_leftelbow_chain(2) = configuration[2]; 
  config_leftelbow_chain(3) = configuration[3]; 
  config_leftelbow_chain(4) = configuration[4]; 
  config_leftelbow_chain(5) = configuration[5]; 
  config_leftelbow_chain(6) = configuration[13];
  config_leftelbow_chain(7) = configuration[14];
  
#elif PEPPER_PLATFORM_N_DOF == 0
  //   HEAD
  KDL::JntArray config_head_chain = KDL::JntArray(PEPPER_HEAD_CHAIN_JOINTS);
  config_head_chain(0) = configuration[0];
  config_head_chain(1) = configuration[1]; 
  config_head_chain(2) = configuration[2]; 
  config_head_chain(3) = configuration[3]; 
  config_head_chain(4) = configuration[4]; 
  
  //   RIGHT ARM
  KDL::JntArray config_rightarm_chain = KDL::JntArray(PEPPER_RIGHTARM_CHAIN_JOINTS);
  config_rightarm_chain(0)  = configuration[0];
  config_rightarm_chain(1)  = configuration[1]; 
  config_rightarm_chain(2)  = configuration[2]; 
  config_rightarm_chain(3)  = configuration[5]; 
  config_rightarm_chain(4)  = configuration[6]; 
  config_rightarm_chain(5)  = configuration[7]; 
  config_rightarm_chain(6)  = configuration[8];
  config_rightarm_chain(7)  = configuration[9];
  
  //   LEFT ARM  
  KDL::JntArray config_leftarm_chain = KDL::JntArray(PEPPER_LEFTARM_CHAIN_JOINTS);
  config_leftarm_chain(0)  = configuration[0];
  config_leftarm_chain(1)  = configuration[1]; 
  config_leftarm_chain(2)  = configuration[2]; 
  config_leftarm_chain(3)  = configuration[10]; 
  config_leftarm_chain(4)  = configuration[11]; 
  config_leftarm_chain(5)  = configuration[12]; 
  config_leftarm_chain(6)  = configuration[13];
  config_leftarm_chain(7)  = configuration[14];
  
  //   TORSO
  KDL::JntArray config_torso_chain = KDL::JntArray(PEPPER_TORSO_CHAIN_JOINTS);
  config_torso_chain(0) = configuration[0];
  config_torso_chain(1) = configuration[1]; 
  config_torso_chain(2) = configuration[2]; 
  
  //   RIGHT ELBOW
  KDL::JntArray config_rightelbow_chain = KDL::JntArray(PEPPER_RIGHTELBOW_CHAIN_JOINTS);
  config_rightelbow_chain(0) = configuration[0];
  config_rightelbow_chain(1) = configuration[1]; 
  config_rightelbow_chain(2) = configuration[2]; 
  config_rightelbow_chain(3) = configuration[5]; 
  config_rightelbow_chain(4) = configuration[6]; 
  
  //   LEFT ELBOW
  KDL::JntArray config_leftelbow_chain = KDL::JntArray(PEPPER_LEFTELBOW_CHAIN_JOINTS);
  config_leftelbow_chain(0) = configuration[0];
  config_leftelbow_chain(1) = configuration[1]; 
  config_leftelbow_chain(2) = configuration[2]; 
  config_leftelbow_chain(3) = configuration[10]; 
  config_leftelbow_chain(4) = configuration[11]; 
#endif  
  
  // Direct kinematics *****************************************************************************************************************************
  KDL::Frame KDL_head_frame, KDL_rightarm_frame, KDL_leftarm_frame, KDL_torso_frame, KDL_rightelbow_frame, KDL_leftelbow_frame;
  fk_solvers[0]->JntToCart(config_head_chain,       KDL_head_frame);
  fk_solvers[1]->JntToCart(config_rightarm_chain,   KDL_rightarm_frame);
  fk_solvers[2]->JntToCart(config_leftarm_chain,    KDL_leftarm_frame);
  fk_solvers[3]->JntToCart(config_torso_chain,      KDL_torso_frame);
  fk_solvers[4]->JntToCart(config_rightelbow_chain, KDL_rightelbow_frame);
  fk_solvers[5]->JntToCart(config_leftelbow_chain,  KDL_leftelbow_frame);

  
  // Pose vector ***********************************************************************************************************************************
  
  //   Positions
  //     Head
  poses[0] = KDL_head_frame(0,3);
  poses[1] = KDL_head_frame(1,3);
  poses[2] = KDL_head_frame(2,3);
  //     Right arm
  poses[7] = KDL_rightarm_frame(0,3);
  poses[8] = KDL_rightarm_frame(1,3);
  poses[9] = KDL_rightarm_frame(2,3);
  //     Left arm
  poses[14] = KDL_leftarm_frame(0,3);
  poses[15] = KDL_leftarm_frame(1,3);
  poses[16] = KDL_leftarm_frame(2,3);
  //     Torso
  poses[21] = KDL_torso_frame(0,3);
  poses[22] = KDL_torso_frame(1,3);
  poses[23] = KDL_torso_frame(2,3);
  //     Right elbow
  poses[28] = KDL_rightelbow_frame(0,3);
  poses[29] = KDL_rightelbow_frame(1,3);
  poses[30] = KDL_rightelbow_frame(2,3);
  //     Left elbow
  poses[35] = KDL_leftelbow_frame(0,3);
  poses[36] = KDL_leftelbow_frame(1,3);
  poses[37] = KDL_leftelbow_frame(2,3);
  
  //   Orientations
  KDL_head_frame.M.GetQuaternion(poses[3], poses[4], poses[5], poses[6]);		// Head
  KDL_rightarm_frame.M.GetQuaternion(poses[10], poses[11], poses[12], poses[13]);	// Right arm
  KDL_leftarm_frame.M.GetQuaternion(poses[17], poses[18], poses[19], poses[20]);	// Left arm
  KDL_torso_frame.M.GetQuaternion(poses[24], poses[25], poses[26], poses[27]);		// Torso
  KDL_rightelbow_frame.M.GetQuaternion(poses[31], poses[32], poses[33], poses[34]);	// Right elbow
  KDL_leftelbow_frame.M.GetQuaternion(poses[38], poses[39], poses[40], poses[41]);	// Left elbow
  
  return;
}


void compute_pose_from_config(KDL::ChainFkSolverPos_recursive** fk_solvers, const double* configuration, double* pose, const unsigned int i_pose){
 
  // KDL CHAINS CONFIGURATIONS *****************************************************************************************************************************

  if (i_pose == 0){
    //   HEAD
    KDL::JntArray config_head_chain = KDL::JntArray(PEPPER_HEAD_CHAIN_JOINTS);
#if PEPPER_PLATFORM_N_DOF == 3    
    config_head_chain(0) = configuration[0];
    config_head_chain(1) = configuration[1]; 
    config_head_chain(2) = configuration[2]; 
    config_head_chain(3) = configuration[3]; 
    config_head_chain(4) = configuration[4]; 
    config_head_chain(5) = configuration[5]; 
    config_head_chain(6) = configuration[6];
    config_head_chain(7) = configuration[7]; 
#elif PEPPER_PLATFORM_N_DOF == 0
    config_head_chain(0) = configuration[0];
    config_head_chain(1) = configuration[1]; 
    config_head_chain(2) = configuration[2]; 
    config_head_chain(3) = configuration[3]; 
    config_head_chain(4) = configuration[4]; 
#endif  
    
    KDL::Frame KDL_head_frame;
    fk_solvers[0]->JntToCart(config_head_chain,       KDL_head_frame);
    
    pose[0] = KDL_head_frame(0,3);
    pose[1] = KDL_head_frame(1,3);
    pose[2] = KDL_head_frame(2,3);
    KDL_head_frame.M.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);
    
    return;
  }
  

  else if (i_pose == 1){
    //   RIGHT ARM
    KDL::JntArray config_rightarm_chain = KDL::JntArray(PEPPER_RIGHTARM_CHAIN_JOINTS);
#if PEPPER_PLATFORM_N_DOF == 3    
    config_rightarm_chain(0)  = configuration[0];
    config_rightarm_chain(1)  = configuration[1]; 
    config_rightarm_chain(2)  = configuration[2]; 
    config_rightarm_chain(3)  = configuration[3]; 
    config_rightarm_chain(4)  = configuration[4]; 
    config_rightarm_chain(5)  = configuration[5]; 
    config_rightarm_chain(6)  = configuration[8];
    config_rightarm_chain(7)  = configuration[9];
    config_rightarm_chain(8)  = configuration[10];
    config_rightarm_chain(9)  = configuration[11];
    config_rightarm_chain(10) = configuration[12];
#elif PEPPER_PLATFORM_N_DOF == 0
    config_rightarm_chain(0)  = configuration[0];
    config_rightarm_chain(1)  = configuration[1]; 
    config_rightarm_chain(2)  = configuration[2]; 
    config_rightarm_chain(3)  = configuration[5]; 
    config_rightarm_chain(4)  = configuration[6]; 
    config_rightarm_chain(5)  = configuration[7]; 
    config_rightarm_chain(6)  = configuration[8];
    config_rightarm_chain(7)  = configuration[9];
#endif
    
    KDL::Frame KDL_rightarm_frame;
    fk_solvers[1]->JntToCart(config_rightarm_chain, KDL_rightarm_frame);
    
    pose[0] = KDL_rightarm_frame(0,3);
    pose[1] = KDL_rightarm_frame(1,3);
    pose[2] = KDL_rightarm_frame(2,3);
    KDL_rightarm_frame.M.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);
    
    return;
  }  
  
  
  else if (i_pose == 2){
    //   LEFT ARM  
    KDL::JntArray config_leftarm_chain = KDL::JntArray(PEPPER_LEFTARM_CHAIN_JOINTS);
#if PEPPER_PLATFORM_N_DOF == 3    
    config_leftarm_chain(0)  = configuration[0];
    config_leftarm_chain(1)  = configuration[1]; 
    config_leftarm_chain(2)  = configuration[2]; 
    config_leftarm_chain(3)  = configuration[3]; 
    config_leftarm_chain(4)  = configuration[4]; 
    config_leftarm_chain(5)  = configuration[5]; 
    config_leftarm_chain(6)  = configuration[13];
    config_leftarm_chain(7)  = configuration[14];
    config_leftarm_chain(8)  = configuration[15];
    config_leftarm_chain(9)  = configuration[16];
    config_leftarm_chain(10) = configuration[17];
#elif PEPPER_PLATFORM_N_DOF == 0    
    config_leftarm_chain(0)  = configuration[0];
    config_leftarm_chain(1)  = configuration[1]; 
    config_leftarm_chain(2)  = configuration[2]; 
    config_leftarm_chain(3)  = configuration[10]; 
    config_leftarm_chain(4)  = configuration[11]; 
    config_leftarm_chain(5)  = configuration[12]; 
    config_leftarm_chain(6)  = configuration[13];
    config_leftarm_chain(7)  = configuration[14];
#endif    
    
    KDL::Frame KDL_leftarm_frame;
    fk_solvers[2]->JntToCart(config_leftarm_chain, KDL_leftarm_frame);
    
    pose[0] = KDL_leftarm_frame(0,3);
    pose[1] = KDL_leftarm_frame(1,3);
    pose[2] = KDL_leftarm_frame(2,3);
    KDL_leftarm_frame.M.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);
    
    return;
  }
  
  
  else if (i_pose == 3){
    //   TORSO
    KDL::JntArray config_torso_chain = KDL::JntArray(PEPPER_TORSO_CHAIN_JOINTS);
#if PEPPER_PLATFORM_N_DOF == 3
    config_torso_chain(0) = configuration[0];
    config_torso_chain(1) = configuration[1]; 
    config_torso_chain(2) = configuration[2]; 
    config_torso_chain(3) = configuration[3]; 
    config_torso_chain(4) = configuration[4]; 
    config_torso_chain(5) = configuration[5];
#elif PEPPER_PLATFORM_N_DOF == 0
    config_torso_chain(0) = configuration[0];
    config_torso_chain(1) = configuration[1]; 
    config_torso_chain(2) = configuration[2]; 
#endif
    
    KDL::Frame KDL_torso_frame;
    fk_solvers[3]->JntToCart(config_torso_chain, KDL_torso_frame);
    
    pose[0] = KDL_torso_frame(0,3);
    pose[1] = KDL_torso_frame(1,3);
    pose[2] = KDL_torso_frame(2,3);
    KDL_torso_frame.M.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);
    
    return;
  }
  
  
  else if (i_pose == 4){
    //   RIGHT ELBOW
    KDL::JntArray config_rightelbow_chain = KDL::JntArray(PEPPER_RIGHTELBOW_CHAIN_JOINTS);
#if PEPPER_PLATFORM_N_DOF == 3
    config_rightelbow_chain(0) = configuration[0];
    config_rightelbow_chain(1) = configuration[1]; 
    config_rightelbow_chain(2) = configuration[2]; 
    config_rightelbow_chain(3) = configuration[3]; 
    config_rightelbow_chain(4) = configuration[4]; 
    config_rightelbow_chain(5) = configuration[5]; 
    config_rightelbow_chain(6) = configuration[8];
    config_rightelbow_chain(7) = configuration[9];
#elif PEPPER_PLATFORM_N_DOF == 0
    config_rightelbow_chain(0) = configuration[0];
    config_rightelbow_chain(1) = configuration[1]; 
    config_rightelbow_chain(2) = configuration[2]; 
    config_rightelbow_chain(3) = configuration[5]; 
    config_rightelbow_chain(4) = configuration[6]; 
#endif
    
    KDL::Frame KDL_rightelbow_frame;
    fk_solvers[4]->JntToCart(config_rightelbow_chain, KDL_rightelbow_frame);
    
    pose[0] = KDL_rightelbow_frame(0,3);
    pose[1] = KDL_rightelbow_frame(1,3);
    pose[2] = KDL_rightelbow_frame(2,3);
    KDL_rightelbow_frame.M.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);
    
    return;
  }
    
    
  else if (i_pose == 5){
    //   LEFT ELBOW
    KDL::JntArray config_leftelbow_chain = KDL::JntArray(PEPPER_LEFTELBOW_CHAIN_JOINTS);
#if PEPPER_PLATFORM_N_DOF == 3
    config_leftelbow_chain(0) = configuration[0];
    config_leftelbow_chain(1) = configuration[1]; 
    config_leftelbow_chain(2) = configuration[2]; 
    config_leftelbow_chain(3) = configuration[3]; 
    config_leftelbow_chain(4) = configuration[4]; 
    config_leftelbow_chain(5) = configuration[5]; 
    config_leftelbow_chain(6) = configuration[13];
    config_leftelbow_chain(7) = configuration[14];
#elif PEPPER_PLATFORM_N_DOF == 0
    config_leftelbow_chain(0) = configuration[0];
    config_leftelbow_chain(1) = configuration[1]; 
    config_leftelbow_chain(2) = configuration[2]; 
    config_leftelbow_chain(3) = configuration[10]; 
    config_leftelbow_chain(4) = configuration[11]; 
#endif
    
    KDL::Frame KDL_leftelbow_frame;
    fk_solvers[5]->JntToCart(config_leftelbow_chain, KDL_leftelbow_frame);

    pose[0] = KDL_leftelbow_frame(0,3);
    pose[1] = KDL_leftelbow_frame(1,3);
    pose[2] = KDL_leftelbow_frame(2,3);
    KDL_leftelbow_frame.M.GetQuaternion(pose[3], pose[4], pose[5], pose[6]);
    
    return;
  }
  

  return;
}



void compute_errors_from_poses(double* current_poses, double* desired_poses, double* past_desired_poses, double* Kp, Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, const double period, const unsigned int error_alg_id){
  
  // ****************************************************************************************************************************************************************************************
  // ERROR VECTOR ***************************************************************************************************************************************************************************
  switch (error_alg_id)
  {
  
    // SIMPLE ERROR
    case 0:
      for (unsigned int n_tcp = 0; n_tcp < PEPPER_N_TCP; n_tcp++)
      {  
        alg_error_p[n_tcp] = Kp[0] * Eigen::Vector3d(desired_poses[n_tcp*7 + 0] - current_poses[n_tcp*7 + 0], desired_poses[n_tcp*7 + 1] - current_poses[n_tcp*7 + 1], desired_poses[n_tcp*7 + 2] - current_poses[n_tcp*7 + 2]);  
        Eigen::AngleAxis<double> errRotAngAxis = Eigen::AngleAxis<double>(Eigen::Quaternion<double>(desired_poses[n_tcp*7 + 6], desired_poses[n_tcp*7 + 3], desired_poses[n_tcp*7 + 4], desired_poses[n_tcp*7 + 5]).toRotationMatrix() * 
									  Eigen::Quaternion<double>(current_poses[n_tcp*7 + 6], current_poses[n_tcp*7 + 3], current_poses[n_tcp*7 + 4], current_poses[n_tcp*7 + 5]).toRotationMatrix().transpose());
        alg_error_w[n_tcp] = Kp[1] * errRotAngAxis.angle() * errRotAngAxis.axis();
      }
      break;

      
    // CLIK
    case 1:
      for (unsigned int n_tcp = 0; n_tcp < PEPPER_N_TCP; n_tcp++)
      {
	//   Velocities
        alg_error_p[n_tcp] << (desired_poses[n_tcp*7 + 0] - past_desired_poses[n_tcp*7 + 0]), (desired_poses[n_tcp*7 + 1] - past_desired_poses[n_tcp*7 + 1]), (desired_poses[n_tcp*7 + 2] - past_desired_poses[n_tcp*7 + 2]);

        Eigen::AngleAxis<double> angAx(Eigen::Quaternion<double>(desired_poses[n_tcp*7 + 6],      desired_poses[n_tcp*7 + 3],      desired_poses[n_tcp*7 + 4],      desired_poses[n_tcp*7 + 5]     ).toRotationMatrix() *
				   Eigen::Quaternion<double>(past_desired_poses[n_tcp*7 + 6], past_desired_poses[n_tcp*7 + 3], past_desired_poses[n_tcp*7 + 4], past_desired_poses[n_tcp*7 + 5]).toRotationMatrix().transpose());
        alg_error_w[n_tcp] = sin(angAx.angle()) * angAx.axis();
	
	//   Errors
	alg_error_p[n_tcp] = alg_error_p[n_tcp] + Kp[0] * Eigen::Vector3d(desired_poses[n_tcp*7 + 0] - current_poses[n_tcp*7 + 0], desired_poses[n_tcp*7 + 1] - current_poses[n_tcp*7 + 1], desired_poses[n_tcp*7 + 2] - current_poses[n_tcp*7 + 2]);  
        Eigen::AngleAxis<double> errRotAngAxis = Eigen::AngleAxis<double>(Eigen::Quaternion<double>(desired_poses[n_tcp*7 + 6], desired_poses[n_tcp*7 + 3], desired_poses[n_tcp*7 + 4], desired_poses[n_tcp*7 + 5]).toRotationMatrix() * 
									  Eigen::Quaternion<double>(current_poses[n_tcp*7 + 6], current_poses[n_tcp*7 + 3], current_poses[n_tcp*7 + 4], current_poses[n_tcp*7 + 5]).toRotationMatrix().transpose());
        alg_error_w[n_tcp] = alg_error_w[n_tcp] + Kp[1] * errRotAngAxis.angle() * errRotAngAxis.axis();
      }
      break;

      
    // CLIK - Siciliano
    case 2:
      for (unsigned int n_tcp = 0; n_tcp < PEPPER_N_TCP; n_tcp++)
      {
	Eigen::Matrix3d L = Lmat(Eigen::Quaternion<double>(current_poses[n_tcp*7 + 6], current_poses[n_tcp*7 + 3], current_poses[n_tcp*7 + 4], current_poses[n_tcp*7 + 5]),
				 Eigen::Quaternion<double>(desired_poses[n_tcp*7 + 6], desired_poses[n_tcp*7 + 3], desired_poses[n_tcp*7 + 4], desired_poses[n_tcp*7 + 5]));	

	//   Velocities
        alg_error_p[n_tcp] << (desired_poses[n_tcp*7 + 0] - past_desired_poses[n_tcp*7 + 0]), (desired_poses[n_tcp*7 + 1] - past_desired_poses[n_tcp*7 + 1]), (desired_poses[n_tcp*7 + 2] - past_desired_poses[n_tcp*7 + 2]);
        Eigen::AngleAxis<double> angAx(Eigen::Quaternion<double>(desired_poses[n_tcp*7 + 6],      desired_poses[n_tcp*7 + 3],      desired_poses[n_tcp*7 + 4],      desired_poses[n_tcp*7 + 5]     ).toRotationMatrix() *
				       Eigen::Quaternion<double>(past_desired_poses[n_tcp*7 + 6], past_desired_poses[n_tcp*7 + 3], past_desired_poses[n_tcp*7 + 4], past_desired_poses[n_tcp*7 + 5]).toRotationMatrix().transpose());
        alg_error_w[n_tcp] = L.transpose() * sin(angAx.angle()) * angAx.axis();

	// Errors
	alg_error_p[n_tcp] = alg_error_p[n_tcp] + Kp[0] * Eigen::Vector3d(desired_poses[n_tcp*7 + 0] - current_poses[n_tcp*7 + 0], desired_poses[n_tcp*7 + 1] - current_poses[n_tcp*7 + 1], desired_poses[n_tcp*7 + 2] - current_poses[n_tcp*7 + 2]);  
	alg_error_w[n_tcp] = alg_error_w[n_tcp] + Kp[1] * sicil_or_error(Eigen::Quaternion<double>(desired_poses[n_tcp*7 + 6], desired_poses[n_tcp*7 + 3], desired_poses[n_tcp*7 + 4], desired_poses[n_tcp*7 + 5]),
									 Eigen::Quaternion<double>(current_poses[n_tcp*7 + 6], current_poses[n_tcp*7 + 3], current_poses[n_tcp*7 + 4], current_poses[n_tcp*7 + 5]));

	alg_error_w[n_tcp] = L.inverse() * alg_error_w[n_tcp];
      }
      break;
  }
  return;
}



void compute_jacobians_from_config(KDL::JntArray* pepper_chains, KDL::ChainJntToJacSolver** jacobian_solvers, const double* configuration, KDL::Jacobian* KDL_jacobians){
  
  // Chain initialization ------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
#if PEPPER_PLATFORM_N_DOF == 3
  //  HEAD
  pepper_chains[0](0)  = configuration[0];
  pepper_chains[0](1)  = configuration[1];
  pepper_chains[0](2)  = configuration[2];
  pepper_chains[0](3)  = configuration[3];
  pepper_chains[0](4)  = configuration[4];
  pepper_chains[0](5)  = configuration[5];
  pepper_chains[0](6)  = configuration[6];
  pepper_chains[0](7)  = configuration[7];

  //  RIGHT ARM
  pepper_chains[1](0)  = configuration[0];
  pepper_chains[1](1)  = configuration[1];
  pepper_chains[1](2)  = configuration[2];
  pepper_chains[1](3)  = configuration[3];
  pepper_chains[1](4)  = configuration[4];
  pepper_chains[1](5)  = configuration[5];
  pepper_chains[1](6)  = configuration[8];
  pepper_chains[1](7)  = configuration[9];
  pepper_chains[1](8)  = configuration[10];
  pepper_chains[1](9)  = configuration[11];
  pepper_chains[1](10) = configuration[12];

  //  LEFT ARM
  pepper_chains[2](0)  = configuration[0];
  pepper_chains[2](1)  = configuration[1];
  pepper_chains[2](2)  = configuration[2];
  pepper_chains[2](3)  = configuration[3];
  pepper_chains[2](4)  = configuration[4];
  pepper_chains[2](5)  = configuration[5];
  pepper_chains[2](6)  = configuration[13];
  pepper_chains[2](7)  = configuration[14];
  pepper_chains[2](8)  = configuration[15];
  pepper_chains[2](9)  = configuration[16];
  pepper_chains[2](10) = configuration[17];

  //  TORSO
  pepper_chains[3](0)  = configuration[0];
  pepper_chains[3](1)  = configuration[1];
  pepper_chains[3](2)  = configuration[2];
  pepper_chains[3](3)  = configuration[3];
  pepper_chains[3](4)  = configuration[4];
  pepper_chains[3](5)  = configuration[5];

  //  RIGHT ELBOW
  pepper_chains[4](0)  = configuration[0];
  pepper_chains[4](1)  = configuration[1];
  pepper_chains[4](2)  = configuration[2];
  pepper_chains[4](3)  = configuration[3];
  pepper_chains[4](4)  = configuration[4];
  pepper_chains[4](5)  = configuration[5];
  pepper_chains[4](6)  = configuration[8];
  pepper_chains[4](7)  = configuration[9];

  //  LEFT ELBOW
  pepper_chains[5](0)  = configuration[0];
  pepper_chains[5](1)  = configuration[1];
  pepper_chains[5](2)  = configuration[2];
  pepper_chains[5](3)  = configuration[3];
  pepper_chains[5](4)  = configuration[4];
  pepper_chains[5](5)  = configuration[5];
  pepper_chains[5](6)  = configuration[13];
  pepper_chains[5](7)  = configuration[14];
#elif PEPPER_PLATFORM_N_DOF == 0
  //  HEAD
  pepper_chains[0](0)  = configuration[0];
  pepper_chains[0](1)  = configuration[1];
  pepper_chains[0](2)  = configuration[2];
  pepper_chains[0](3)  = configuration[3];
  pepper_chains[0](4)  = configuration[4];

  //  RIGHT ARM
  pepper_chains[1](0)  = configuration[0];
  pepper_chains[1](1)  = configuration[1];
  pepper_chains[1](2)  = configuration[2];
  pepper_chains[1](3)  = configuration[5];
  pepper_chains[1](4)  = configuration[6];
  pepper_chains[1](5)  = configuration[7];
  pepper_chains[1](6)  = configuration[8];
  pepper_chains[1](7)  = configuration[9];

  //  LEFT ARM
  pepper_chains[2](0)  = configuration[0];
  pepper_chains[2](1)  = configuration[1];
  pepper_chains[2](2)  = configuration[2];
  pepper_chains[2](3)  = configuration[10];
  pepper_chains[2](4)  = configuration[11];
  pepper_chains[2](5)  = configuration[12];
  pepper_chains[2](6)  = configuration[13];
  pepper_chains[2](7)  = configuration[14];

  //  TORSO
  pepper_chains[3](0)  = configuration[0];
  pepper_chains[3](1)  = configuration[1];
  pepper_chains[3](2)  = configuration[2];

  //  RIGHT ELBOW
  pepper_chains[4](0)  = configuration[0];
  pepper_chains[4](1)  = configuration[1];
  pepper_chains[4](2)  = configuration[2];
  pepper_chains[4](3)  = configuration[5];
  pepper_chains[4](4)  = configuration[6];

  //  LEFT ELBOW
  pepper_chains[5](0)  = configuration[0];
  pepper_chains[5](1)  = configuration[1];
  pepper_chains[5](2)  = configuration[2];
  pepper_chains[5](3)  = configuration[10];
  pepper_chains[5](4)  = configuration[11];
#endif  
  
  // Chain jacobians ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	jacobian_solvers[i]->JntToJac(pepper_chains[i], KDL_jacobians[i]);
  
  
  return;
}


// generate_pepper_random_pose
void generate_pepper_random_pose(double* random_configuration, double* random_poses){

  generate_pepper_random_configuration(random_configuration);
  compute_poses_from_config(random_configuration, random_poses);

  return;
}

void generate_pepper_random_pose(double* random_poses){

  double random_configuration[PEPPER_PLATFORM_N_DOF+PEPPER_N_JOINTS];
  generate_pepper_random_pose(random_configuration, random_poses);

  return;
}

void generate_pepper_random_pose(KDL::Chain* pepper_chains, double* random_configuration, double* random_poses){

  generate_pepper_random_configuration(random_configuration);
  compute_poses_from_config(pepper_chains, random_configuration, random_poses);

  return;
}

void generate_pepper_random_pose(KDL::Chain* pepper_chains, double* random_poses){

  double random_configuration[PEPPER_PLATFORM_N_DOF+PEPPER_N_JOINTS];
  generate_pepper_random_pose(pepper_chains, random_configuration, random_poses);

  return;
}

void generate_pepper_random_pose(KDL::ChainFkSolverPos_recursive** fk_solvers, double* random_configuration, double* random_poses){

  generate_pepper_random_configuration(random_configuration);
  compute_poses_from_config(fk_solvers, random_configuration, random_poses);

  return;
}

void generate_pepper_random_pose(KDL::ChainFkSolverPos_recursive** fk_solvers, double* random_poses){

  double random_configuration[PEPPER_PLATFORM_N_DOF+PEPPER_N_JOINTS];
  generate_pepper_random_pose(fk_solvers, random_configuration, random_poses);

  return;
}


void pepperRosConfig_to_3ChainModelConfig(const double* pepperROSConfig2DPlatformPose, const double* pepperROSConfigJoints, double* pepper3ChainModel)
{
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF; i++)		pepper3ChainModel[i] = pepperROSConfig2DPlatformPose[i];
  //   Torso
  pepper3ChainModel[PLATFORM_JNT_TORSO_1] = pepperROSConfigJoints[4];		// KneePitch
  pepper3ChainModel[PLATFORM_JNT_TORSO_2] = pepperROSConfigJoints[3];		// HipPitch
  pepper3ChainModel[PLATFORM_JNT_TORSO_3] = pepperROSConfigJoints[2];		// HipRoll
  //   Head
  pepper3ChainModel[PLATFORM_JNT_HEAD_1] = pepperROSConfigJoints[0];		// HeadYaw
  pepper3ChainModel[PLATFORM_JNT_HEAD_2] = pepperROSConfigJoints[1];		// HeadPitch
  //   Right arm
  pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_1] = pepperROSConfigJoints[11];	// HeadPitch
  pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_1] = pepperROSConfigJoints[12];	// HeadRoll
  pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_1] = pepperROSConfigJoints[13];	// RElbowYaw
  pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_1] = pepperROSConfigJoints[14];	// RElbowRoll
  pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_1] = pepperROSConfigJoints[15];	// RWristYaw
  //   Left arm
  pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_1] = pepperROSConfigJoints[5];	// HeadPitch
  pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_1] = pepperROSConfigJoints[6];	// HeadRoll
  pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_1] = pepperROSConfigJoints[7];	// LElbowYaw
  pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_1] = pepperROSConfigJoints[8];	// LElbowRoll
  pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_1] = pepperROSConfigJoints[9];	// LWristYaw    
  
  return;
}


void pepper3ChainModelConfig_to_RosConfig(const double* pepper3ChainModel, double* pepperROSConfig2DPlatformPose, double* pepperROSConfigJoints)
{
  // Platform
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF; i++)		pepperROSConfig2DPlatformPose[i] = pepper3ChainModel[i];
  // Pepper
  //   Torso
  pepperROSConfigJoints[4]  = pepper3ChainModel[PLATFORM_JNT_TORSO_1];		// KneePitch
  pepperROSConfigJoints[3]  = pepper3ChainModel[PLATFORM_JNT_TORSO_2];		// HipPitch
  pepperROSConfigJoints[2]  = pepper3ChainModel[PLATFORM_JNT_TORSO_3];		// HipRoll
  //   Head
  pepperROSConfigJoints[0]  = pepper3ChainModel[PLATFORM_JNT_HEAD_1];		// HeadYaw
  pepperROSConfigJoints[1]  = pepper3ChainModel[PLATFORM_JNT_HEAD_2];		// HeadPitch
  //   Right arm
  pepperROSConfigJoints[11] = pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_1];	// HeadPitch
  pepperROSConfigJoints[12] = pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_2];	// HeadRoll
  pepperROSConfigJoints[13] = pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_3];	// RElbowYaw
  pepperROSConfigJoints[14] = pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_4];	// RElbowRoll
  pepperROSConfigJoints[15] = pepper3ChainModel[PLATFORM_JNT_RIGHT_ARM_5];	// RWristYaw
  //   Left arm
  pepperROSConfigJoints[5]  = pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_1];	// HeadPitch
  pepperROSConfigJoints[6]  = pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_2];	// HeadRoll
  pepperROSConfigJoints[7]  = pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_3];	// LElbowYaw
  pepperROSConfigJoints[8]  = pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_4];	// LElbowRoll
  pepperROSConfigJoints[9]  = pepper3ChainModel[PLATFORM_JNT_LEFT_ARM_5];	// LWristYaw
  
  return;
}