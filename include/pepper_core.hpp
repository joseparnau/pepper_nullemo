// ----------------------------------------------------------------------------------------------------------------------------
//
// Header file
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


#ifndef PEPPER_DATA_HPP
#define PEPPER_DATA_HPP


#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>


// 18 DoF for reduced model with platform and without hands
#define PEPPER_PLATFORM_N_DOF 0
#define PEPPER_N_JOINTS 15
#define PEPPER_TOTAL_N_JOINTS_SIMP PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS
//   Platform
#define PLATFORM_JNT_X 3
#define PLATFORM_JNT_Y PLATFORM_JNT_X + 1
#define PLATFORM_THETA PLATFORM_JNT_Y + 1
//   Torso
#define PLATFORM_JNT_TORSO_1 PEPPER_PLATFORM_N_DOF
#define PLATFORM_JNT_TORSO_2 PLATFORM_JNT_TORSO_1 + 1
#define PLATFORM_JNT_TORSO_3 PLATFORM_JNT_TORSO_2 + 1
//   Head
#define PLATFORM_JNT_HEAD_1 PLATFORM_JNT_TORSO_3 + 1
#define PLATFORM_JNT_HEAD_2 PLATFORM_JNT_HEAD_1 + 1
//   Right arm
#define PLATFORM_JNT_RIGHT_ARM_1 PLATFORM_JNT_HEAD_2 + 1
#define PLATFORM_JNT_RIGHT_ARM_2 PLATFORM_JNT_RIGHT_ARM_1 + 1
#define PLATFORM_JNT_RIGHT_ARM_3 PLATFORM_JNT_RIGHT_ARM_2 + 1
#define PLATFORM_JNT_RIGHT_ARM_4 PLATFORM_JNT_RIGHT_ARM_3 + 1
#define PLATFORM_JNT_RIGHT_ARM_5 PLATFORM_JNT_RIGHT_ARM_4 + 1
//   Left arm
#define PLATFORM_JNT_LEFT_ARM_1 PLATFORM_JNT_RIGHT_ARM_5 + 1
#define PLATFORM_JNT_LEFT_ARM_2 PLATFORM_JNT_LEFT_ARM_1 + 1
#define PLATFORM_JNT_LEFT_ARM_3 PLATFORM_JNT_LEFT_ARM_2 + 1
#define PLATFORM_JNT_LEFT_ARM_4 PLATFORM_JNT_LEFT_ARM_3 + 1
#define PLATFORM_JNT_LEFT_ARM_5 PLATFORM_JNT_LEFT_ARM_4 + 1

// 48 for complete model
#define PEPPER_TOTAL_N_JOINTS 48


#define PEPPER_N_CHAINS 6

#define PEPPER_N_TCP 3
// Two hands and head

// Chain information
#define PEPPER_PLATFORM_TO_TORSO_CHAIN_JOINTS PEPPER_PLATFORM_N_DOF + 3
#define PEPPER_HEAD_CHAIN_JOINTS PEPPER_PLATFORM_N_DOF + 5
#define PEPPER_RIGHTARM_CHAIN_JOINTS PEPPER_PLATFORM_N_DOF + 8
#define PEPPER_LEFTARM_CHAIN_JOINTS PEPPER_PLATFORM_N_DOF + 8
#define PEPPER_TORSO_CHAIN_JOINTS PEPPER_PLATFORM_N_DOF + 3
#define PEPPER_RIGHTELBOW_CHAIN_JOINTS PEPPER_PLATFORM_N_DOF + 5
#define PEPPER_LEFTELBOW_CHAIN_JOINTS PEPPER_PLATFORM_N_DOF + 5


// Mathematic variables
#ifndef PI
#define PI 3.141592
#endif


struct robotConfigData {
  
  double configuration[PEPPER_TOTAL_N_JOINTS_SIMP];
  double config_velocity[PEPPER_TOTAL_N_JOINTS_SIMP];
  double config_acceleration[PEPPER_TOTAL_N_JOINTS_SIMP];
  
  double singular_values[PEPPER_TOTAL_N_JOINTS_SIMP];
  
  double period;
};

// Joint values
double pepper_joint_limits_min[PEPPER_TOTAL_N_JOINTS_SIMP];
double pepper_joint_limits_max[PEPPER_TOTAL_N_JOINTS_SIMP];
double pepper_joint_velocity_limits[PEPPER_TOTAL_N_JOINTS_SIMP];
double pepper_joint_acceleration_limits[PEPPER_TOTAL_N_JOINTS_SIMP];

void load_pepper_joint_values(void);

void set_home_configuration(double* home_configuration);

void saturate_joint_velocity(Eigen::VectorXd &configuration_inc, const double time_increment);

void saturate_joint_acceleration(Eigen::VectorXd &configuration_inc, Eigen::VectorXd past_configuration_inc, const double time_increment);

void saturate_joint_angles(double* configuration);

void generate_pepper_random_configuration(double* random_configuration);

void add_random_offset_to_pepper_configuration(double* configuration, const double offset);

// compute_poses_from_config
void compute_poses_from_config(const double* configuration, double* poses);
void compute_poses_from_config(KDL::Chain* pepper_chains, const double* configuration, double* poses);
void compute_poses_from_config(KDL::ChainFkSolverPos_recursive** fk_solvers, const double* configuration, double* poses);
void compute_pose_from_config(KDL::ChainFkSolverPos_recursive** fk_solvers, const double* configuration, double* pose, const unsigned int i_chain);

void compute_errors_from_poses(double* current_poses, double* desired_poses, double* past_desired_poses, double* Kp, Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, const double period, const unsigned int error_alg_id);

void compute_jacobians_from_config(KDL::JntArray* pepper_chains, KDL::ChainJntToJacSolver** jacobian_solvers, const double* configuration, KDL::Jacobian* KDL_jacobians);

// generate_pepper_random_pose
void generate_pepper_random_pose(double* random_configuration, double* random_poses);
void generate_pepper_random_pose(double* random_poses);
void generate_pepper_random_pose(KDL::Chain* pepper_chains, double* random_configuration, double* random_poses);
void generate_pepper_random_pose(KDL::Chain* pepper_chains, double* random_poses);
void generate_pepper_random_pose(KDL::ChainFkSolverPos_recursive** fk_solvers, double* random_configuration, double* random_poses);
void generate_pepper_random_pose(KDL::ChainFkSolverPos_recursive** fk_solvers, double* random_poses);

void pepperRosConfig_to_3ChainModelConfig(const double* pepperROSConfig2DPlatformPose, const double* pepperROSConfigJoints, double* pepper3ChainModel);
void pepper3ChainModelConfig_to_RosConfig(const double* pepper3ChainModel, double* pepperROSConfig2DPlatformPose, double* pepperROSConfigJoints);

#endif