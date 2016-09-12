// ----------------------------------------------------------------------------------------------------------------------------
//
// Code to initialize the kinematic chains of Pepper
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


#include <pepper_chain_init.hpp>


void initialize_kinematic_chains(KDL::Chain *chains){
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
//       8-13:   'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand',
//     LEFT ARM
//       14-19:  'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand',
// 
//  desired_poses: [{P_head, q_head}, {P_RightArm, q_RightArm}, {P_LeftArm, q_LeftArm}]
  
  
  
  // Variables ******************************************************************************************************************************************
  unsigned int ci = 0;
  
  
  // ****************************************************************************************************************************************************
  // HEAD ***********************************************************************************************************************************************
  ci = 0;
  // From platform to torso -----------------------------------------------------------------------------------------------------------------------------
#if PEPPER_PLATFORM_N_DOF == 3
  //   Platform
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation X
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation Y
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#elif PEPPER_PLATFORM_N_DOF == 0
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#endif
  //   Pepper
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.268))));		// KneePitch - 0.602 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.079))));		// HipPitch - 0.681 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(-0.038, 0.0, 0.3089))));	// HipRoll - 0.9899 m
  // From torso to head ---------------------------------------------------------------------------------------------------------------------------------
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// HeadYaw
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.1245))));		// HeadPitch
 
  
  // ****************************************************************************************************************************************************
  // RIGHT ARM ******************************************************************************************************************************************
  ci = 1;
  // From platform to torso -----------------------------------------------------------------------------------------------------------------------------
#if PEPPER_PLATFORM_N_DOF == 3
  //   Platform
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation X
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation Y
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#elif PEPPER_PLATFORM_N_DOF == 0
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#endif
  //   Pepper
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.268))));		// KneePitch - 0.602 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.079))));		// HipPitch - 0.681 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(-0.057, -0.14974, 0.22582))));	// HipRoll - 0.90682 m
  // From torso to right arm ----------------------------------------------------------------------------------------------------------------------------
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// RShoulderPitch
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0.0,-0.157079,0.0), KDL::Vector(0.1812, -0.015, 0.00013))));	// RShoulderRoll
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// RElbowYaw
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.15, 0.0, 0.0))));		// RElbowRoll
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.025, 0.0, 0.0))));		// RWristYaw


  // ****************************************************************************************************************************************************
  // LEFT ARM *******************************************************************************************************************************************
  ci = 2;
  // From platform to torso -----------------------------------------------------------------------------------------------------------------------------
#if PEPPER_PLATFORM_N_DOF == 3
  //   Platform
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation X
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation Y
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#elif PEPPER_PLATFORM_N_DOF == 0
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#endif
  //   Pepper
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.268))));		// KneePitch - 0.602 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.079))));		// HipPitch - 0.681 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(-0.057, 0.14974, 0.22582))));	// HipRoll - 0.90682 m
  // From torso to right arm ----------------------------------------------------------------------------------------------------------------------------
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// LShoulderPitch
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0.0,-0.157079,0.0), KDL::Vector(0.1812, 0.015, 0.00013))));	// LShoulderRoll  
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// LElbowYaw
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.15, 0.0, 0.0))));		// LElbowRoll
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.025, 0.0, 0.0))));		// LWristYaw
  
  
  // ****************************************************************************************************************************************************
  // TORSO **********************************************************************************************************************************************
  ci = 3;
  // From platform to torso -----------------------------------------------------------------------------------------------------------------------------
#if PEPPER_PLATFORM_N_DOF == 3
  //   Platform
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation X
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation Y
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#elif PEPPER_PLATFORM_N_DOF == 0
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#endif
  //   Pepper
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.268))));		// KneePitch - 0.602 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.079))));		// HipPitch - 0.681 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.2))));		// HipRoll - 0.9899 m
  
  
  // ****************************************************************************************************************************************************
  // RIGHT ELBOW ******************************************************************************************************************************************
  ci = 4;
  // From platform to torso -----------------------------------------------------------------------------------------------------------------------------
#if PEPPER_PLATFORM_N_DOF == 3
  //   Platform
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation X
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation Y
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#elif PEPPER_PLATFORM_N_DOF == 0
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#endif
  //   Pepper
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.268))));		// KneePitch - 0.602 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.079))));		// HipPitch - 0.681 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(-0.057, -0.14974, 0.22582))));	// HipRoll - 0.90682 m
  // From torso to right arm ----------------------------------------------------------------------------------------------------------------------------
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// RShoulderPitch
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0.0,-0.157079,0.0), KDL::Vector(0.1812, -0.015, 0.00013))));	// RShoulderRoll

  
  // ****************************************************************************************************************************************************
  // LEFT ARM *******************************************************************************************************************************************
  ci = 5;
  // From platform to torso -----------------------------------------------------------------------------------------------------------------------------
#if PEPPER_PLATFORM_N_DOF == 3
  //   Platform
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation X
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// Translation Y
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#elif PEPPER_PLATFORM_N_DOF == 0
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0, 0.0, 0.334))));		// Rotation around Z axis - 0.334 m
#endif
  //   Pepper
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.268))));		// KneePitch - 0.602 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.079))));		// HipPitch - 0.681 m
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(-0.057, 0.14974, 0.22582))));	// HipRoll - 0.90682 m
  // From torso to right arm ----------------------------------------------------------------------------------------------------------------------------
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));		// LShoulderPitch
  chains[ci].addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Rotation::RPY(0.0,-0.157079,0.0), KDL::Vector(0.1812, 0.015, 0.00013))));	// LShoulderRoll  
  
  
  return;
}