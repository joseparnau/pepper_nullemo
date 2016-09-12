// ----------------------------------------------------------------------------------------------------------------------------
//
// Functions with some tasks
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


#include <pepper_nullemo_tasks.hpp>



void compute_next_task(double* current_poses,
		       double* pepper_poses, unsigned int* dimension_selection, double* dimension_selection_frames, double* gazed_position, 
		       const double trajectory_time, const unsigned int trajectory_id
		      )
{
  
  // Initialization
  for (unsigned int i=0; i<PEPPER_N_CHAINS*7; i++)	pepper_poses[i] = current_poses[i];
  for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)		dimension_selection[i] = (unsigned int)0;
  for (unsigned int i=0; i<PEPPER_N_TCP*8; i++)		dimension_selection_frames[i] = 0.0;
  dimension_selection_frames[3] = dimension_selection_frames[7] = dimension_selection_frames[11] = dimension_selection_frames[15] = dimension_selection_frames[19] = dimension_selection_frames[23] = 1.0;
 
  
  // Generate a simple circular trajectory in the YZ plane for the arms and the head
  if (trajectory_id == HEADARM_POSES_CIRCLES)		// ***********************************************************************************************************************************************
  {
    // Kp = 0.20;
    // Kv = 0.0;
    // Ka = 7.0;
    
    double ang_vel = PI/2;	// rad/s
    double radius  = 0.2;	// m
    
    double basePos[3], rightarmPosInc[3], leftPosInc[3];
    basePos[0] = 1.0;
    basePos[1] = 0.0  + radius*cos(ang_vel * trajectory_time);
    basePos[2] = 0.75 + radius*sin(ang_vel * trajectory_time);
    rightarmPosInc[0] = 0.0;
    rightarmPosInc[1] = -0.2;
    rightarmPosInc[2] = 0.0;
    leftPosInc[0] = 0.0;
    leftPosInc[1] = 0.2;
    leftPosInc[2] = 0.0;
    
    
    // Desired poses
    //   Right arm - Position
    dimension_selection[6] = dimension_selection[7] = dimension_selection[8] = 1.0;
    for (unsigned int i=0; i<3; i++)		pepper_poses[7+i] = basePos[i] + rightarmPosInc[i];
    //   Left arm - Position
    dimension_selection[12] = dimension_selection[13] = dimension_selection[14] = 1.0;
    for (unsigned int i=0; i<3; i++)		pepper_poses[14+i] = basePos[i] + leftPosInc[i];
    
    
    // Gazed position
    gazed_position[0] = 10;
    gazed_position[1] = 0;
    gazed_position[2] = 1.1;    
  }
  
  
  else if (trajectory_id == RIGHTARM_WAVING)		// ***********************************************************************************************************************************************
  {
    // Kp = 0.20;
    // Kv = 0.0;
    // Ka = 7.0;
    
    // Right arm wave
#if PEPPER_PLATFORM_N_DOF == 3
    double ang_vel = 150.0*(PI/180.0);	// rad/s
    double wRad    = 0.25;	// m
    double ang_max = 40.0*(PI/180);	// rad
    double rot_ang = 25.0*(PI/180.0) + ang_max * sin(ang_vel * trajectory_time);
#elif PEPPER_PLATFORM_N_DOF == 0
    double ang_vel = 150.0*(PI/180.0);	// rad/s
    double wRad    = 0.25;	// m
    double ang_max = 40.0*(PI/180);	// rad
    double rot_ang = 25.0*(PI/180.0) + ang_max * sin(ang_vel * trajectory_time);
#endif    
    
    Eigen::MatrixXd waveP(4,4);
    waveP.col(0) << 0.0, -sin(rot_ang), cos(rot_ang), 0.0;
    waveP.col(1) << 0.0, cos(rot_ang), sin(rot_ang), 0.0;
    waveP.col(2) << -1.0, 0.0, 0.0, 0.0;
    waveP.col(3) << 0.0, -0.15 - wRad*sin(rot_ang), 0.9 + wRad*cos(rot_ang), 1.0;
    
 
    // Desired poses
    //   Head - Orientation
    dimension_selection[3] = dimension_selection[4] = dimension_selection[5] = 1.0;
    pepper_poses[3] = 0.0;
    pepper_poses[4] = 0.0;
    pepper_poses[5] = 0.0;
    pepper_poses[6] = 1.0;
    
    //   Right arm
    //     Position X, Y, Z
    dimension_selection[6] = dimension_selection[7] = dimension_selection[8] = 1.0;
//     dimension_selection[6] = 0.0;
#if PEPPER_PLATFORM_N_DOF == 3
    pepper_poses[7] = 0.0;
    pepper_poses[8] = waveP(1,3);
    pepper_poses[9] = waveP(2,3);
#elif PEPPER_PLATFORM_N_DOF == 0
    pepper_poses[7] = 0.05;
    pepper_poses[8] = -0.12 + waveP(1,3);
    pepper_poses[9] = -0.05 + waveP(2,3);
#endif
    //     Orientation
    dimension_selection[9] = dimension_selection[10] = dimension_selection[11] = 1.0;
    pepper_poses[10] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).x();
    pepper_poses[11] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).y();
    pepper_poses[12] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).z();
    pepper_poses[13] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).w();
    
    // Gazed position
    gazed_position[0] = 10;
    gazed_position[1] = 0;
    gazed_position[2] = 1.1;    
  }
  
  
  // Gazed position	TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST
  double ang_vel = PI/4;	// rad/s
  double radius = 0.0;		// m
  
  gazed_position[0] =  3.0 + radius * fabs(cos(ang_vel * trajectory_time));
  gazed_position[1] =  0.0 - radius *      sin(ang_vel * trajectory_time);
  gazed_position[2] =  1.1;
  // Gazed position	TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST
  
  
  return;
}