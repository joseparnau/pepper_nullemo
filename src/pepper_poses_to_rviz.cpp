// ----------------------------------------------------------------------------------------------------------------------------
//
// Publisher to RVIZ of the Pepper TF transforms
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



#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/MarkerArray.h>

#include <pepper_nullemo/geometry_msgs_pose_vector.h>

#include <pepper_core.hpp>


#define LOOP_FREQUENCY 100



// ROS Subscribers ------------------------------------------------------------------------------------------------------------
bool new_poses_received = false;
double pepper_poses[PEPPER_N_CHAINS*7]; // Position & Quaternion
// [pos_head, quat_head, pos_rightarm, quat_rightarm, pos_leftarm, quat_leftarm] = [ {px, py, pz, qx, qy, qz, qw} ]
void PepperDesiredPosesCallback(const pepper_nullemo::geometry_msgs_pose_vector::ConstPtr& msg){

  if ( msg->pose_vector.size() != PEPPER_N_CHAINS )
  {
    ROS_ERROR("Size of Pepper chains is incorrect!");
  }
  else
  {    
    for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)
    {
      // Position
      pepper_poses[7*i + 0] = msg->pose_vector[i].position.x;
      pepper_poses[7*i + 1] = msg->pose_vector[i].position.y;
      pepper_poses[7*i + 2] = msg->pose_vector[i].position.z;
      // Orientation
      pepper_poses[7*i + 3] = msg->pose_vector[i].orientation.x;
      pepper_poses[7*i + 4] = msg->pose_vector[i].orientation.y;
      pepper_poses[7*i + 5] = msg->pose_vector[i].orientation.z;
      pepper_poses[7*i + 6] = msg->pose_vector[i].orientation.w;
    }
    new_poses_received = true;
  }
}


int main(int argc, char** argv){
  
  // ROS
  ros::init(argc, argv, "pepper_poses_to_rviz");

  ros::NodeHandle node;
  ros::Rate loop_rate(LOOP_FREQUENCY);
  
  //   Topics
  ros::Subscriber desired_poses_sub = node.subscribe("pepper_poses", 10, &PepperDesiredPosesCallback);

  //   Markers
  ros::Publisher marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  
  
  // Variables ----------------------------------------------------------------------------------------------------------------
  double arrow_scale[3];
  arrow_scale[0] = 0.2;	// Length
  arrow_scale[1] = 0.025;
  arrow_scale[2] = 0.025;
  
  // Initialize desired cartesian poses to 0
  for (unsigned int i=0; i<PEPPER_N_CHAINS*7; i++)		pepper_poses[i] = 0.0;
  
  
  // Loop ***********************************************************************************************************
  while (ros::ok()){
    
    if (new_poses_received){ 
      
      unsigned int marker_id = 0;
      Eigen::Matrix3d auxMat;
      Eigen::Quaternion<double> auxQuat;
      
      visualization_msgs::MarkerArray marker_array;
      
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::ADD;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.header.frame_id = "/platf_coordX_link";
      marker.header.stamp = ros::Time::now();
      marker.ns = "pose_shapes";
      
      marker.scale.x = arrow_scale[0];
      marker.scale.y = arrow_scale[1];
      marker.scale.z = arrow_scale[1];

      marker.lifetime = ros::Duration();
      
      
      // HEAD Marker ------------------------------------------------------------------------------------------------
      //   Arrow - X axis
      marker.id = marker_id++;
      marker.pose.position.x = pepper_poses[0];
      marker.pose.position.y = pepper_poses[1];
      marker.pose.position.z = pepper_poses[2];
      marker.pose.orientation.x = pepper_poses[3];
      marker.pose.orientation.y = pepper_poses[4];
      marker.pose.orientation.z = pepper_poses[5];
      marker.pose.orientation.w = pepper_poses[6];
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);   
      
      // RIGHT ARM Marker ------------------------------------------------------------------------------------------------
      //   Arrow - -Z axis
      marker.id = marker_id++;
      marker.pose.position.x = pepper_poses[7];
      marker.pose.position.y = pepper_poses[8];
      marker.pose.position.z = pepper_poses[9];
      auxQuat.x() = pepper_poses[10];
      auxQuat.y() = pepper_poses[11];
      auxQuat.z() = pepper_poses[12];
      auxQuat.w() = pepper_poses[13];
      auxMat.col(0) = -auxQuat.toRotationMatrix().col(2);
      auxMat.col(1) = auxQuat.toRotationMatrix().col(1);
      auxMat.col(2) = auxMat.col(0).cross(auxMat.col(1));
      marker.pose.orientation.x = Eigen::Quaternion<double>(auxMat).x();
      marker.pose.orientation.y = Eigen::Quaternion<double>(auxMat).y();
      marker.pose.orientation.z = Eigen::Quaternion<double>(auxMat).z();
      marker.pose.orientation.w = Eigen::Quaternion<double>(auxMat).w();
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);   
      
      // LEFT ARM Marker ------------------------------------------------------------------------------------------------
      //   Arrow - -Z axis
      marker.id = marker_id++;
      marker.pose.position.x = pepper_poses[14];
      marker.pose.position.y = pepper_poses[15];
      marker.pose.position.z = pepper_poses[16];
      auxQuat.x() = pepper_poses[17];
      auxQuat.y() = pepper_poses[18];
      auxQuat.z() = pepper_poses[19];
      auxQuat.w() = pepper_poses[20];
      auxMat.col(0) = -auxQuat.toRotationMatrix().col(2);
      auxMat.col(1) = auxQuat.toRotationMatrix().col(1);
      auxMat.col(2) = auxMat.col(0).cross(auxMat.col(1));
      marker.pose.orientation.x = Eigen::Quaternion<double>(auxMat).x();
      marker.pose.orientation.y = Eigen::Quaternion<double>(auxMat).y();
      marker.pose.orientation.z = Eigen::Quaternion<double>(auxMat).z();
      marker.pose.orientation.w = Eigen::Quaternion<double>(auxMat).w();
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);   
      
      // RIGHT ELBOW Marker ------------------------------------------------------------------------------------------------
      //   Arrow - -Z axis
      marker.id = marker_id++;
      marker.pose.position.x = pepper_poses[28];
      marker.pose.position.y = pepper_poses[29];
      marker.pose.position.z = pepper_poses[30];
      auxQuat.x() = pepper_poses[31];
      auxQuat.y() = pepper_poses[32];
      auxQuat.z() = pepper_poses[33];
      auxQuat.w() = pepper_poses[34];
      auxMat.col(0) = -auxQuat.toRotationMatrix().col(2);
      auxMat.col(1) = auxQuat.toRotationMatrix().col(1);
      auxMat.col(2) = auxMat.col(0).cross(auxMat.col(1));
      marker.pose.orientation.x = Eigen::Quaternion<double>(auxMat).x();
      marker.pose.orientation.y = Eigen::Quaternion<double>(auxMat).y();
      marker.pose.orientation.z = Eigen::Quaternion<double>(auxMat).z();
      marker.pose.orientation.w = Eigen::Quaternion<double>(auxMat).w();      
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker); 

      // RIGHT ELBOW Marker ------------------------------------------------------------------------------------------------
      //   Arrow - -Z axis
      marker.id = marker_id++;
      marker.pose.position.x = pepper_poses[35];
      marker.pose.position.y = pepper_poses[36];
      marker.pose.position.z = pepper_poses[37];
      auxQuat.x() = pepper_poses[38];
      auxQuat.y() = pepper_poses[39];
      auxQuat.z() = pepper_poses[40];
      auxQuat.w() = pepper_poses[41];
      auxMat.col(0) = -auxQuat.toRotationMatrix().col(2);
      auxMat.col(1) = auxQuat.toRotationMatrix().col(1);
      auxMat.col(2) = auxMat.col(0).cross(auxMat.col(1));
      marker.pose.orientation.x = Eigen::Quaternion<double>(auxMat).x();
      marker.pose.orientation.y = Eigen::Quaternion<double>(auxMat).y();
      marker.pose.orientation.z = Eigen::Quaternion<double>(auxMat).z();
      marker.pose.orientation.w = Eigen::Quaternion<double>(auxMat).w();
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker);      
      
      // TORSO Marker ------------------------------------------------------------------------------------------------
      //   Arrow - X axis - Arrow is longer
      marker.id = marker_id++;
      marker.pose.position.x = pepper_poses[21];
      marker.pose.position.y = pepper_poses[22];
      marker.pose.position.z = pepper_poses[23];
      marker.pose.orientation.x = pepper_poses[24];
      marker.pose.orientation.y = pepper_poses[25];
      marker.pose.orientation.z = pepper_poses[26];
      marker.pose.orientation.w = pepper_poses[27];
      marker.scale.x = 1.3*arrow_scale[0];
      marker.scale.y = arrow_scale[1];
      marker.scale.z = arrow_scale[1];
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker_array.markers.push_back(marker); 
      
      // Publish
      marker_array_pub.publish(marker_array);
    }
    
    ros::spinOnce(); 
    loop_rate.sleep();
  }

    
  return 0;
}