// ----------------------------------------------------------------------------------------------------------------------------
//
// Program to compute the inverse kinematics of Pepper for the wrists and the head, without modeling the hands

#include <iostream>
#include <fstream>


#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <pepper_nullemo/geometry_msgs_pose_vector.h>

#include <pepper_core.hpp>
#include <pepper_chain_init.hpp>
#include <pepper_ik_algs.hpp>
#include <pepper_trajectories_data.hpp>


#define LOOP_FREQUENCY 100


using namespace Eigen;



// ROS Subscribers --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool motion_initiated = false;
void PepperInitMotionCallback(const std_msgs::Bool msg){

  if (msg.data == true)	{
    motion_initiated = true;
    ROS_INFO("Pepper motion initiated");
  }
}


unsigned int trajectory_id = 0;
void PepperGeneratePosesTrajectoryCallback(const std_msgs::Int32 msg){
  
  trajectory_id = msg.data;
  
  ROS_INFO("pepper_emonull: Pepper poses trajectory selected. Trajectory Id = %d", trajectory_id);
  
  return;
}


double desired_pepper_poses[PEPPER_N_CHAINS*7], past_desired_pepper_poses[PEPPER_N_CHAINS*7]; // Position & Quaternion
// [pos_head, quat_head, pos_rightarm, quat_rightarm, pos_leftarm, quat_leftarm] = [ {px, py, pz, qx, qy, qz, qw} ]
void PepperDesiredPosesCallback(const pepper_nullemo::geometry_msgs_pose_vector::ConstPtr& msg){
  unsigned int vsize = msg->pose_vector.size();
  
  if ( msg->pose_vector.size() != PEPPER_N_CHAINS )
  {
    ROS_ERROR("pepper_emonull: Size of Pepper chains is incorrect!");
  }
  else
  {
    motion_initiated = true;
    
//     ROS_INFO("New Pepper poses received!");
    
    for (unsigned int i=0; i<PEPPER_N_CHAINS*7; i++)		past_desired_pepper_poses[i] = desired_pepper_poses[i];
    
    for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)
    {
      // Position
      desired_pepper_poses[7*i + 0] = msg->pose_vector[i].position.x;
      desired_pepper_poses[7*i + 1] = msg->pose_vector[i].position.y;
      desired_pepper_poses[7*i + 2] = msg->pose_vector[i].position.z;
      // Orientation
      desired_pepper_poses[7*i + 3] = msg->pose_vector[i].orientation.x;
      desired_pepper_poses[7*i + 4] = msg->pose_vector[i].orientation.y;
      desired_pepper_poses[7*i + 5] = msg->pose_vector[i].orientation.z;
      desired_pepper_poses[7*i + 6] = msg->pose_vector[i].orientation.w;
    }
  }
}


unsigned int dimension_selection[PEPPER_N_CHAINS*6];
void dimensionSelectionCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
  unsigned int i = 0;
  for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
    dimension_selection[i] = *it;
    i++;
  }

  return;
}


double dimension_selection_frames[PEPPER_N_CHAINS*8];
// 3 (head, right arm, left arm) x 2 (position, orientation) x [qx qy qz qw]
void dimensionSelectionFramesCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
  for(unsigned int i = 0; i<PEPPER_N_CHAINS*8; i++)		dimension_selection_frames[i] = array->data[i];

  return;
}


double gazedPosition[3];
void gazedPositionCallback(const geometry_msgs::Vector3 msg)
{
  gazedPosition[0] = msg.x;
  gazedPosition[1] = msg.y;
  gazedPosition[2] = msg.z;

  return;
}



int main(int argc, char** argv){
  
  // Initialize random seed:
  srand (time(NULL));
  
  double all_time = GetTimeMs64();
  double alg_time = 0;    
  
  
  // ROS -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  ros::init(argc, argv, "pepper_emonull");

  ros::NodeHandle node;
  ros::Rate loop_rate(LOOP_FREQUENCY);

  //   Topics
  ros::Subscriber init_motion_sub = node.subscribe("pepper_init_motion", 10, &PepperInitMotionCallback);
  ros::Subscriber poses_trajectories_sub = node.subscribe("pepper_generate_poses_trajectory", 10, &PepperGeneratePosesTrajectoryCallback);
  ros::Subscriber desired_poses_sub = node.subscribe("pepper_desired_poses", 10, &PepperDesiredPosesCallback);
  ros::Subscriber dimension_selection_sub = node.subscribe("pepper_dimension_selection", 100, dimensionSelectionCallback);
  ros::Subscriber dimension_selection_frames_sub = node.subscribe("pepper_dimension_selection_frames", 100, dimensionSelectionFramesCallback);
  ros::Subscriber gazed_position_sub = node.subscribe("pepper_gazed_position", 100, gazedPositionCallback);
  
  ros::Publisher pepper_platform_2Dpose_pub = node.advertise<geometry_msgs::Vector3>("pepper_platform_2Dpose", 1);
  ros::Publisher pepper_joint_pub = node.advertise<sensor_msgs::JointState>("pepper_joint_angles", 1);
  
  ros::Publisher test_value_pub_1 = node.advertise<std_msgs::Float64>("pepper_joint_test_1", 1);
  ros::Publisher test_value_pub_2 = node.advertise<std_msgs::Float64>("pepper_joint_test_2", 1);
  ros::Publisher test_value_pub_3 = node.advertise<std_msgs::Float64>("pepper_joint_test_3", 1);

  
  // Variables ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
  load_pepper_joint_values();
  
  
  Eigen::VectorXd past_configuration_inc(PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS);
  double past_diffH[PEPPER_TOTAL_N_JOINTS];
  
  double period = 1.0/(double)LOOP_FREQUENCY;
  
  double pepper_platform_2Dpose[PEPPER_PLATFORM_N_DOF];
  //   0:      'Pos X'
  //   1:      'Pos Y'
  //   2:      'Rotation around Z axis'
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF; i++)		pepper_platform_2Dpose[i] = 0.0;
  
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

  double current_configuration[PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS];
  //   0:'Pos X', 1:'Pos Y', 2:'Rotation around Z axis'
  //   Joint angles
  double past_current_configuration[PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS], past_past_current_configuration[PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS];
  
  // Initialize desired cartesian poses to 0
  for (unsigned int i=0; i<PEPPER_N_CHAINS*7; i++)	desired_pepper_poses[i] = 0.0;
  
  double righthand_jnt_val, lefthand_jnt_val;
  righthand_jnt_val = lefthand_jnt_val = 0.5;
  
  // Default gaze
  gazedPosition[0] = gazedPosition[1] = gazedPosition[2] = -1.0;
  
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	dimension_selection_frames[i] = 0.0;
  dimension_selection_frames[3] = dimension_selection_frames[7] = 1.0;		// Head
  dimension_selection_frames[11] = dimension_selection_frames[15] = 1.0;	// Right arm
  dimension_selection_frames[19] = dimension_selection_frames[23] = 1.0;	// Left arm
  
  
  //   ROS
  geometry_msgs::Vector3 rosPepperPlatform2DPose;
  rosPepperPlatform2DPose.x = 0.0;
  rosPepperPlatform2DPose.y = 0.0;
  rosPepperPlatform2DPose.z = 0.0;

  sensor_msgs::JointState rosPepperJoints;
  rosPepperJoints.position.resize(PEPPER_TOTAL_N_JOINTS);
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)		rosPepperJoints.position[i] = 0.0;

  
  //   KDL
  KDL::Chain* pepper_tree_chains = new KDL::Chain[PEPPER_N_CHAINS];
  initialize_kinematic_chains(pepper_tree_chains);
  
  //     Forward kinematics solvers
  KDL::ChainFkSolverPos_recursive** fk_solvers = new KDL::ChainFkSolverPos_recursive*[PEPPER_N_CHAINS];
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	  fk_solvers[i] = new KDL::ChainFkSolverPos_recursive(pepper_tree_chains[i]);
  
  //     Jacobian computation
  KDL::ChainJntToJacSolver** jacobian_solvers = new KDL::ChainJntToJacSolver*[PEPPER_N_CHAINS];
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	  jacobian_solvers[i] = new KDL::ChainJntToJacSolver(pepper_tree_chains[i]);

  
  // Statistical data
  double time0 = 0.0;
  double timeDiff = 0.0;
  double ik_average, ik_stdev;
  ik_average = 0.0;
  ik_stdev = 0.0;
  double time_init = GetTimeMs64();
  
  // Files
  // Test data
  double data = 0.0;
  std_msgs::Float64 test_value_1, test_value_2, test_value_3;
  
//   std::ofstream file_exp("/home/joseparnau/nullemo_data/nullemo.dat", std::ios::out);
//   std::ofstream file_jnt_pos("/home/joseparnau/nullemo_data/joint_pos.dat", std::ios::out);
//   std::ofstream file_jnt_vel("/home/joseparnau/nullemo_data/joint_vel.dat", std::ios::out);
//   std::ofstream file_jnt_acc("/home/joseparnau/nullemo_data/joint_acc.dat", std::ios::out);
  
  // Set default initial configuration as home  
  set_home_configuration(current_configuration);
  

//       std::cout << "000000000000" << std::endl;  
  
  // Loop ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  unsigned int n_it = 0;
  while (ros::ok()){
    
//       std::cout << "111111111111" << std::endl;
    
    
    update_time_increment();
    
    if (motion_initiated){
      n_it++;
      
      alg_time += (GetTimeMs64() - all_time)/1000.0;
      all_time  =  GetTimeMs64();	// ms
      
      
      if (n_it == 1)	// Set initial configurations
      {
	set_home_configuration(current_configuration);
	
	if ( ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR ) || ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY ) )
	{
	  // Left arm
	  current_configuration[13] = 54.0*(PI/180.0);
	  current_configuration[14] = 0.0;
	  current_configuration[15] = -PI/2;
	  current_configuration[16] = -PI/4;
	  current_configuration[17] = -PI/2;;
	}
	
	else if ( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD_HANDS_POS_GRASPCOORD )
	{
	  current_configuration[3] = 0.0*(PI/180.0);
	  current_configuration[4] = 0.0*(PI/180.0);
	  current_configuration[5] = 0.0*(PI/180.0);
	  current_configuration[6] = 0.0*(PI/180.0);
	  current_configuration[7] = -0.722845*(PI/180.0);
	  current_configuration[8] = 95.7708*(PI/180.0);
	  current_configuration[9] = -1.66546*(PI/180.0);
	  current_configuration[10] = 89.6637*(PI/180.0);
	  current_configuration[11] = 70.0611*(PI/180.0);
	  current_configuration[12] = 1.74987*(PI/180.0);
	  current_configuration[13] = 95.7708*(PI/180.0);
	  current_configuration[14] = 1.66546*(PI/180.0);
	  current_configuration[15] = -89.6637*(PI/180.0);
	  current_configuration[16] = -70.0611*(PI/180.0);
	  current_configuration[17] = -1.74987*(PI/180.0);
	}
      }
      
      
      if (n_it == 1)	for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)		past_past_current_configuration[i] = current_configuration[i];
      else		for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)		past_past_current_configuration[i] = past_current_configuration[i];
      for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)				past_current_configuration[i] = current_configuration[i];
      
      

      // Press Enter to advance TEST TEST TEST TEST TEST TEST TEST TEST
//       std::cin.ignore();

      // IK Time computation
      time0 = GetTimeMs64();	// TEST TEST TEST TEST
	
      compute_next_configuration(current_configuration, desired_pepper_poses, dimension_selection, dimension_selection_frames, gazedPosition,
				 fk_solvers, jacobian_solvers, past_desired_pepper_poses, past_configuration_inc, past_diffH, trajectory_id, period, alg_time, n_it, data);
      
      // IK Time computation
      timeDiff = GetTimeMs64() - time0;	// TEST TEST TEST TEST
      ik_average = (((double)n_it-1.0)/n_it)*ik_average + (1.0/(double)n_it)*timeDiff;
      if (n_it == 1)	ik_stdev = 0.0;
      else		ik_stdev = sqrt( (((double)n_it-1.0)/n_it)*ik_stdev*ik_stdev + (1.0/((double)n_it-1.0))*(timeDiff - ik_average)*(timeDiff - ik_average) );
//       std::cout << n_it << " :    " << timeDiff << "   /   " << ik_average << "  -  " << ik_stdev << std::endl;
//       file_exp << ik_time_inc << std::endl;

      
//       for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)	file_jnt_pos << std::setw(15) << current_configuration[i] << " ";      							file_jnt_pos << std::endl;
//       for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)	file_jnt_vel << std::setw(15) << (current_configuration[i]-past_current_configuration[i])/time_increment << " ";      	file_jnt_vel << std::endl;
//       for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)	
// 	file_jnt_acc << std::setw(15) << (current_configuration[i]-2.0*past_current_configuration[i]+past_past_current_configuration[i])/(time_increment*time_increment) << " ";      			file_jnt_acc << std::endl;
      
      
      for (unsigned int i=0; i<PEPPER_N_CHAINS*7; i++)		past_desired_pepper_poses[i] = desired_pepper_poses[i];
    }

    //   To ROS format
    pepper3ChainModelConfig_to_RosConfig(current_configuration, pepper_platform_2Dpose, pepper_joints);



    // Set rest of pepper joints  --------------------------------------------------------------------------------------------------------------------------------------------------  
    if ( (trajectory_id == RIGHTARM_WAVING) || (trajectory_id == RIGHTARM_WAVING_RELAXED) || (trajectory_id == RIGHTARM_WAVING_RELAXED_LOOK) )
    {
      // Left arm
      pepper_joints[5] = PI/2;
      pepper_joints[6] = PI/8;
      pepper_joints[8] = -PI/16;      
      
      righthand_jnt_val = 0.85;
      lefthand_jnt_val = 0.5;
    }
    else if ( (trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS) || (trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS_XY) )
    {
      // Left arm
      pepper_joints[5] = PI/2;
      pepper_joints[6] = PI/8;
      pepper_joints[8] = -PI/16;
      
      righthand_jnt_val = 0.5;
      lefthand_jnt_val = 0.5;
    }
    else if ( ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR ) || ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY ) )
    {
      // Right arm
      pepper_joints[11] = PI/2;
      pepper_joints[12] = -PI/8;
      pepper_joints[14] = PI/16;      
      
      righthand_jnt_val = 0.5;
      lefthand_jnt_val = 0.05;
    }    
    else if ( (trajectory_id == RIGHTHAND_SHAKE) || 
	      (trajectory_id == RIGHTHAND_SHAKE_POS_Z) || 
	      (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y) || 
	      (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_RIGHTARM_YZ) || 
	      (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ) || 
	      (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR) || 
	      (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WX_WY_RIGHTARM_YZ_OR) || 
	      (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ_HEAD_WY_WZ) )
    {
      // Left arm
      pepper_joints[5] = PI/2;
      pepper_joints[6] = PI/8;
      pepper_joints[8] = -PI/16;       
      
      righthand_jnt_val = 0.625;
      lefthand_jnt_val = 0.5;
    }
    else if ( ( trajectory_id == OBJECT_BOTH_HANDS ) ||
	      ( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD ) ||
	      ( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD_HANDS_POS_GRASPCOORD ) )
    {
      righthand_jnt_val = 0.85;
      lefthand_jnt_val = 0.85;
    }
    else if ( ( trajectory_id == HEAD_MOVE ) ||
	      ( trajectory_id == HEAD_MOVE_STARE ) ||
	      ( trajectory_id == HEAD_MOVE_STARE_FREEWX ) || 
	      ( trajectory_id == HEAD_CRAZY_ARCH_FREEWX ) ||
	      ( trajectory_id == HEAD_CRAZY_ARCH )||
	      ( trajectory_id == HEAD_GAZE ) )
    {
      // Right arm
      pepper_joints[11] = PI/2;
      pepper_joints[12] = -PI/8;
      pepper_joints[14] = PI/16;       
      
      // Left arm
      pepper_joints[5] = PI/2;
      pepper_joints[6] = PI/8;
      pepper_joints[8] = -PI/16;
      
      righthand_jnt_val = 0.5;
      lefthand_jnt_val = 0.5;
    }
    else
    {
      righthand_jnt_val = 0.5;
      lefthand_jnt_val = 0.5;
    }
    
    //   Right hand
    pepper_joints[17] = righthand_jnt_val;
    pepper_joints[19] = righthand_jnt_val;
    pepper_joints[20] = righthand_jnt_val;
    pepper_joints[22] = righthand_jnt_val;
    pepper_joints[24] = righthand_jnt_val;
    pepper_joints[28] = righthand_jnt_val;
    pepper_joints[30] = righthand_jnt_val;
    pepper_joints[32] = righthand_jnt_val;
    pepper_joints[34] = righthand_jnt_val;
    pepper_joints[37] = righthand_jnt_val;
    pepper_joints[39] = righthand_jnt_val;
    pepper_joints[40] = righthand_jnt_val;
    pepper_joints[42] = righthand_jnt_val;
    pepper_joints[43] = righthand_jnt_val; 
    //   Left hand
    pepper_joints[18] = lefthand_jnt_val;
    pepper_joints[21] = lefthand_jnt_val;
    pepper_joints[23] = lefthand_jnt_val;
    pepper_joints[25] = lefthand_jnt_val;
    pepper_joints[26] = lefthand_jnt_val;
    pepper_joints[27] = lefthand_jnt_val;
    pepper_joints[29] = lefthand_jnt_val;
    pepper_joints[31] = lefthand_jnt_val;
    pepper_joints[33] = lefthand_jnt_val;
    pepper_joints[35] = lefthand_jnt_val;
    pepper_joints[36] = lefthand_jnt_val;
    pepper_joints[38] = lefthand_jnt_val;
    pepper_joints[41] = lefthand_jnt_val;
    pepper_joints[44] = lefthand_jnt_val;
    //   Wheels
    pepper_joints[45] = pepper_joints[46] = pepper_joints[47] = 0.0;

    
    
    // Publish -----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    //   Pepper
    //     Platform
    rosPepperPlatform2DPose.x = pepper_platform_2Dpose[0];
    rosPepperPlatform2DPose.y = pepper_platform_2Dpose[1];
    rosPepperPlatform2DPose.z = pepper_platform_2Dpose[2];
    pepper_platform_2Dpose_pub.publish(rosPepperPlatform2DPose);
    //     Joints
    rosPepperJoints.header.stamp = ros::Time::now();
    for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)	rosPepperJoints.position[i] = pepper_joints[i];
    pepper_joint_pub.publish(rosPepperJoints);


    // TEST TEST
    test_value_1.data = current_configuration[6];
    test_value_2.data = current_configuration[7];
    test_value_3.data = data;
    test_value_pub_1.publish(test_value_1);
    test_value_pub_2.publish(test_value_2);
    test_value_pub_3.publish(test_value_3);

    
    if ( (GetTimeMs64() - time_init) > 600000 )		break;
    
    ros::spinOnce(); 
    loop_rate.sleep();
  }

  std::cout << n_it << " :  " << GetTimeMs64() - time_init << "   /   " << ik_average << "  -  " << ik_stdev << std::endl;
  

//   file_exp.close();
//   file_jnt_pos.close();
//   file_jnt_vel.close();
//   file_jnt_acc.close();
  
    
  return 0;
}