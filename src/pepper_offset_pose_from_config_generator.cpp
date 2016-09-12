// ----------------------------------------------------------------------------------------------------------------------------
//
// Program to generate a random pepper configuration (no platform values and no hands) each time Enter is pressed


#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

#include <pepper_nullemo/pepper_rnd_pose_gen.h>

#include <pepper_nullemo/geometry_msgs_pose_vector.h>

#include <pepper_core.hpp>
#include <pepper_chain_init.hpp>

#define LOOP_FREQUENCY 60


// ROS Subscribers ------------------------------------------------------------------------------------------------------------
bool generate_offset_pose = false;
bool PepperGenerateOffsetPoseCallback(pepper_nullemo::pepper_rnd_pose_gen::Request  &req,
				      pepper_nullemo::pepper_rnd_pose_gen::Response &res){

  if (req.new_pose == true)	{
    ROS_INFO("Offset poses generated");
    generate_offset_pose = true;
  }
  
  return true;
}


double pepper_platform_2Dpose[PEPPER_PLATFORM_N_DOF];
//   0:      'Pos X'
//   1:      'Pos Y'
//   2:      'Rotation around Z axis'
void PepperPlatform2DPoseCallback(const geometry_msgs::Vector3 msg){

  pepper_platform_2Dpose[0] = msg.x;
  pepper_platform_2Dpose[1] = msg.y;
  pepper_platform_2Dpose[2] = msg.z;	// Rotation around Z axis
}


double pepper_joints[PEPPER_TOTAL_N_JOINTS];
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
void PepperConfigCallback(const sensor_msgs::JointState msg){

  if ( msg.position.size() != PEPPER_TOTAL_N_JOINTS )
    ROS_ERROR("Size of Pepper joint vector is incorrect!");
  else
  {
    for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)	pepper_joints[i] = msg.position[i];
  }
}



int main(int argc, char** argv){
  
  // Initialize random seed:
  srand (time(NULL));

  
  // ROS
  ros::init(argc, argv, "pepper_offset_pose_from_config_generator");

  ros::NodeHandle node;
  ros::Rate loop_rate(LOOP_FREQUENCY);
  
  // Serveis
  ros::ServiceServer random_pose_service = node.advertiseService("pepper_generate_offset_poses", PepperGenerateOffsetPoseCallback);

  //   Topics
  ros::Subscriber pepper_2Dpose_sub = node.subscribe("pepper_platform_2Dpose", 10, &PepperPlatform2DPoseCallback);
  ros::Subscriber pepper_config_sub = node.subscribe("pepper_joint_angles", 10, &PepperConfigCallback);
  
  ros::Publisher pepper_offset_platform_2Dpose_pub = node.advertise<geometry_msgs::Vector3>("pepper_offset_platform_2Dpose", 1);
  ros::Publisher pepper_offset_joint_pub = node.advertise<sensor_msgs::JointState>("pepper_offset_joint_angles", 1);
  ros::Publisher pepper_offset_pose_pub = node.advertise<pepper_nullemo::geometry_msgs_pose_vector>("pepper_offset_random_poses", 1);


  // Variables ----------------------------------------------------------------------------------------------------------------
  load_pepper_joint_values();
  
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF; i++)		pepper_platform_2Dpose[i] = 0.0;
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)		pepper_joints[i] = 0.0;

  double offset_configuration[PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS];
  // Platform values set to zero
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)		offset_configuration[i] = 0.0;

  double pepper_poses[PEPPER_N_CHAINS*7], pose_aux[7];
  // [pos_head, quat_head, pos_rightarm, quat_rightarm, pos_leftarm, quat_leftarm] = [ {px, py, pz, qx, qy, qz, qw} ]
  
  //   ROS
  geometry_msgs::Vector3 rosPepperPlatform2DPose;
  rosPepperPlatform2DPose.x = 0.0;
  rosPepperPlatform2DPose.y = 0.0;
  rosPepperPlatform2DPose.z = 0.0;

  sensor_msgs::JointState rosPepperJoints;
  rosPepperJoints.position.resize(PEPPER_TOTAL_N_JOINTS);
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)		rosPepperJoints.position[i] = 0.0;
  geometry_msgs::Pose ROSPoseAux;

  //   KDL
  KDL::Chain* pepper_tree_chains = new KDL::Chain[PEPPER_N_CHAINS];
  initialize_kinematic_chains(pepper_tree_chains);
  
  KDL::ChainFkSolverPos_recursive** fk_solvers = new KDL::ChainFkSolverPos_recursive*[PEPPER_N_CHAINS];
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	  fk_solvers[i] = new KDL::ChainFkSolverPos_recursive(pepper_tree_chains[i]);  
 
  
  // Loop -----------------------------------------------------------------------------------------------------------------------
  while (ros::ok()){

    // Random configuration generation
    if (generate_offset_pose == true)
    {
      // From ROS format
      pepperRosConfig_to_3ChainModelConfig(pepper_platform_2Dpose, pepper_joints, offset_configuration);
      
      
      add_random_offset_to_pepper_configuration(offset_configuration, 0.25);
      
      compute_poses_from_config(offset_configuration, pepper_poses);
      
      generate_offset_pose = false;

      
      // Publish -----------------------------------------------------------------------------------------------------------------
      
      //   To ROS format
      pepper3ChainModelConfig_to_RosConfig(offset_configuration, pepper_platform_2Dpose, pepper_joints);
      for (unsigned int i=16; i<PEPPER_TOTAL_N_JOINTS; i++)	pepper_joints[i] = 0.0;
      
      //   Configuration
      //     Platform
      rosPepperPlatform2DPose.x = pepper_platform_2Dpose[0];
      rosPepperPlatform2DPose.y = pepper_platform_2Dpose[1];
      rosPepperPlatform2DPose.z = pepper_platform_2Dpose[2];
      pepper_offset_platform_2Dpose_pub.publish(rosPepperPlatform2DPose);
      //     Joints
      rosPepperJoints.header.stamp = ros::Time::now();
      for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)	rosPepperJoints.position[i] = pepper_joints[i];
      pepper_offset_joint_pub.publish(rosPepperJoints);
      
      //   Poses
      pepper_nullemo::geometry_msgs_pose_vector rosPepperPoses;
      //     Head
      for (unsigned int i=0; i<7; i++)		pose_aux[i] = pepper_poses[i];
      ROSPoseAux.position.x = pose_aux[0];
      ROSPoseAux.position.y = pose_aux[1];
      ROSPoseAux.position.z = pose_aux[2];
      ROSPoseAux.orientation.x = pose_aux[3];
      ROSPoseAux.orientation.y = pose_aux[4];
      ROSPoseAux.orientation.z = pose_aux[5];
      ROSPoseAux.orientation.w = pose_aux[6];
      rosPepperPoses.pose_vector.push_back(ROSPoseAux);
      //     Right arm
      for (unsigned int i=0; i<7; i++)		pose_aux[i] = pepper_poses[i+7];
      ROSPoseAux.position.x = pose_aux[0];
      ROSPoseAux.position.y = pose_aux[1];
      ROSPoseAux.position.z = pose_aux[2];
      ROSPoseAux.orientation.x = pose_aux[3];
      ROSPoseAux.orientation.y = pose_aux[4];
      ROSPoseAux.orientation.z = pose_aux[5];
      ROSPoseAux.orientation.w = pose_aux[6];
      rosPepperPoses.pose_vector.push_back(ROSPoseAux);
      //     Left arm
      for (unsigned int i=0; i<7; i++)		pose_aux[i] = pepper_poses[i+14];
      ROSPoseAux.position.x = pose_aux[0];
      ROSPoseAux.position.y = pose_aux[1];
      ROSPoseAux.position.z = pose_aux[2];
      ROSPoseAux.orientation.x = pose_aux[3];
      ROSPoseAux.orientation.y = pose_aux[4];
      ROSPoseAux.orientation.z = pose_aux[5];
      ROSPoseAux.orientation.w = pose_aux[6];
      rosPepperPoses.pose_vector.push_back(ROSPoseAux);
      //     Torso
      for (unsigned int i=0; i<7; i++)		pose_aux[i] = pepper_poses[i+21];
      ROSPoseAux.position.x = pose_aux[0];
      ROSPoseAux.position.y = pose_aux[1];
      ROSPoseAux.position.z = pose_aux[2];
      ROSPoseAux.orientation.x = pose_aux[3];
      ROSPoseAux.orientation.y = pose_aux[4];
      ROSPoseAux.orientation.z = pose_aux[5];
      ROSPoseAux.orientation.w = pose_aux[6];
      rosPepperPoses.pose_vector.push_back(ROSPoseAux);
      //     Right elbow
      for (unsigned int i=0; i<7; i++)		pose_aux[i] = pepper_poses[i+28];
      ROSPoseAux.position.x = pose_aux[0];
      ROSPoseAux.position.y = pose_aux[1];
      ROSPoseAux.position.z = pose_aux[2];
      ROSPoseAux.orientation.x = pose_aux[3];
      ROSPoseAux.orientation.y = pose_aux[4];
      ROSPoseAux.orientation.z = pose_aux[5];
      ROSPoseAux.orientation.w = pose_aux[6];
      rosPepperPoses.pose_vector.push_back(ROSPoseAux);
      //     Left elbow
      for (unsigned int i=0; i<7; i++)		pose_aux[i] = pepper_poses[i+35];
      ROSPoseAux.position.x = pose_aux[0];
      ROSPoseAux.position.y = pose_aux[1];
      ROSPoseAux.position.z = pose_aux[2];
      ROSPoseAux.orientation.x = pose_aux[3];
      ROSPoseAux.orientation.y = pose_aux[4];
      ROSPoseAux.orientation.z = pose_aux[5];
      ROSPoseAux.orientation.w = pose_aux[6];
      rosPepperPoses.pose_vector.push_back(ROSPoseAux);

      pepper_offset_pose_pub.publish(rosPepperPoses);
    }
    
    
    ros::spinOnce(); 
    loop_rate.sleep();
  }

    
  return 0;
}