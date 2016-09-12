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

#define LOOP_FREQUENCY 1


bool generate_rnd_pose = false;
bool PepperGenerateRandomPoseCallback(pepper_nullemo::pepper_rnd_pose_gen::Request  &req,
				      pepper_nullemo::pepper_rnd_pose_gen::Response &res){
  
  if (req.new_pose == true)
  {
    ROS_INFO("Random poses generated");
    generate_rnd_pose = true;
  }
  
  return true;
}


int main(int argc, char** argv){
  
  // Initialize random seed:
  srand (time(NULL));

  
  // ROS
  ros::init(argc, argv, "pepper_random_poses_generator");

  ros::NodeHandle node;
  ros::Rate loop_rate(LOOP_FREQUENCY);
  
  // Serveis
  ros::ServiceServer random_pose_service = node.advertiseService("pepper_generate_random_poses", PepperGenerateRandomPoseCallback);

  //   Topics
  ros::Publisher pepper_platform_2Dpose_pub = node.advertise<geometry_msgs::Vector3>("pepper_platform_2Dpose", 1);
  ros::Publisher pepper_joint_pub = node.advertise<sensor_msgs::JointState>("pepper_joint_angles", 1);
  ros::Publisher pepper_rnd_pose_pub = node.advertise<pepper_nullemo::geometry_msgs_pose_vector>("pepper_random_poses", 1);


  // Variables ----------------------------------------------------------------------------------------------------------------
  load_pepper_joint_values();
  
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

  double random_configuration[PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS];
  // Platform values set to zero
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)		random_configuration[i] = 0.0;

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
    if (generate_rnd_pose == true)
    {
      // Generate random 2D platform pose inside a 10x10 m2 square
      random_configuration[0] = -5.0 + 10.0*((double)rand() / (double)(RAND_MAX));
      random_configuration[1] = -5.0 + 10.0*((double)rand() / (double)(RAND_MAX));
      random_configuration[2] = 2.0*PI*((double)rand() / (double)(RAND_MAX));
      
      
      generate_pepper_random_pose(fk_solvers, random_configuration, pepper_poses);
      
      generate_rnd_pose = false;
 
      
      // Publish -----------------------------------------------------------------------------------------------------------------
      
      //   To ROS format
      pepper3ChainModelConfig_to_RosConfig(random_configuration, pepper_platform_2Dpose, pepper_joints);
      for (unsigned int i=16; i<PEPPER_TOTAL_N_JOINTS; i++)	pepper_joints[i] = 0.0;
      
      //   Configuration
      //     Platform
      rosPepperPlatform2DPose.x = pepper_platform_2Dpose[0];
      rosPepperPlatform2DPose.y = pepper_platform_2Dpose[1];
      rosPepperPlatform2DPose.z = pepper_platform_2Dpose[2];
      pepper_platform_2Dpose_pub.publish(rosPepperPlatform2DPose);
      //     Joints
      rosPepperJoints.header.stamp = ros::Time::now();
      for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)	rosPepperJoints.position[i] = pepper_joints[i];
      pepper_joint_pub.publish(rosPepperJoints);
      
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

      
      pepper_rnd_pose_pub.publish(rosPepperPoses);
    }
    
    
    ros::spinOnce(); 
    loop_rate.sleep();
  }

    
  return 0;
}