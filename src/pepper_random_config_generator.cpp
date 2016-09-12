// ----------------------------------------------------------------------------------------------------------------------------
//
// Program to generate a random pepper configuration (no platform values and no hands) each time Enter is pressed


#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

#include <pepper_nullemo/pepper_rnd_cfg_gen.h>

#include <pepper_core.hpp>

#define LOOP_FREQUENCY 10



bool generate_rnd_cfg = false;
bool PepperGenerateRandomConfigCallback(pepper_nullemo::pepper_rnd_cfg_gen::Request  &req,
					pepper_nullemo::pepper_rnd_cfg_gen::Response &res){

  if (req.new_config == true)	{
    ROS_INFO("Random configuration generated");
    generate_rnd_cfg = true;
  }
  
  return true;
}


int main(int argc, char** argv){
  
  // Initialize random seed:
  srand (time(NULL));
  
  // ROS
  ros::init(argc, argv, "pepper_random_config_generator");

  ros::NodeHandle node;
  ros::Rate loop_rate(LOOP_FREQUENCY);
  
  // Serveis
  ros::ServiceServer random_config_service = node.advertiseService("pepper_generate_random_config", PepperGenerateRandomConfigCallback);

  //   Topics
  ros::Publisher pepper_platform_2Dpose_pub = node.advertise<geometry_msgs::Vector3>("pepper_platform_2Dpose", 1);
  ros::Publisher pepper_joint_pub = node.advertise<sensor_msgs::JointState>("pepper_joint_angles", 1);


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

  double current_configuration[PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS];
  //   0:'Pos X', 1:'Pos Y', 2:'Rotation around Z axis'
  //   Joint angles
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)		current_configuration[i] = 0.0;

  
  //   ROS
  geometry_msgs::Vector3 rosPepperPlatform2DPose;
  rosPepperPlatform2DPose.x = 0.0;
  rosPepperPlatform2DPose.y = 0.0;
  rosPepperPlatform2DPose.z = 0.0;

  sensor_msgs::JointState rosPepperJoints;
  rosPepperJoints.position.resize(PEPPER_TOTAL_N_JOINTS);
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)		rosPepperJoints.position[i] = 0.0;

  
  // Loop -----------------------------------------------------------------------------------------------------------------------
  while (ros::ok()){

      
    // Random configuration generation
    if (generate_rnd_cfg == true)
    {
      generate_pepper_random_configuration(current_configuration);

      generate_rnd_cfg = false;


      // Publish -----------------------------------------------------------------------------------------------------------------
      
      //   To ROS format
      pepper3ChainModelConfig_to_RosConfig(current_configuration, pepper_platform_2Dpose, pepper_joints);
      for (unsigned int i=16; i<PEPPER_TOTAL_N_JOINTS; i++)	pepper_joints[i] = 0.0;
      
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
    }
    
    
    ros::spinOnce(); 
    loop_rate.sleep();
  }

    
  return 0;
}