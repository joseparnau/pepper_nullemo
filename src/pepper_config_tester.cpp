// ----------------------------------------------------------------------------------------------------------------------------
//
// Program to generate a random pepper configuration (no platform values and no hands) each time Enter is pressed


#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <pepper_core.hpp>
#include <pepper_nullemo_aux.hpp>
#include <pepper_chain_init.hpp>

#define LOOP_FREQUENCY 100


double emot_param[3];
void PepperEmotParamCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
  for(unsigned int i=0; i<3; i++)	emot_param[i] = array->data[i];

  return;
}


int main(int argc, char** argv){
  
  // Initialize random seed:
  srand (time(NULL));
  
  double period = 1.0/(double)LOOP_FREQUENCY;
  
  // ROS
  ros::init(argc, argv, "pepper_config_tester");

  ros::NodeHandle node;
  ros::Rate loop_rate(LOOP_FREQUENCY);
  
  //   Topics
  ros::Subscriber pepper_emot_param = node.subscribe("pepper_emot_param", 10, &PepperEmotParamCallback);
  
  ros::Publisher pepper_platform_2Dpose_pub = node.advertise<geometry_msgs::Vector3>("pepper_platform_2Dpose", 1);
  ros::Publisher pepper_joint_pub = node.advertise<sensor_msgs::JointState>("pepper_joint_angles", 1);
  
  ros::Publisher test_value_pub_1 = node.advertise<std_msgs::Float64>("pepper_joint_test_1", 1);
  ros::Publisher test_value_pub_2 = node.advertise<std_msgs::Float64>("pepper_joint_test_2", 1);
  ros::Publisher test_value_pub_3 = node.advertise<std_msgs::Float64>("pepper_joint_test_3", 1);


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

  double current_configuration[PEPPER_TOTAL_N_JOINTS_SIMP];
  //   0:'Pos X', 1:'Pos Y', 2:'Rotation around Z axis'
  //   Joint angles
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		current_configuration[i] = 0.0;

  double pepper_poses[PEPPER_N_CHAINS*7];
  Eigen::Quaternion<double> emot_head_quat, low_head_quat, gaze_quat;
  double low_head_configuration[PEPPER_TOTAL_N_JOINTS_SIMP];
  
  //   KDL
  KDL::Chain* pepper_tree_chains = new KDL::Chain[PEPPER_N_CHAINS];
  initialize_kinematic_chains(pepper_tree_chains);
  
  
  // Time data
  double all_time = GetTimeMs64();
  double alg_time = 0;  
  
  //   ROS
  geometry_msgs::Vector3 rosPepperPlatform2DPose;
  rosPepperPlatform2DPose.x = 0.0;
  rosPepperPlatform2DPose.y = 0.0;
  rosPepperPlatform2DPose.z = 0.0;

  sensor_msgs::JointState rosPepperJoints;
  rosPepperJoints.position.resize(PEPPER_TOTAL_N_JOINTS);
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS; i++)		rosPepperJoints.position[i] = 0.0;

  
  // Files
//   std::ofstream file_exp_1("/home/joseparnau/nullemo_data/emot_conv_1.dat", std::ios::out);
//   std::ofstream file_exp_2("/home/joseparnau/nullemo_data/emot_conv_2.dat", std::ios::out);
//   std::ofstream file_exp_3("/home/joseparnau/nullemo_data/emot_conv_3.dat", std::ios::out);
//   std::ofstream file_exp_4("/home/joseparnau/nullemo_data/emot_conv_4.dat", std::ios::out);

  // Emotion model ---------------------------------------------------------------------------------------

  // H, E, J, G
  double ext = 0.0;  	// Extension
  double EnK = 0.0;	// Kinetic energy
  double JerkA = 0.0;	// Jerk amount
  double symA = 0.0;	// Simmetry
  double Gdir = 0.0;	// Gaze directness
  double dom = 0.0;	// Dominance

  
  // Extent configurations
  double cont_cfg[PEPPER_TOTAL_N_JOINTS_SIMP], ext_cfg[PEPPER_TOTAL_N_JOINTS_SIMP];
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++){
    cont_cfg[i] = 0;
    ext_cfg[i]  = 0;
  }
  double righthand_jnt_val, lefthand_jnt_val;
  righthand_jnt_val = lefthand_jnt_val = 0.5;
  // EXTENT = 0 --------
  // Torso
  cont_cfg[3]  = 0.15;
  cont_cfg[4]  = 0.15;
  // Head
  cont_cfg[7]  = 0.5;
  // Right arm
  cont_cfg[8]  = 0.9;
  cont_cfg[9]  = -0.3;
  cont_cfg[10] = 0.0;
  cont_cfg[11] = 1.1;
  cont_cfg[12] = 1.0;
  // Left arm
  cont_cfg[13] = 0.9;
  cont_cfg[14] = 0.3;
  cont_cfg[15] = 0.0;
  cont_cfg[16] = -1.1;
  cont_cfg[17] = -1.0;
  // EXTENT = 1 ----------
  // Torso
  ext_cfg[3]   = 0.0;
  ext_cfg[4]   = -0.05;
  // Head
  ext_cfg[7]   = -0.1;
  // Right arm
  ext_cfg[8]   = 1.57;
  ext_cfg[9]   = -0.85;
  ext_cfg[10]  = 1.50;
  ext_cfg[11]  = 0.1;
  ext_cfg[12]  = 1.15;
  // Left arm
  ext_cfg[13]  = 1.57;
  ext_cfg[14]  = 0.85;
  ext_cfg[15]  = -1.50;
  ext_cfg[16]  = -0.1;
  ext_cfg[17]  = -1.15;

  
  // Energetic level amplitudes  
  double KAmp[PEPPER_TOTAL_N_JOINTS_SIMP];
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	KAmp[i] = 0.0;
  // Torso
  KAmp[5]   = 0.15;
  // Head
  KAmp[6]   = 0.6;
  // Right arm
  KAmp[9]   = 0.65;
  KAmp[10]  = 0.7;
  KAmp[11]  = 0.3;
  KAmp[12]  = -0.2;
  // Left arm
  KAmp[14]  = -0.65;
  KAmp[15]  = -0.7;
  KAmp[16]  = -0.3;
  KAmp[17]  = 0.2; 
    
  
  // Sym data
  double sym_A[5], sym_w[5], sym_sin[5], sym_sin_past[5];
  for (unsigned int i=0; i<5; i++)	sym_w[i] = sym_A[i] = sym_sin[i] = sym_sin_past[i] = 0.0;
  sym_A[0] = 0.0;
  sym_A[1] = 0.5;
  sym_A[2] = 0.6;
  sym_A[3] = 0.75;
  sym_A[4] = 0.65;
  sym_w[0] = 0.0;
  sym_w[1] = (2*PI)/11.7;
  sym_w[2] = (2*PI)/9.0;
  sym_w[3] = (2*PI)/10.5;
  sym_w[4] = (2*PI)/15.5;
  
  // Test data
  std_msgs::Float64 test_value_1, test_value_2, test_value_3;
  double w = 0.0;  
  
  // Loop -----------------------------------------------------------------------------------------------------------------------
  while (ros::ok()){

//     std::cout << "emot:   " << emot_param[0] << "  " << emot_param[1] << "  " << emot_param[2] << std::endl;
    
    alg_time += (GetTimeMs64() - all_time)/1000.0;
    all_time  =  GetTimeMs64();	// ms


    // Test values
    double ext_period = 2.25;
    w     = (2*PI)/ext_period;
    ext   = 0.5 - 0.5*cos(w*alg_time);
    ext   = 1.0;
    JerkA = 1.0;
    symA  = 0.5;
    dom   = 0.0;
    
    if 	    	( ext   < 0.0 )		ext = 0.0;
    else if 	( ext   > 1.0 )		ext = 1.0;
    if 	    	( JerkA < 0.0 )		JerkA = 0.0;
    else if 	( JerkA > 1.0 )		JerkA = 1.0;
    if 	    	( symA  < 0.0 )		symA = 0.0;
    else if 	( symA  > 1.0 )		symA = 1.0;
    if 	    	( dom   < 0.0 )		dom = 0.0;
    else if 	( dom   > 1.0 )		dom = 1.0;
    
    
    for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)
    {
      double sym_offset = 0.0;
      
      unsigned int i_sym = 0;
      if      ( i == 5 )								i_sym = 1;	// Torso
      else if ( i == 6 )								i_sym = 2;	// Neck
      else if ( (i == 8)  || (i == 9)  || (i == 10) || (i == 11) || (i == 12) )		i_sym = 3;	// Right arm
      else if ( (i == 13) || (i == 14) || (i == 15) || (i == 16) || (i == 17) )		i_sym = 4;	// Left arm
      
      // Symmetry
      sym_sin[i_sym] += period;
      if (  ( sin(sym_w[i_sym] * sym_sin_past[i_sym]) <  0.0)  &&  ( sin(sym_w[i_sym] * sym_sin[i_sym]) >= 0.0)  )
      {
	sym_A[i_sym] = 0.3 + 0.6*rnd0to1();
	sym_w[i_sym] = (2.0*PI)/( 15.0 + 5.0*rnd0to1() );
	sym_sin[i_sym] = 0.0;
      }
      sym_sin_past[i_sym] = sym_sin[i_sym];
      sym_offset = (1.0 - symA) * sym_A[i_sym] * sin( sym_w[i_sym] * sym_sin[i_sym] );      
      
      // Jerkiness
      double jerky_w = (2*PI)/1.0;
      double jerky_offset = JerkA * ( (1.0/5.0) * sin( jerky_w * alg_time + sym_offset) + (1.0/3.0) * cos( 1.5*jerky_w * alg_time  + sym_offset) );
      
      // Excursion
//       KAmp[i] = 0.0;
      
      if ( (i == 5) || (i == 6) )	current_configuration[i] = ( 1.0 - ext ) * cont_cfg[i] + symA * ext * ( ext_cfg[i] + KAmp[i] * sin( w * alg_time + jerky_offset + sym_offset ) );
      else				current_configuration[i] = ( 1.0 - ext ) * cont_cfg[i] + ext * ( ext_cfg[i] + KAmp[i] * sin( w * alg_time + jerky_offset + sym_offset ) );

      if (i == 14)
      {
	test_value_1.data = ( 1.0 - ext ) * cont_cfg[i] + ext * ( ext_cfg[i] + KAmp[i] * sin( w * alg_time ) );
	test_value_2.data = ( 1.0 - ext ) * cont_cfg[i] + ext * ( ext_cfg[i] + KAmp[i] * sin( w * alg_time + jerky_offset ) );
	test_value_3.data = current_configuration[i];
	
// 	file_exp_1 <<  ( 1.0 - ext ) * cont_cfg[i]  +  ext * ( ext_cfg[i] + KAmp[i] * sin( w * alg_time                             ) ) << std::endl;
// 	file_exp_2 <<  ( 1.0 - ext ) * cont_cfg[i]  +  ext * ( ext_cfg[i] + KAmp[i] * sin( w * alg_time + sym_offset                ) ) << std::endl;
// 	file_exp_3 <<  ( 1.0 - ext ) * cont_cfg[i]  +  ext * ( ext_cfg[i] + KAmp[i] * sin( w * alg_time + jerky_offset              ) ) << std::endl;
// 	file_exp_4 <<  ( 1.0 - ext ) * cont_cfg[i]  +  ext * ( ext_cfg[i] + KAmp[i] * sin( w * alg_time + jerky_offset + sym_offset ) ) << std::endl;
      }
      
//       ext = emot_param[0];
//       symA = emot_param[2];
//       current_configuration[i] = ( 1.0 - ext ) * cont_cfg[i] + ext * ( ext_cfg[i] + KAmp[i] * emot_param[1] );
//       if ( (i == 5) || (i == 6) )	current_configuration[i] = ( 1.0 - ext ) * cont_cfg[i] + symA * ext * ( ext_cfg[i] + KAmp[i] * emot_param[1] );
//       else				current_configuration[i] = ( 1.0 - ext ) * cont_cfg[i] + ext * ( ext_cfg[i] + KAmp[i] * emot_param[1] );
    }
    righthand_jnt_val = lefthand_jnt_val = 0.25 + 0.8*ext;
    

    // Gaze
    compute_poses_from_config(pepper_tree_chains, current_configuration, pepper_poses);
    emot_head_quat.x() = pepper_poses[3];
    emot_head_quat.y() = pepper_poses[4];
    emot_head_quat.z() = pepper_poses[5];
    emot_head_quat.w() = pepper_poses[6];
    
    for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	low_head_configuration[i] = current_configuration[i];
    low_head_configuration[6] = 0.0;
    low_head_configuration[7] = cont_cfg[7];
    compute_poses_from_config(pepper_tree_chains, low_head_configuration, pepper_poses);
    low_head_quat.x() = pepper_poses[3];
    low_head_quat.y() = pepper_poses[4];
    low_head_quat.z() = pepper_poses[5];
    low_head_quat.w() = pepper_poses[6];
    
    gaze_quat = low_head_quat.slerp(ext, emot_head_quat).slerp(dom, Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    
    
    saturate_joint_angles(current_configuration);
    
    // Publish -----------------------------------------------------------------------------------------------------------------
      
    //   To ROS format
    pepper3ChainModelConfig_to_RosConfig(current_configuration, pepper_platform_2Dpose, pepper_joints);
    for (unsigned int i=16; i<PEPPER_TOTAL_N_JOINTS; i++)	pepper_joints[i] = 0.0;
    
    
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
    
    
//     std::cout << "t:  " << GetTimeMs64() - all_time << std::endl;
    
    ros::spinOnce(); 
    loop_rate.sleep();

    
    
    // TEST TEST TEST TEST
    test_value_pub_1.publish(test_value_1);
    test_value_pub_2.publish(test_value_2);
    test_value_pub_3.publish(test_value_3);
  }


//   file_exp_1.close();
//   file_exp_2.close();
//   file_exp_3.close();
//   file_exp_4.close();

  
  return 0;
}