// ----------------------------------------------------------------------------------------------------------------------------
//
// Program to generate a random pepper configuration (no platform values and no hands) each time Enter is pressed


#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <visualization_msgs/MarkerArray.h>

#include <pepper_nullemo/geometry_msgs_pose_vector.h>

#include <pepper_core.hpp>
#include <pepper_chain_init.hpp>
#include <pepper_nullemo_aux.hpp>
#include <pepper_trajectories_data.hpp>


#define LOOP_FREQUENCY 100


bool generate_poses_trajectory = false;
unsigned int trajectory_id = 0;
double cycle_time = 0.0;
double trajectory_time = 0.0;
void PepperGeneratePosesTrajectoryCallback(const std_msgs::Int32 msg){
  
  trajectory_id = msg.data;
  
  ROS_INFO("pepper_cartesian_trajectories_generator: Pepper poses trajectory selected. Trajectory Id = %d", trajectory_id);
  
  generate_poses_trajectory = true;
  
  cycle_time = 0.0;
  trajectory_time = 0.0;
  
  return;
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


Eigen::Matrix3d S(Eigen::Vector3d v);


int main(int argc, char** argv){
  
  // Initialize random seed:
  srand (time(NULL));

  // Emotion timer
  double all_time = GetTimeMs64();
  double alg_time = 0;  
  
  double period = 1.0/(double)LOOP_FREQUENCY;  
  
  
  // ROS
  ros::init(argc, argv, "pepper_cartesian_trajectories_generator");

  ros::NodeHandle node;
  ros::Rate loop_rate(LOOP_FREQUENCY);
  
  //   Topics
  ros::Subscriber poses_trajectories_sub = node.subscribe("pepper_generate_poses_trajectory", 10, &PepperGeneratePosesTrajectoryCallback);
  ros::Subscriber pepper_2Dpose_sub      = node.subscribe("pepper_platform_2Dpose", 10, &PepperPlatform2DPoseCallback);
  ros::Subscriber pepper_config_sub      = node.subscribe("pepper_joint_angles", 10, &PepperConfigCallback);
  
  ros::Publisher pepper_pose_trajectory_pub     = node.advertise<pepper_nullemo::geometry_msgs_pose_vector>("pepper_poses_trajectory", 1);
  ros::Publisher dimension_selection_pub        = node.advertise<std_msgs::Int32MultiArray>("pepper_dimension_selection", 100);
  ros::Publisher dimension_selection_frames_pub = node.advertise<std_msgs::Float64MultiArray>("pepper_dimension_selection_frames", 100);
  ros::Publisher gazed_position_pub             = node.advertise<geometry_msgs::Vector3>("pepper_gazed_position", 100);

  //   Markers
  ros::Publisher marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("pepper_object_marker_array", 1);
  
  
  // Variables ----------------------------------------------------------------------------------------------------------------

  double current_configuration[PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS];
  // Platform values set to zero
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)		current_configuration[i] = 0.0;
  
  double pepper_poses[PEPPER_N_CHAINS*7], current_poses[PEPPER_N_CHAINS*7], pose_aux[7];
  // [pos_head, quat_head, pos_rightarm, quat_rightarm, pos_leftarm, quat_leftarm] = [ {px, py, pz, qx, qy, qz, qw} ]
  for (unsigned int i=0; i<PEPPER_N_CHAINS*7; i++)		current_poses[i] = pepper_poses[i] = 0.0;
  current_poses[6]  = pepper_poses[6]  = 1.0;
  current_poses[13] = pepper_poses[13] = 1.0;
  current_poses[20] = pepper_poses[20] = 1.0;  
  
 
  //   ROS
  geometry_msgs::Pose ROSPoseAux;
  geometry_msgs::Vector3 gazedPosition;
  gazedPosition.x = gazedPosition.y = gazedPosition.z = -1.0;	// if all three values are -1 then there is no gazed object and the head orientation is computed from the desired_poses[3:6]
    
  load_pepper_joint_values();
  //   KDL
  KDL::Chain* pepper_tree_chains = new KDL::Chain[PEPPER_N_CHAINS];
  initialize_kinematic_chains(pepper_tree_chains);
  
  KDL::ChainFkSolverPos_recursive** fk_solvers = new KDL::ChainFkSolverPos_recursive*[PEPPER_N_CHAINS];
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	  fk_solvers[i] = new KDL::ChainFkSolverPos_recursive(pepper_tree_chains[i]);  
 
  //     Dimension selection
  Eigen::VectorXd vSel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6);
  std_msgs::Int32MultiArray ROSdimensionSelection;
  ROSdimensionSelection.data.resize(PEPPER_N_TCP*6);
  for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)		ROSdimensionSelection.data[i] = 0.0;
  
  Eigen::VectorXd sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
  sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
  std_msgs::Float64MultiArray ROSdimensionSelectionFramesQuat;
  ROSdimensionSelectionFramesQuat.data.resize(PEPPER_N_TCP*8);
  for (unsigned int i=0; i<PEPPER_N_TCP; i++)
  {
    // Position
    ROSdimensionSelectionFramesQuat.data[8*i+0] = ROSdimensionSelectionFramesQuat.data[8*i+1] = ROSdimensionSelectionFramesQuat.data[8*i+2] = 0.0;
    ROSdimensionSelectionFramesQuat.data[8*i+3] = 1.0;
    // Orientation
    ROSdimensionSelectionFramesQuat.data[8*i+4] = ROSdimensionSelectionFramesQuat.data[8*i+5] = ROSdimensionSelectionFramesQuat.data[8*i+6] = 0.0;
    ROSdimensionSelectionFramesQuat.data[8*i+7] = 1.0;
  }
  
  
  // Loop -----------------------------------------------------------------------------------------------------------------------
  time_increment = 0.0;
  unsigned int n_it = 0;
  while ( ros::ok() ){
    
    if (generate_poses_trajectory)
    {
      n_it++;

      
      update_time_increment();
      if (n_it == 1)	time_increment = 0.0;      
      cycle_time += time_increment;
      trajectory_time += time_increment;
      
      // Default gaze is sent through the head orientation
      gazedPosition.x = gazedPosition.y = gazedPosition.z = -1.0;	// if all three values are -1 then there is no gazed object and the head orientation is computed from the desired_poses[3:6]
      
      // From ROS format
//       if (n_it < 10)	set_home_configuration(current_configuration);
//       else	
      pepperRosConfig_to_3ChainModelConfig(pepper_platform_2Dpose, pepper_joints, current_configuration);
      
      // Obtain current poses
      compute_poses_from_config(fk_solvers, current_configuration, current_poses);

      
      // Markers setup
      unsigned int marker_id = 0;
      Eigen::Matrix3d auxMat;
      Eigen::Quaternion<double> auxQuat;
      
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::ADD;
      marker.header.stamp = ros::Time::now();
      marker.ns = "pepper_objects";
      marker.lifetime = ros::Duration();      
      
      
      // Generate random 2D platform pose inside a squared area
      if (trajectory_id == RANDOM_POSES)			// ***********************************************************************************************************************************************
      {
	
	double random_configuration[PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS];
	// Platform values set to zero
	for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)		random_configuration[i] = 0.0;
	
	random_configuration[0] = -2.0 + 10.0*((double)rand() / (double)(RAND_MAX));
	random_configuration[1] = -2.0 + 10.0*((double)rand() / (double)(RAND_MAX));
	random_configuration[2] =         2.0*PI*((double)rand() / (double)(RAND_MAX));
	generate_pepper_random_pose(fk_solvers, random_configuration, pepper_poses);
	
	generate_poses_trajectory = false;
	
	// Dimension selector
	vSel = Eigen::VectorXd::Zero(18);
	vSel.setOnes();
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
      }
      
      // Generate a simple circular trajectory in the YZ plane for the arms and the head
      else if (trajectory_id == HEADARM_POSES_CIRCLES)		// ***********************************************************************************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	double ang_vel = PI/2;	// rad/s
	double radius = 0.1;	// m
	
	double basePos[3];
	basePos[0] = 1.0;
	basePos[1] = 0.0 + radius*cos(ang_vel * trajectory_time);
	basePos[2] = 0.75 + radius*sin(ang_vel * trajectory_time);
	
	double headPosInc[3], rightarmPosInc[3], leftPosInc[3];
	headPosInc[0] = -0.15;
	headPosInc[1] = 0.0;
	headPosInc[2] = 0.20;
	rightarmPosInc[0] = 0.0;
	rightarmPosInc[1] = -0.2;
	rightarmPosInc[2] = 0.0;
	leftPosInc[0] = 0.0;
	leftPosInc[1] = 0.2;
	leftPosInc[2] = 0.0;
	
	
	// Head
	//   Position
	pepper_poses[0] = current_poses[0];
	pepper_poses[1] = current_poses[1];
	pepper_poses[2] = current_poses[2];
	//   Orientation
	pepper_poses[3] = current_poses[3];
	pepper_poses[4] = current_poses[4];
	pepper_poses[5] = current_poses[5];
	pepper_poses[6] = current_poses[6];
	
	// Right arm
	//   Position
	pepper_poses[7] = basePos[0] + rightarmPosInc[0];
	pepper_poses[8] = basePos[1] + rightarmPosInc[1];
	pepper_poses[9] = basePos[2] + rightarmPosInc[2];
	//   Orientation
	pepper_poses[10] = 0.0;
	pepper_poses[11] = 0.0;
	pepper_poses[12] = 0.0;
	pepper_poses[13] = 1.0;
	
	// Left arm
	//   Position
	pepper_poses[14] = basePos[0] + leftPosInc[0];
	pepper_poses[15] = basePos[1] + leftPosInc[1];
	pepper_poses[16] = basePos[2] + leftPosInc[2];
	//   Orientation
	pepper_poses[17] = 0.0;
	pepper_poses[18] = 0.0;
	pepper_poses[19] = 0.0;
	pepper_poses[20] = 1.0;
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	//     Right arm - Position
	vSel(6) = vSel(7) = vSel(8) = 1.0;
	//     Left arm - Position
	vSel(12) = vSel(13) = vSel(14) = 1.0;
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;	
	
// 	std::cout << "In circles!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!111" << std::endl;
      }
      
      // Generate a simple circular trajectory in the YZ plane for the arms and the head
      else if (trajectory_id == RIGHTARM_WAVING)		// ***********************************************************************************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	// Right arm
	double ang_vel = 150.0*(PI/180.0);	// rad/s
	double wRad = 0.25;	// m
	double ang_max = 40.0*(PI/180);	// rad
	double rot_ang = 25.0*(PI/180.0) + ang_max * sin(ang_vel * trajectory_time);
	
	Eigen::MatrixXd waveP(4,4);
	waveP.col(0) << 0.0, -sin(rot_ang), cos(rot_ang), 0.0;
	waveP.col(1) << 0.0, cos(rot_ang), sin(rot_ang), 0.0;
	waveP.col(2) << -1.0, 0.0, 0.0, 0.0;
	waveP.col(3) << 0.0, -0.15 - wRad*sin(rot_ang), 0.9 + wRad*cos(rot_ang), 1.0;
	
	// Head
	//   Position
	pepper_poses[0] = current_poses[0];	// Visualization
	pepper_poses[1] = current_poses[1];	// Visualization
	pepper_poses[2] = current_poses[2];	// Visualization
	//   Orientation
	pepper_poses[3] = 0.0;
	pepper_poses[4] = 0.0;
	pepper_poses[5] = 0.0;
	pepper_poses[6] = 1.0;
	
	// Right arm
	//   Position Y,Z
	pepper_poses[7] = current_poses[7];	// Visualization
	pepper_poses[8] = waveP(1,3);
	pepper_poses[9] = waveP(2,3);
	//   Orientation
	pepper_poses[10] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).x();
	pepper_poses[11] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).y();
	pepper_poses[12] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).z();
	pepper_poses[13] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).w();
	
	// Left arm	// Visualization
	//   Position
	pepper_poses[14] = current_poses[14];
	pepper_poses[15] = current_poses[15];
	pepper_poses[16] = current_poses[16];
	//   Orientation
	pepper_poses[17] = current_poses[17];
	pepper_poses[18] = current_poses[18];
	pepper_poses[19] = current_poses[19];
	pepper_poses[20] = current_poses[20];
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	//     Head orientations w_y w_z
	vSel(4) = vSel(5) = 1.0;
	//     Right arm - Position Y Z
	vSel(7) = vSel(8) = 1.0;
	//     Right arm - Orientation
	vSel(9) = vSel(10) = vSel(11) = 1.0;
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
      }
      
      // Generate a simple circular trajectory in the YZ plane for the arms and the head less constrained than RIGHTARM_WAVING in the head orientation
      else if (trajectory_id == RIGHTARM_WAVING_RELAXED)		// ************************************************************************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	// Right arm
	double ang_vel = 150.0*(PI/180.0);	// rad/s
	double wRad = 0.25;	// m
	double ang_max = 40.0*(PI/180);	// rad
	double rot_ang = 25.0*(PI/180.0) + ang_max * sin(ang_vel * trajectory_time);
	
	Eigen::MatrixXd waveP(4,4);
	waveP.col(0) << 0.0, -sin(rot_ang), cos(rot_ang), 0.0;
	waveP.col(1) << 0.0, cos(rot_ang), sin(rot_ang), 0.0;
	waveP.col(2) << -1.0, 0.0, 0.0, 0.0;
	waveP.col(3) << 0.0, -0.15 - wRad*sin(rot_ang), 0.9 + wRad*cos(rot_ang), 1.0;
	
	// Head
	//   Position
	pepper_poses[0] = current_poses[0];	// Visualization
	pepper_poses[1] = current_poses[1];	// Visualization
	pepper_poses[2] = current_poses[2];	// Visualization
	//   Orientation
	pepper_poses[3] = 0.0;
	pepper_poses[4] = 0.0;
	pepper_poses[5] = 0.0;
	pepper_poses[6] = 1.0;
	
	// Right arm
	//   Position Y,Z
	pepper_poses[7] = current_poses[7];	// Visualization
	pepper_poses[8] = waveP(1,3);
	pepper_poses[9] = waveP(2,3);
	//   Orientation
	pepper_poses[10] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).x();
	pepper_poses[11] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).y();
	pepper_poses[12] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).z();
	pepper_poses[13] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).w();
	
	// Left arm	// Visualization
	//   Position
	pepper_poses[14] = current_poses[14];
	pepper_poses[15] = current_poses[15];
	pepper_poses[16] = current_poses[16];
	//   Orientation
	pepper_poses[17] = current_poses[17];
	pepper_poses[18] = current_poses[18];
	pepper_poses[19] = current_poses[19];
	pepper_poses[20] = current_poses[20];
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	//     Head orientations w_z
	vSel(5) = 1.0;
	//     Right arm - Position Y Z
	vSel(7) = vSel(8) = 1.0;
	//     Right arm - Orientation
	vSel(9) = vSel(10) = vSel(11) = 1.0;
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
      }
      
      // Generate a simple circular trajectory in the YZ plane for the arms and the head less constrained than RIGHTARM_WAVING in the head orientation and heading towards point (10,0,0)
      else if (trajectory_id == RIGHTARM_WAVING_RELAXED_LOOK)		// ************************************************************************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	// Right arm
	double ang_vel = 150.0*(PI/180.0);	// rad/s
	double wRad = 0.25;	// m
	double ang_max = 40.0*(PI/180);	// rad
	double rot_ang = 25.0*(PI/180.0) + ang_max * sin(ang_vel * trajectory_time);
	
	double head_point[3];	// Point to look for
	head_point[0] = 5.0;
	head_point[1] = 0.0;
	head_point[2] = 1.2;
	
	Eigen::Matrix3d headRotMat;
	headRotMat.col(0) << Eigen::Vector3d(head_point[0] - current_poses[0], head_point[1] - current_poses[1], head_point[2] - current_poses[2]).normalized();
	headRotMat.col(1) << Eigen::Vector3d::UnitZ().cross(headRotMat.col(0));
	headRotMat.col(2) =  headRotMat.col(0).cross(headRotMat.col(1));
	
// 	std::cout << "headRotMat ------------" << std::endl;
// 	std::cout << headRotMat << std::endl;
	
	Eigen::MatrixXd waveP(4,4);
	waveP.col(0) << 0.0, -sin(rot_ang), cos(rot_ang), 0.0;
	waveP.col(1) << 0.0, cos(rot_ang), sin(rot_ang), 0.0;
	waveP.col(2) << -1.0, 0.0, 0.0, 0.0;
	waveP.col(3) << 0.0, -0.15 - wRad*sin(rot_ang), 0.9 + wRad*cos(rot_ang), 1.0;
	
	// Head
	//   Position
	pepper_poses[0] = current_poses[0];	// Visualization
	pepper_poses[1] = current_poses[1];	// Visualization
	pepper_poses[2] = current_poses[2];	// Visualization
	//   Orientation
	pepper_poses[3] = Eigen::Quaternion<double>(headRotMat).x();
	pepper_poses[4] = Eigen::Quaternion<double>(headRotMat).y();
	pepper_poses[5] = Eigen::Quaternion<double>(headRotMat).z();
	pepper_poses[6] = Eigen::Quaternion<double>(headRotMat).w();
	
	// Right arm
	//   Position Y,Z
	pepper_poses[7] = current_poses[7];	// Visualization
	pepper_poses[8] = waveP(1,3);
	pepper_poses[9] = waveP(2,3);
	//   Orientation
	pepper_poses[10] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).x();
	pepper_poses[11] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).y();
	pepper_poses[12] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).z();
	pepper_poses[13] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).w();
	
	// Left arm	// Visualization
	//   Position
	pepper_poses[14] = current_poses[14];
	pepper_poses[15] = current_poses[15];
	pepper_poses[16] = current_poses[16];
	//   Orientation
	pepper_poses[17] = current_poses[17];
	pepper_poses[18] = current_poses[18];
	pepper_poses[19] = current_poses[19];
	pepper_poses[20] = current_poses[20];
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	//     Head orientations w_x
	vSel(3) = 1.0;
	//     Right arm - Position Y Z
	vSel(7) = vSel(8) = 1.0;
	//     Right arm - Orientation
	vSel(9) = vSel(10) = vSel(11) = 1.0;
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
	//     Head orientation
	sel_frames_quat(4) = Eigen::Quaternion<double>(headRotMat).x();
	sel_frames_quat(5) = Eigen::Quaternion<double>(headRotMat).y();
	sel_frames_quat(6) = Eigen::Quaternion<double>(headRotMat).z();
	sel_frames_quat(7) = Eigen::Quaternion<double>(headRotMat).w();
      } 
      
      // Generate a simple circular trajectory in the YZ plane for the arms and the head less constrained than RIGHTARM_WAVING in the head orientation and heading towards point (10,0,0)
      else if ( (trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS) || (trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS_XY) )	// ******************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	double pointFollow[3], pointVel[3];
	for (unsigned int i=0; i<3; i++)	pointFollow[i] = 0.0;
	for (unsigned int i=0; i<3; i++)	pointVel[i] = 0.0;
	
	double t1 = 6.0;
	double t2 = 0.0;
	double t3 = 6.0;
	double t4 = 0.0;
	
	double dy = 0.0;
	double vl;
	if (t2 != 0.0)		vl = PI/t2;
	else			vl = 0.65;
	
	if (cycle_time > t1 + t2 + t3 + t4)		cycle_time -= t1 + t2 + t3 + t4;

	
	if ( trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS )			pointFollow[2] = 0.7;
	else if ( trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS_XY )		pointFollow[2] = current_poses[9];
	else 									pointFollow[2] = 0.7;
	
	
	if (cycle_time < t1)
	{
	  pointFollow[0] = vl*cycle_time;
	}
	else if (cycle_time < t1 + t2)
	{
	  pointFollow[0] = vl*t1 + sin( (PI/t2)*(cycle_time-t1) );
	  if ( fabs(t2) > 0.000001 )		pointFollow[1] = (dy/2)*(  1.0 - cos( (PI/t2)*(cycle_time-t1) )  );
	  else					pointFollow[1] = 0.0;
	}
	else if (cycle_time < t1 + t2 + t3)
	{
	  pointFollow[0] = vl*t3 - vl*(cycle_time-t2-t1);
	  pointFollow[1] = dy;
	}
	else
	{
	  pointFollow[0] = -sin( (PI/t4)*(cycle_time-t1-t2-t3) );
	  if ( fabs(t4) > 0.000001 )		pointFollow[1] = (dy/2)*(  1.0 + cos( (PI/t4)*(cycle_time-t1-t2-t3) )  );
	  else					pointFollow[1] = 0.0;
	}
	
	Eigen::Quaternion<double> currHeadQuat;
	currHeadQuat.x() = current_poses[3];
	currHeadQuat.y() = current_poses[4];
	currHeadQuat.z() = current_poses[5];
	currHeadQuat.w() = current_poses[6];
	Eigen::Matrix3d currHeadRotMat;
	currHeadRotMat = currHeadQuat.toRotationMatrix();
	
	double cY2dY = acos( currHeadRotMat.col(1).dot( Eigen::Vector3d::UnitY() ) );
	Eigen::Vector3d rotAxis = currHeadRotMat.col(1).cross( Eigen::Vector3d::UnitY() ).normalized();
	if ( fabs(cY2dY) < 0.00000001 )		rotAxis = Eigen::Vector3d::UnitX();	
	Eigen::Matrix3d headRotMat2AY = Eigen::AngleAxis<double>( cY2dY, rotAxis ).toRotationMatrix();
	Eigen::Matrix3d headRotMat = headRotMat2AY * currHeadRotMat;

	
	// Head
	//   Position
	pepper_poses[0] = current_poses[0];	// Visualization
	pepper_poses[1] = current_poses[1];	// Visualization
	pepper_poses[2] = current_poses[2];	// Visualization
	//   Orientation
	pepper_poses[3] = Eigen::Quaternion<double>(headRotMat).x();
	pepper_poses[4] = Eigen::Quaternion<double>(headRotMat).y();
	pepper_poses[5] = Eigen::Quaternion<double>(headRotMat).z();
	pepper_poses[6] = Eigen::Quaternion<double>(headRotMat).w();

	// Right arm
	//   Position Y,Z
	pepper_poses[7] = pointFollow[0];
	pepper_poses[8] = pointFollow[1];
	pepper_poses[9] = pointFollow[2];
	//   Orientation
	pepper_poses[10] = current_poses[10];	// Visualization
	pepper_poses[11] = current_poses[11];	// Visualization
	pepper_poses[12] = current_poses[12];	// Visualization
	pepper_poses[13] = current_poses[13];	// Visualization
	
	// Left arm	// Visualization
	//   Position
	pepper_poses[14] = current_poses[14];
	pepper_poses[15] = current_poses[15];
	pepper_poses[16] = current_poses[16];
	//   Orientation
	pepper_poses[17] = current_poses[17];
	pepper_poses[18] = current_poses[18];
	pepper_poses[19] = current_poses[19];
	pepper_poses[20] = current_poses[20];
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	if ( trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS )
	{
	  //     Head orientations w_z
	  vSel(5) = 1.0;
	  //     Right arm - Position X Y Z
	  vSel(6) = vSel(7) = vSel(8) = 1.0;
	}
	else if ( trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS_XY )
	{
	  //     Head orientations w_z
	  vSel(5) = 1.0;
	  //     Right arm - Position X Y Z
	  vSel(6) = vSel(7) = 1.0;
	}
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
      }
      
      else if ( ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR ) || ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY ) )	// ****************************************************************************
      {	
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	double pointFollow[3], pointVel[3];
	for (unsigned int i=0; i<3; i++)	pointFollow[i] = 0.0;
	for (unsigned int i=0; i<3; i++)	pointVel[i] = 0.0;
	
	double t1 = 6.0;
	double t2 = 0.0;
	double t3 = 6.0;
	double t4 = 0.0;
	
	double dy = 0.0;
	double vl;
	if (t2 != 0.0)		vl = PI/t2;
	else			vl = 0.65;
	
	if (cycle_time > t1 + t2 + t3 + t4)		cycle_time -= t1 + t2 + t3 + t4;

	
	if ( ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR ) || ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY ) )		pointFollow[2] = current_poses[17];
	else 																	pointFollow[2] = 0.7;
	
	
	if (cycle_time <= t1)
	{
	  pointFollow[0] = vl*cycle_time;
	}
	else if (cycle_time <= t1 + t2)
	{
	  if ( fabs(t2) > 0.000001 ){
	    pointFollow[0] = vl*t1 + sin( (PI/t2)*(cycle_time-t1) );
	    pointFollow[1] = (dy/2)*(  1.0 - cos( (PI/t2)*(cycle_time-t1) )  );
	  }
	  else{
	    pointFollow[0] = vl*t1;
	    pointFollow[1] = 0.0;
	  }
	}
	else if (cycle_time <= t1 + t2 + t3)
	{
	  pointFollow[0] = vl*t3 - vl*(cycle_time-t2-t1);
	  pointFollow[1] = dy;
	}
	else
	{
	  if ( fabs(t4) > 0.000001 )
	  {
	    pointFollow[0] = -sin( (PI/t4)*(cycle_time-t1-t2-t3) );
	    pointFollow[1] = (dy/2)*(  1.0 + cos( (PI/t4)*(cycle_time-t1-t2-t3) )  );
	  }
	  else
	  {
	    pointFollow[0] = 0.0;
	    pointFollow[1] = 0.0;
	  }
	}
	
	Eigen::Quaternion<double> currHeadQuat;
	currHeadQuat.x() = current_poses[3];
	currHeadQuat.y() = current_poses[4];
	currHeadQuat.z() = current_poses[5];
	currHeadQuat.w() = current_poses[6];
	Eigen::Matrix3d currHeadRotMat;
	currHeadRotMat = currHeadQuat.toRotationMatrix();
	
	double cY2dY = acos( currHeadRotMat.col(1).dot( Eigen::Vector3d::UnitY() ) );
	Eigen::Vector3d rotAxis = currHeadRotMat.col(1).cross( Eigen::Vector3d::UnitY() ).normalized();
	if ( fabs(cY2dY) < 0.00000001 )		rotAxis = Eigen::Vector3d::UnitX();	
	Eigen::Matrix3d headRotMat2AY = Eigen::AngleAxis<double>( cY2dY, rotAxis ).toRotationMatrix();
	Eigen::Matrix3d headRotMat = headRotMat2AY * currHeadRotMat;

	Eigen::Matrix3d leftarmRotMat;
	leftarmRotMat.col(2) << 0.0, 0.0, -1.0;
	leftarmRotMat.col(0) = Eigen::Vector3d::UnitX();
	leftarmRotMat.col(1) = leftarmRotMat.col(2).cross(leftarmRotMat.col(0));
	
	
	// Head
	//   Position
	pepper_poses[0] = current_poses[0];	// Visualization
	pepper_poses[1] = current_poses[1];	// Visualization
	pepper_poses[2] = current_poses[2];	// Visualization
	//   Orientation
	pepper_poses[3] = Eigen::Quaternion<double>(headRotMat).x();
	pepper_poses[4] = Eigen::Quaternion<double>(headRotMat).y();
	pepper_poses[5] = Eigen::Quaternion<double>(headRotMat).z();
	pepper_poses[6] = Eigen::Quaternion<double>(headRotMat).w();

	// Right arm	// Visualization
	//   Position
	pepper_poses[7] = current_poses[7];
	pepper_poses[8] = current_poses[8];
	pepper_poses[9] = current_poses[9];
	//   Orientation
	pepper_poses[10] = current_poses[10];	// Visualization
	pepper_poses[11] = current_poses[11];	// Visualization
	pepper_poses[12] = current_poses[12];	// Visualization
	pepper_poses[13] = current_poses[13];	// Visualization
	
	// Left arm
	//   Position, X,Y
	pepper_poses[14] = pointFollow[0];
	pepper_poses[15] = pointFollow[1];
	pepper_poses[16] = current_poses[16];
	//   Orientation w_x, w_y
	pepper_poses[17] = Eigen::Quaternion<double>(leftarmRotMat).x();
	pepper_poses[18] = Eigen::Quaternion<double>(leftarmRotMat).y();
	pepper_poses[19] = Eigen::Quaternion<double>(leftarmRotMat).z();
	pepper_poses[20] = Eigen::Quaternion<double>(leftarmRotMat).w();


	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	if ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR )
	{
	  //     Head orientations w_z
	  vSel(5) = 1.0;
	  //     Left arm - Position X Y
	  vSel(12) = vSel(13) = 1.0;
	  //     Left arm - Orientation
	  vSel(15) = vSel(16) = vSel(17) = 1.0;
	}
	else if ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY )
	{
	  //     Head orientations w_z
	  vSel(5) = 1.0;
	  //     Left arm - Position X Y
	  vSel(12) = vSel(13) = 1.0;
	  //     Left arm - Orientation w_x w_y
	  vSel(15) = vSel(16) = 1.0;
	}
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;

	
// 	std::cout << "------------------------------------------------------------------------------------------" << std::endl;
// 	std::cout << "cycle_time:  " << cycle_time << std::endl;
// 	std::cout << "current_configuration:  ";
// 	std::cout << "  " << current_configuration[0];
// 	std::cout << "  " << current_configuration[1];
// 	for (unsigned int i=2; i<18; i++)	std::cout << "  " << current_configuration[i]*(180.0/PI);
// 	std::cout << std::endl;
// 	std::cout << "current_poses:  ";
// 	for (unsigned int i=0; i<21; i++)	std::cout << "  " << current_poses[i];
// 	std::cout << std::endl;
// 	std::cout << "pepper_poses:  ";
// 	for (unsigned int i=0; i<21; i++)	std::cout << "  " << pepper_poses[i];
// 	std::cout << std::endl;
// 	std::cout << "   quat norms:  " 
// 		  << sqrt( pow(pepper_poses[3],2)+pow(pepper_poses[4],2)+pow(pepper_poses[5],2)+pow(pepper_poses[6],2) ) << "  "
// 		  << sqrt( pow(pepper_poses[10],2)+pow(pepper_poses[11],2)+pow(pepper_poses[12],2)+pow(pepper_poses[13],2) ) << "  "
// 		  << sqrt( pow(pepper_poses[17],2)+pow(pepper_poses[18],2)+pow(pepper_poses[19],2)+pow(pepper_poses[20],2) ) << "  "
// 		  << std::endl;
      }
      
      // Generate a simple circular trajectory in the YZ plane for the arms and the head less constrained than RIGHTARM_WAVING in the head orientation
      else if ( (trajectory_id == RIGHTHAND_SHAKE) || 
		(trajectory_id == RIGHTHAND_SHAKE_POS_Z) || 
		(trajectory_id == RIGHTHAND_SHAKE_HEAD_Y)  || 
		(trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_RIGHTARM_YZ) ||
		(trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR) ||
		(trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ) ||
		(trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR) ||
		(trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WX_WY_RIGHTARM_YZ_OR) ||
	        (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ_HEAD_WY_WZ) ) // ***********************************************************************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	// Right arm
	double shakeHeight = 0.20;
	double shake_vel = 3.0;

	Eigen::MatrixXd waveP(4,4);
	waveP = Eigen::MatrixXd::Identity(4,4);
	
	double d2 = sqrt(2.0)/2.0;
	waveP.col(0) << d2, d2, 0.0, 0.0;
	waveP.col(1) << 0.0, 0.0, 1.0, 0.0;
	waveP.col(2) << d2, -d2, 0.0, 0.0;
	
	Eigen::Matrix3d RotMat = Eigen::AngleAxis<double>(-PI/4.0, Eigen::Vector3d(d2, -d2, 0.0).normalized()).toRotationMatrix();;	
	waveP.block<3,3>(0,0) = RotMat * waveP.block<3,3>(0,0);
	if ( (trajectory_id == RIGHTHAND_SHAKE) || 
	     (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y) )				waveP.col(3) << 0.0, 0.0, 0.7 + (shakeHeight/2.0)*sin(shake_vel * trajectory_time), 1.0;
	else if (trajectory_id == RIGHTHAND_SHAKE_POS_Z)				waveP.col(3) << current_poses[7], current_poses[8], 0.7 + (shakeHeight/2.0)*sin(shake_vel * trajectory_time), 1.0;
	else if ( (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_RIGHTARM_YZ) ||
	          (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ) ||
	          (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR) ||
	          (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WX_WY_RIGHTARM_YZ_OR) || 
	          (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ_HEAD_WY_WZ) )		waveP.col(3) << current_poses[7], 0.0, 0.7 + (shakeHeight/2.0)*sin(shake_vel * trajectory_time), 1.0;	// Right arm pos Y,Z
	
	Eigen::Vector3d headPos;
	if ( (trajectory_id == RIGHTHAND_SHAKE) || 
	     (trajectory_id == RIGHTHAND_SHAKE_POS_Z) || 
	     (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ)  || 
	     (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ_HEAD_WY_WZ))			headPos << current_poses[0], current_poses[1], current_poses[2];	// Visualization
	else if ( (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y) ||
	          (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_RIGHTARM_YZ) ||
	          (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR) ||
	          (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WX_WY_RIGHTARM_YZ_OR) )	headPos << current_poses[0], 0.0, current_poses[2];			// Constrain pos Y
	
	Eigen::Matrix3d headRot;
	headRot = Eigen::Matrix3d::Identity(3,3);
	if ( (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR) ||
	     (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WX_WY_RIGHTARM_YZ_OR) )
	{
	  headRot.col(0) << 1.0, 0.0, 0.0;
	  headRot.col(1) << 0.0, 1.0, 0.0;
	  headRot.col(2) << 0.0, 0.0, 1.0;
	}
	
	
	// Head
	//   Position
	pepper_poses[0] = headPos(0);
	pepper_poses[1] = headPos(1);
	pepper_poses[2] = headPos(2);
	//   Orientation
	pepper_poses[3] = Eigen::Quaternion<double>(headRot).x();
	pepper_poses[4] = Eigen::Quaternion<double>(headRot).y();
	pepper_poses[5] = Eigen::Quaternion<double>(headRot).z();
	pepper_poses[6] = Eigen::Quaternion<double>(headRot).w();
	
	// Right arm
	//   Position Y,Z
	pepper_poses[7] = waveP(0,3);
	pepper_poses[8] = waveP(1,3);
	pepper_poses[9] = waveP(2,3);
	//   Orientation
	pepper_poses[10] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).x();
	pepper_poses[11] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).y();
	pepper_poses[12] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).z();
	pepper_poses[13] = Eigen::Quaternion<double>(waveP.block<3,3>(0,0)).w();
	
	// Left arm	// Visualization
	//   Position
	pepper_poses[14] = current_poses[14];
	pepper_poses[15] = current_poses[15];
	pepper_poses[16] = current_poses[16];
	//   Orientation
	pepper_poses[17] = current_poses[17];
	pepper_poses[18] = current_poses[18];
	pepper_poses[19] = current_poses[19];
	pepper_poses[20] = current_poses[20];
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	if (trajectory_id == RIGHTHAND_SHAKE)
	{
	  //     Head orientations w_z
	  vSel(3) = vSel(4) = vSel(5) = 1.0;
	  //     Right arm - Position
	  vSel(6) = vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	}	  
	else if (trajectory_id == RIGHTHAND_SHAKE_POS_Z)
	{
	  //     Head - Orientation
	  vSel(3) = vSel(4) = vSel(5) = 1.0;
	  //     Right arm - Position
	  vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	}
	else if (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y)
	{
	  //     Head - Position Y
	  vSel(1) = 1.0;
	  //     Head - Orientation
	  vSel(3) = vSel(4) = vSel(5) = 1.0;
	  //     Right arm - Position
	  vSel(6) = vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	}
	else if (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_RIGHTARM_YZ)
	{
	  //     Head - Position Y
	  vSel(1) = 1.0;
	  //     Head - Orientation
	  vSel(3) = vSel(4) = vSel(5) = 1.0;
	  //     Right arm - Position
	  vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	}
	else if (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WZ_RIGHTARM_YZ_OR)
	{
	  //     Head - Position Y
	  vSel(1) = 1.0;
	  //     Head - Orientation w_z
	  vSel(5) = 1.0;
	  //     Right arm - Position Y Z
	  vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	}
	else if (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ)
	{
	  //     Head orientations w_z
	  vSel(3) = vSel(4) = vSel(5) = 1.0;
	  //     Right arm - Position Y Z
	  vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	}
	else if (trajectory_id == RIGHTHAND_SHAKE_HEAD_Y_WX_WY_RIGHTARM_YZ_OR)
	{
	  //     Head - Position Y
	  vSel(1) = 1.0;
	  //     Head - Orientation w_x, w_y
	  vSel(3) = vSel(4) = 1.0;
	  //     Right arm - Position Y Z
	  vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	}
	else if (trajectory_id == RIGHTHAND_SHAKE_RIGHTARM_YZ_HEAD_WY_WZ)
	{
	  //     Head - Orientation w_y, w_z
	  vSel(4) = vSel(5) = 1.0;
	  //     Right arm - Position Y Z
	  vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0; 
	}
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
      }
      
      else if ( ( trajectory_id == OBJECT_BOTH_HANDS ) ||
		( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD ) ) // ***********************************************************************************************************************
      {	
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	double tl = 6.0;
	double vl = 0.65;
	
	if ( cycle_time > 2*tl )		cycle_time -= 2*tl;

	// Object
	double objectPos[3];
	for (unsigned int i=0; i<3; i++)	objectPos[i] = 0.0;
	if (cycle_time <= tl)		objectPos[0] = vl*cycle_time;
	else if (cycle_time <= 2*tl)	objectPos[0] = vl*tl - vl*(cycle_time-tl);
	objectPos[1] = 0.0;
	objectPos[2] = 0.7;
// 	if ( ( trajectory_id == OBJECT_BOTH_HANDS ) )		objectPos[2] = 0.7;
// 	else							objectPos[2] = current_poses[17];
	
	Eigen::Quaternion<double> objectQuat;
	objectQuat.x() = 0.0;
	objectQuat.y() = 0.0;
	objectQuat.z() = 0.0;
	objectQuat.w() = 1.0;
	
	Eigen::MatrixXd objectMat(4,4);
	objectMat = Eigen::MatrixXd::Identity(4,4);
	objectMat.block<3,3>(0,0) = objectQuat.toRotationMatrix();
	objectMat.col(3) << objectPos[0], objectPos[1], objectPos[2], 1.0;
	
	//   Grasp points
	Eigen::MatrixXd rightGraspTF(4,4), leftGraspTF(4,4);
	double dx = -0.1;
	double dy = 0.17;
	double dz = 0.0;
	
	rightGraspTF = Eigen::MatrixXd::Identity(4,4);
	rightGraspTF.col(0) << 1.0, 0.0, 0.0, 0.0;
	rightGraspTF.col(1) << 0.0, 0.0, 1.0, 0.0;
	rightGraspTF.col(2) << 0.0, -1.0, 0.0, 0.0;
	rightGraspTF.col(3) << dx, -dy, dz, 1.0;
	
	leftGraspTF = Eigen::MatrixXd::Identity(4,4);
	leftGraspTF.col(0) << 1.0, 0.0, 0.0, 0.0;
	leftGraspTF.col(1) << 0.0, 0.0, -1.0, 0.0;
	leftGraspTF.col(2) << 0.0, 1.0, 0.0, 0.0;
	leftGraspTF.col(3) << dx, dy, dz, 1.0;
	
	// Hands
	Eigen::MatrixXd righthandPose(4,4), lefthandPose(4,4);
	righthandPose = objectMat * rightGraspTF;
	lefthandPose = objectMat * leftGraspTF;
	
	// Head
	Eigen::Quaternion<double> currHeadQuat;
	currHeadQuat.x() = current_poses[3];
	currHeadQuat.y() = current_poses[4];
	currHeadQuat.z() = current_poses[5];
	currHeadQuat.w() = current_poses[6];
	Eigen::Matrix3d currHeadRotMat;
	currHeadRotMat = currHeadQuat.toRotationMatrix();
	
	double cY2dY = acos( currHeadRotMat.col(1).dot( Eigen::Vector3d::UnitY() ) );
	Eigen::Vector3d rotAxis = currHeadRotMat.col(1).cross( Eigen::Vector3d::UnitY() ).normalized();
	if ( fabs(cY2dY) < 0.00000001 )		rotAxis = Eigen::Vector3d::UnitX();	
	Eigen::Matrix3d headRotMat2AY = Eigen::AngleAxis<double>( cY2dY, rotAxis ).toRotationMatrix();
	
	Eigen::Matrix3d headRotMat;
	if ( ( trajectory_id == OBJECT_BOTH_HANDS ) )				headRotMat = headRotMat2AY * currHeadRotMat;
	else if ( ( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD ) )		headRotMat = Eigen::Quaternion<double>(current_poses[6], current_poses[3], current_poses[4], current_poses[5]).toRotationMatrix();


	// Head
	//   Position
	pepper_poses[0] = current_poses[0];	// Visualization
	pepper_poses[1] = current_poses[1];	// Visualization
	pepper_poses[2] = current_poses[2];	// Visualization
	//   Orientation
	pepper_poses[3] = Eigen::Quaternion<double>(headRotMat).x();
	pepper_poses[4] = Eigen::Quaternion<double>(headRotMat).y();
	pepper_poses[5] = Eigen::Quaternion<double>(headRotMat).z();
	pepper_poses[6] = Eigen::Quaternion<double>(headRotMat).w();

	// Right arm	// Visualization
	//   Position
	pepper_poses[7] = righthandPose.col(3)(0);
	pepper_poses[8] = righthandPose.col(3)(1);
	pepper_poses[9] = righthandPose.col(3)(2);
	//   Orientation
	pepper_poses[10] = Eigen::Quaternion<double>(righthandPose.block<3,3>(0,0)).x();
	pepper_poses[11] = Eigen::Quaternion<double>(righthandPose.block<3,3>(0,0)).y();
	pepper_poses[12] = Eigen::Quaternion<double>(righthandPose.block<3,3>(0,0)).z();
	pepper_poses[13] = Eigen::Quaternion<double>(righthandPose.block<3,3>(0,0)).w();
	
	// Left arm
	//   Position, X,Y
	pepper_poses[14] = lefthandPose.col(3)(0);
	pepper_poses[15] = lefthandPose.col(3)(1);
	pepper_poses[16] = lefthandPose.col(3)(2);
	//   Orientation w_x, w_y
	pepper_poses[17] = Eigen::Quaternion<double>(lefthandPose.block<3,3>(0,0)).x();
	pepper_poses[18] = Eigen::Quaternion<double>(lefthandPose.block<3,3>(0,0)).y();
	pepper_poses[19] = Eigen::Quaternion<double>(lefthandPose.block<3,3>(0,0)).z();
	pepper_poses[20] = Eigen::Quaternion<double>(lefthandPose.block<3,3>(0,0)).w();
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	if ( trajectory_id == OBJECT_BOTH_HANDS )
	{
	  //     Head - Position Y
	  vSel(1) = 1.0;
	  //     Right arm - Position
	  vSel(6) = vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	  //     Left arm - Position
	  vSel(12) = vSel(13) = vSel(14) = 1.0;
	  //     Left arm - Orientation
	  vSel(15) = vSel(16) = vSel(17) = 1.0;
	}
	else if ( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD )
	{
	  //     Right arm - Position
	  vSel(6) = vSel(7) = vSel(8) = 1.0;
	  //     Right arm - Orientation
	  vSel(9) = vSel(10) = vSel(11) = 1.0;
	  //     Left arm - Position
	  vSel(12) = vSel(13) = vSel(14) = 1.0;
	  //     Left arm - Orientation
	  vSel(15) = vSel(16) = vSel(17) = 1.0;
	}
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
      }
      
      else if ( ( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD_HANDS_POS_GRASPCOORD ) ) // ********************************************************************************************************
      {	
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	double tl = 0.0;
	double vl = 0.65;
	double ang_vel = PI/2;	// rad/s
	double radius = 0.2;	// m
	
	if ( cycle_time > 2*tl )		cycle_time -= 2*tl;

	// Object
	double objectPos[3];
	for (unsigned int i=0; i<3; i++)	objectPos[i] = 0.0;
	if (cycle_time <= tl)			objectPos[0] = vl*cycle_time;
	else if (cycle_time <= 2*tl)		objectPos[0] = vl*tl - vl*(cycle_time-tl);
	objectPos[1] = radius*cos(ang_vel * trajectory_time);
	objectPos[2] = 0.7 + radius*sin(ang_vel * trajectory_time);
	
	Eigen::Quaternion<double> objectQuat;
	objectQuat.x() = 0.0;
	objectQuat.y() = 0.0;
	objectQuat.z() = 0.0;
	objectQuat.w() = 1.0;
	
	Eigen::MatrixXd objectMat(4,4);
	objectMat = Eigen::MatrixXd::Identity(4,4);
	objectMat.block<3,3>(0,0) = objectQuat.toRotationMatrix();
	objectMat.col(3) << objectPos[0], objectPos[1], objectPos[2], 1.0;
	
	
	// Head
	Eigen::Quaternion<double> currHeadQuat;
	currHeadQuat.x() = current_poses[3];
	currHeadQuat.y() = current_poses[4];
	currHeadQuat.z() = current_poses[5];
	currHeadQuat.w() = current_poses[6];
	Eigen::Matrix3d currHeadRotMat;
	currHeadRotMat = currHeadQuat.toRotationMatrix();
	
	double cY2dY = acos( currHeadRotMat.col(1).dot( Eigen::Vector3d::UnitY() ) );
	Eigen::Vector3d rotAxis = currHeadRotMat.col(1).cross( Eigen::Vector3d::UnitY() ).normalized();
	if ( fabs(cY2dY) < 0.00000001 )		rotAxis = Eigen::Vector3d::UnitX();	
	Eigen::Matrix3d headRotMat2AY = Eigen::AngleAxis<double>( cY2dY, rotAxis ).toRotationMatrix();
	
	Eigen::Matrix3d headRotMat;
	if ( ( trajectory_id == OBJECT_BOTH_HANDS ) )				headRotMat = headRotMat2AY * currHeadRotMat;
	else if ( ( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD ) )	
	  headRotMat = Eigen::Quaternion<double>(current_poses[6], current_poses[3], current_poses[4], current_poses[5]).toRotationMatrix();


	// Head
	//   Position
	pepper_poses[0] = current_poses[0];	// Visualization
	pepper_poses[1] = current_poses[1];	// Visualization
	pepper_poses[2] = current_poses[2];	// Visualization
	//   Orientation
	pepper_poses[3] = Eigen::Quaternion<double>(headRotMat).x();
	pepper_poses[4] = Eigen::Quaternion<double>(headRotMat).y();
	pepper_poses[5] = Eigen::Quaternion<double>(headRotMat).z();
	pepper_poses[6] = Eigen::Quaternion<double>(headRotMat).w();

	// Right arm	// Visualization
	//   Position
	pepper_poses[7] = objectMat.col(3)(0);
	pepper_poses[8] = objectMat.col(3)(1);
	pepper_poses[9] = objectMat.col(3)(2);
	//   Orientation
	pepper_poses[10] = Eigen::Quaternion<double>(objectMat.block<3,3>(0,0)).x();
	pepper_poses[11] = Eigen::Quaternion<double>(objectMat.block<3,3>(0,0)).y();
	pepper_poses[12] = Eigen::Quaternion<double>(objectMat.block<3,3>(0,0)).z();
	pepper_poses[13] = Eigen::Quaternion<double>(objectMat.block<3,3>(0,0)).w();
	
	// Left arm	// Visualization
	//   Position, X,Y
	pepper_poses[14] = objectMat.col(3)(0);
	pepper_poses[15] = objectMat.col(3)(1);
	pepper_poses[16] = objectMat.col(3)(2);
	//   Orientation w_x, w_y
	pepper_poses[17] = Eigen::Quaternion<double>(objectMat.block<3,3>(0,0)).x();
	pepper_poses[18] = Eigen::Quaternion<double>(objectMat.block<3,3>(0,0)).y();
	pepper_poses[19] = Eigen::Quaternion<double>(objectMat.block<3,3>(0,0)).z();
	pepper_poses[20] = Eigen::Quaternion<double>(objectMat.block<3,3>(0,0)).w();
      }
      
      else if ( ( trajectory_id == HEAD_MOVE ) ||
	      ( trajectory_id == HEAD_MOVE_STARE ) ||
	      ( trajectory_id == HEAD_MOVE_STARE_FREEWX ) ) // ********************************************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	double ang_vel = PI/4;	// rad/s
	double radius = 1.0;	// m
	
	double headPos[3];
	headPos[0] = 1.0 - radius*fabs(cos(ang_vel * trajectory_time));
	headPos[1] = 0.0 + radius*sin(ang_vel * trajectory_time);
	headPos[2] = 1.1;

	
	double head_stare_point[3];	// Point to look for
	head_stare_point[0] = 1.0;
	head_stare_point[1] = 0.0;
	head_stare_point[2] = 1.2;
	
	Eigen::Matrix3d headRotMat;
	headRotMat.col(0) << Eigen::Vector3d(head_stare_point[0] - current_poses[0], head_stare_point[1] - current_poses[1], head_stare_point[2] - current_poses[2]).normalized();
	headRotMat.col(1) << Eigen::Vector3d::UnitZ().cross(headRotMat.col(0));
	headRotMat.col(2) =  headRotMat.col(0).cross(headRotMat.col(1));	
	
	
	// Head
	//   Position
	pepper_poses[0] = headPos[0];
	pepper_poses[1] = headPos[1];
	pepper_poses[2] = headPos[2];
	//   Orientation
	if ( trajectory_id == HEAD_MOVE )
	{
	  pepper_poses[3] = current_poses[3];
	  pepper_poses[4] = current_poses[4];
	  pepper_poses[5] = current_poses[5];
	  pepper_poses[6] = current_poses[6];
	}
	else if ( ( trajectory_id == HEAD_MOVE_STARE ) || ( trajectory_id == HEAD_MOVE_STARE_FREEWX ) )
	{
	  pepper_poses[3] = Eigen::Quaternion<double>(headRotMat).x();
	  pepper_poses[4] = Eigen::Quaternion<double>(headRotMat).y();
	  pepper_poses[5] = Eigen::Quaternion<double>(headRotMat).z();
	  pepper_poses[6] = Eigen::Quaternion<double>(headRotMat).w();
	}

	
	// Right arm
	//   Position
	pepper_poses[7] = current_poses[7];
	pepper_poses[8] = current_poses[8];
	pepper_poses[9] = current_poses[9];
	//   Orientation
	pepper_poses[10] = current_poses[10];
	pepper_poses[11] = current_poses[11];
	pepper_poses[12] = current_poses[12];
	pepper_poses[13] = current_poses[13];
	
	// Left arm
	//   Position
	pepper_poses[14] = current_poses[14];
	pepper_poses[15] = current_poses[15];
	pepper_poses[16] = current_poses[16];
	//   Orientation
	pepper_poses[17] = current_poses[17];
	pepper_poses[18] = current_poses[18];
	pepper_poses[19] = current_poses[19];
	pepper_poses[20] = current_poses[20];
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
	if ( trajectory_id == HEAD_MOVE )
	{
	  //   Head - Position
	  vSel(0) = vSel(1) = vSel(2) = 1.0;
	}
	else if ( trajectory_id == HEAD_MOVE_STARE )
	{
	  //   Head - Position
	  vSel(0) = vSel(1) = vSel(2) = 1.0;
	  //   Head - Orientation
	  vSel(3) = vSel(4) = vSel(5) = 1.0;
	}
	else if ( trajectory_id == HEAD_MOVE_STARE_FREEWX )
	{
	  //   Head - Position
	  vSel(0) = vSel(1) = vSel(2) = 1.0;
	  //   Head - Orientation w_y w_z
	  vSel(4) = vSel(5) = 1.0;
	}
      }      
      
      else if ( ( trajectory_id == HEAD_CRAZY_ARCH_FREEWX ) || 
	        ( trajectory_id == HEAD_CRAZY_ARCH ) ) // ********************************************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	double ang_vel = PI/4;	// rad/s
	double radius = 0.325;	// m
	
	double headPos[3];
	headPos[0] = 0.0;
	headPos[1] = 0.0 + radius*sin(ang_vel * trajectory_time);
	headPos[2] = 0.875 + radius*fabs(cos(ang_vel * trajectory_time));

	
	double head_stare_point[3];	// Point to look for
	head_stare_point[0] = 1.0;
	head_stare_point[1] = 0.0;
	head_stare_point[2] = 1.2;
	
	Eigen::Matrix3d headRotMat;
	headRotMat.col(0) << Eigen::Vector3d(head_stare_point[0] - current_poses[0], head_stare_point[1] - current_poses[1], head_stare_point[2] - current_poses[2]).normalized();
	headRotMat.col(1) << Eigen::Vector3d::UnitZ().cross(headRotMat.col(0));
	headRotMat.col(2) =  headRotMat.col(0).cross(headRotMat.col(1));	
	
	
	// Head
	//   Position
	pepper_poses[0] = headPos[0];
	pepper_poses[1] = headPos[1];
	pepper_poses[2] = headPos[2];
	//   Orientation
	pepper_poses[3] = Eigen::Quaternion<double>(headRotMat).x();
	pepper_poses[4] = Eigen::Quaternion<double>(headRotMat).y();
	pepper_poses[5] = Eigen::Quaternion<double>(headRotMat).z();
	pepper_poses[6] = Eigen::Quaternion<double>(headRotMat).w();
	
	// Right arm
	//   Position
	pepper_poses[7] = current_poses[7];
	pepper_poses[8] = current_poses[8];
	pepper_poses[9] = current_poses[9];
	//   Orientation
	pepper_poses[10] = current_poses[10];
	pepper_poses[11] = current_poses[11];
	pepper_poses[12] = current_poses[12];
	pepper_poses[13] = current_poses[13];
	
	// Left arm
	//   Position
	pepper_poses[14] = current_poses[14];
	pepper_poses[15] = current_poses[15];
	pepper_poses[16] = current_poses[16];
	//   Orientation
	pepper_poses[17] = current_poses[17];
	pepper_poses[18] = current_poses[18];
	pepper_poses[19] = current_poses[19];
	pepper_poses[20] = current_poses[20];
	
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
	if ( trajectory_id = HEAD_CRAZY_ARCH_FREEWX )
	{
	  //   Head - Position
	  vSel(1) = vSel(2) = 1.0;
	  //   Head - Orientation w_y w_z
	  vSel(4) = vSel(5) = 1.0;
	}
	else if ( trajectory_id = HEAD_CRAZY_ARCH )
	{
	  //   Head - Position
	  vSel(1) = vSel(2) = 1.0;
	  //   Head - Orientation w_y w_z
	  vSel(3) = vSel(4) = vSel(5) = 1.0;
	}
      }
      
      else if ( trajectory_id == HEAD_GAZE  ) // ********************************************************************************************************
      {
	// Kp = 0.20;
	// Kv = 0.0;
	// Ka = 7.0;
	
	double ang_vel = PI/4;	// rad/s
	double radius = 1.0;	// m
	
	double head_stare_point[3];	// Point to look at
	head_stare_point[0] =  1.0 + radius * fabs(cos(ang_vel * trajectory_time));
	head_stare_point[1] =  0.0 - radius *      sin(ang_vel * trajectory_time);
	head_stare_point[2] =  1.1;
	
	gazedPosition.x = head_stare_point[0];
	gazedPosition.y = head_stare_point[1];
	gazedPosition.z = head_stare_point[2];
	
	Eigen::Matrix3d headRotMat;
	headRotMat.col(0) << Eigen::Vector3d(head_stare_point[0] - current_poses[0], head_stare_point[1] - current_poses[1], head_stare_point[2] - current_poses[2]).normalized();
	headRotMat.col(1) << Eigen::Vector3d::UnitZ().cross(headRotMat.col(0));
	headRotMat.col(2)  = headRotMat.col(0).cross(headRotMat.col(1));
	Eigen::Quaternion<double> headGazeObjectQuat = Eigen::Quaternion<double>(headRotMat);
	

	// Configuration ---
	// Head
	//   Position
	pepper_poses[0] = current_poses[0];
	pepper_poses[1] = current_poses[1];
	pepper_poses[2] = current_poses[2];
	//   Orientation
	pepper_poses[3] = headGazeObjectQuat.x();
	pepper_poses[4] = headGazeObjectQuat.y();
	pepper_poses[5] = headGazeObjectQuat.z();
	pepper_poses[6] = headGazeObjectQuat.w();
	
	// Arms
	//   Position
	pepper_poses[7] = pepper_poses[14] = head_stare_point[0];
	pepper_poses[8] = pepper_poses[15] = head_stare_point[1];
	pepper_poses[9] = pepper_poses[16] = head_stare_point[2];
	//   Orientation
	pepper_poses[10] = pepper_poses[17] = 0.0;
	pepper_poses[11] = pepper_poses[18] = 0.0;
	pepper_poses[12] = pepper_poses[19] = 0.0;
	pepper_poses[13] = pepper_poses[20] = 1.0;
	
	// Selected dimensions
	vSel = Eigen::VectorXd::Zero(18);
	vSel(3) = vSel(4) = vSel(5) = 1.0;
	
	//   Frames
	sel_frames_quat = Eigen::VectorXd::Zero(PEPPER_N_TCP*8);
        sel_frames_quat(3) = sel_frames_quat(7) = sel_frames_quat(11) = sel_frames_quat(15) = sel_frames_quat(19) = sel_frames_quat(23) = 1.0;
      }      

      else	trajectory_id = HEADARM_POSES_CIRCLES;		// *************************************************************************************************************************************
            
            
      // Fill pepper_poses[] with torso, right and left elbows poses ------------------------------------------------------------------------------------------------------------------------------------
      for (unsigned int i=0; i<21; i++)		pepper_poses[i+21] = current_poses[i+21];
      

      // Markers -----------------------------------------------------------------------------------------------------------------------
      if ( (trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS) || (trajectory_id == RIGHTARM_TRANSPORT_OBJECT_POS_XY) )
      {
	// Cylinder in right hand
	Eigen::Quaternion<double> objRelHandQuat = Eigen::Quaternion<double>(Eigen::AngleAxis<double>( PI/2, Eigen::Vector3d::UnitX() ).toRotationMatrix());
	objRelHandQuat = Eigen::AngleAxis<double>( -1.0*(PI/180.0), Eigen::Vector3d::UnitZ() ).toRotationMatrix() * objRelHandQuat.toRotationMatrix();
	
	marker.id = marker_id++;
	marker.header.frame_id = "/r_wrist";
	marker.type = visualization_msgs::Marker::CYLINDER;     
	marker.scale.x = 0.032;
	marker.scale.y = 0.032;
	marker.scale.z = 0.2;
	marker.pose.position.x = 0.07;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = -0.02;
	marker.pose.orientation.x = objRelHandQuat.x();
	marker.pose.orientation.y = objRelHandQuat.y();
	marker.pose.orientation.z = objRelHandQuat.z();
	marker.pose.orientation.w = objRelHandQuat.w();
	marker.color.r = (153.0f/255.0f);
	marker.color.g = (76.0f/255.0f);
	marker.color.b = (0.0f/255.0f);
	marker.color.a = 1.0;
	marker_array.markers.push_back(marker); 
      }
      else if ( ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR ) || ( trajectory_id == LEFTARM_TRANSPORT_OBJECT_POS_XY_OR_XY ) )
      {
	// Disk (cylinder) in left hand
	Eigen::Quaternion<double> objRelHandQuat = Eigen::Quaternion<double>(Eigen::AngleAxis<double>( 0.0, Eigen::Vector3d::UnitX() ).toRotationMatrix());
	
	marker.id = marker_id++;
	marker.header.frame_id = "/l_wrist";
	marker.type = visualization_msgs::Marker::CYLINDER;     
	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.01;
	marker.pose.position.x = 0.07;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = -0.035;
	marker.pose.orientation.x = objRelHandQuat.x();
	marker.pose.orientation.y = objRelHandQuat.y();
	marker.pose.orientation.z = objRelHandQuat.z();
	marker.pose.orientation.w = objRelHandQuat.w();
	marker.color.r = (175.0f/255.0f);
	marker.color.g = (247.0f/255.0f);
	marker.color.b = (255.0f/255.0f);
	marker.color.a = 1.0;
	marker_array.markers.push_back(marker); 
      }      
      else if ( ( trajectory_id == OBJECT_BOTH_HANDS ) ||
		( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD ) ||
		( trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD_HANDS_POS_GRASPCOORD ) )
      {
	// Sphere attached to right hand
	Eigen::Matrix3d objRelHandRotMat;
	
	// CYLINDER
	objRelHandRotMat.col(0) << 1.0, 0.0, 0.0;
	objRelHandRotMat.col(1) << 0.0, 1.0, 0.0;
	objRelHandRotMat.col(2) << 0.0, 0.0, 1.0;
	marker.type = visualization_msgs::Marker::CYLINDER;
	double cylinder_radius = 0.15;
	double cylinder_heigth = 0.3;
	marker.scale.x = cylinder_radius;
	marker.scale.y = cylinder_radius;
	marker.scale.z = cylinder_heigth;
	marker.pose.position.x = 0.125;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = -0.17;
	
	marker.header.frame_id = "/r_wrist";
	marker.id = marker_id++;
	marker.pose.orientation.x = Eigen::Quaternion<double>(objRelHandRotMat).x();
	marker.pose.orientation.y = Eigen::Quaternion<double>(objRelHandRotMat).y();
	marker.pose.orientation.z = Eigen::Quaternion<double>(objRelHandRotMat).z();
	marker.pose.orientation.w = Eigen::Quaternion<double>(objRelHandRotMat).w();
	marker.color.r = (255.0f/255.0f);
	marker.color.g = (152.0f/255.0f);
	marker.color.b = (48.0f/255.0f);
	marker.color.a = 1.0;
	marker_array.markers.push_back(marker); 
      }  
      
      
      
      // Publish --------------------------------------------------------------------------------------------------------------------------
      
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

//       pepper_pose_trajectory_pub.publish(rosPepperPoses);
      
      
      //   Publish dimension selection and frames
      for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)		ROSdimensionSelection.data[i] = vSel(i);
      dimension_selection_pub.publish(ROSdimensionSelection);
      
      for (unsigned int i=0; i<PEPPER_N_TCP*8; i++)		ROSdimensionSelectionFramesQuat.data[i] = sel_frames_quat(i);
      dimension_selection_frames_pub.publish(ROSdimensionSelectionFramesQuat);
      
      
      //   Gazed object
      gazed_position_pub.publish(gazedPosition);      
      
      
      //   Markers
//       marker_array_pub.publish(marker_array);
    }

    
    ros::spinOnce(); 
    loop_rate.sleep();
  }

    
  return 0;
}