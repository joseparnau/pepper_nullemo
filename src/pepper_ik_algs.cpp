
#include <pepper_ik_algs.hpp>
#include <pepper_trajectories_data.hpp>

#include <sstream>
#include <iostream>
#include <iomanip>

#include <Eigen/SparseCore>


void compute_next_configuration(double* current_configuration, double* desired_poses, unsigned int* dimension_selection, double* dimension_selection_frames,
				double *gazed_position,
				KDL::ChainFkSolverPos_recursive** fk_solvers,
				KDL::ChainJntToJacSolver** jacobian_solvers,
				double* past_desired_poses,
				Eigen::VectorXd &past_configuration_inc,
				double* past_diffH,
				const unsigned int trajectory_id,
				const double period,
				const double alg_time,
				const unsigned int n_it,
				double &data
 			      )
{
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
//       8-12:   'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'
//     LEFT ARM
//       13-17:  'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'
// 
//  desired_poses: [{P_head, q_head}, {P_RightArm, q_RightArm}, {P_LeftArm, q_LeftArm}] = [ {px, py, pz, qx, qy, qz, qw} ]
 
  
  
  // ****************************************************************************************************************************************************************************************
  // KDL CHAINS CONFIGURATIONS, DIRECT KINEMATICS & JACOBIANS *******************************************************************************************************************************
  
  
  // Chain initialization ------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
  KDL::JntArray* config_chains = new KDL::JntArray[PEPPER_N_CHAINS];
  
  //  HEAD
  config_chains[0]     = KDL::JntArray(PEPPER_HEAD_CHAIN_JOINTS);
  config_chains[0](0)  = current_configuration[0];
  config_chains[0](1)  = current_configuration[1]; 
  config_chains[0](2)  = current_configuration[2]; 
  config_chains[0](3)  = current_configuration[3]; 
  config_chains[0](4)  = current_configuration[4]; 
  config_chains[0](5)  = current_configuration[5]; 
  config_chains[0](6)  = current_configuration[6];
  config_chains[0](7)  = current_configuration[7];
  
  //  RIGHT ARM
  config_chains[1]     = KDL::JntArray(PEPPER_RIGHTARM_CHAIN_JOINTS);
  config_chains[1](0)  = current_configuration[0];
  config_chains[1](1)  = current_configuration[1];
  config_chains[1](2)  = current_configuration[2];
  config_chains[1](3)  = current_configuration[3];
  config_chains[1](4)  = current_configuration[4];
  config_chains[1](5)  = current_configuration[5];
  config_chains[1](6)  = current_configuration[8];
  config_chains[1](7)  = current_configuration[9];
  config_chains[1](8)  = current_configuration[10];
  config_chains[1](9)  = current_configuration[11];
  config_chains[1](10) = current_configuration[12];

  //  LEFT ARM
  config_chains[2]     = KDL::JntArray(PEPPER_LEFTARM_CHAIN_JOINTS);
  config_chains[2](0)  = current_configuration[0];
  config_chains[2](1)  = current_configuration[1]; 
  config_chains[2](2)  = current_configuration[2]; 
  config_chains[2](3)  = current_configuration[3]; 
  config_chains[2](4)  = current_configuration[4]; 
  config_chains[2](5)  = current_configuration[5]; 
  config_chains[2](6)  = current_configuration[13];
  config_chains[2](7)  = current_configuration[14];
  config_chains[2](8)  = current_configuration[15];
  config_chains[2](9)  = current_configuration[16];
  config_chains[2](10) = current_configuration[17];

  //  TORSO
  config_chains[3]     = KDL::JntArray(PEPPER_TORSO_CHAIN_JOINTS);
  config_chains[3](0)  = current_configuration[0];
  config_chains[3](1)  = current_configuration[1]; 
  config_chains[3](2)  = current_configuration[2]; 
  config_chains[3](3)  = current_configuration[3]; 
  config_chains[3](4)  = current_configuration[4]; 
  config_chains[3](5)  = current_configuration[5]; 

  //  RIGHT ELBOW
  config_chains[4]     = KDL::JntArray(PEPPER_RIGHTELBOW_CHAIN_JOINTS);
  config_chains[4](0)  = current_configuration[0];
  config_chains[4](1)  = current_configuration[1];
  config_chains[4](2)  = current_configuration[2];
  config_chains[4](3)  = current_configuration[3];
  config_chains[4](4)  = current_configuration[4];
  config_chains[4](5)  = current_configuration[5];
  config_chains[4](6)  = current_configuration[8];
  config_chains[4](7)  = current_configuration[9];
  
  //  LEFT ELBOW
  config_chains[5]     = KDL::JntArray(PEPPER_LEFTELBOW_CHAIN_JOINTS);
  config_chains[5](0)  = current_configuration[0];
  config_chains[5](1)  = current_configuration[1]; 
  config_chains[5](2)  = current_configuration[2]; 
  config_chains[5](3)  = current_configuration[3]; 
  config_chains[5](4)  = current_configuration[4]; 
  config_chains[5](5)  = current_configuration[5]; 
  config_chains[5](6)  = current_configuration[13];
  config_chains[5](7)  = current_configuration[14];

  
  // Direct kinematics ----------------------------------------------------------------------------------------------------------------------------------------------------------------------  
  KDL::Frame* KDL_TCPs_frames = new KDL::Frame[PEPPER_N_CHAINS];
  for (unsigned int n_tcp = 0; n_tcp<PEPPER_N_CHAINS; n_tcp++)		fk_solvers[n_tcp]->JntToCart(config_chains[n_tcp], KDL_TCPs_frames[n_tcp]);
  
  
  // Chain jacobians ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  KDL::Jacobian* kdl_tcp_jacobian = new KDL::Jacobian[PEPPER_N_CHAINS];
  kdl_tcp_jacobian[0] = KDL::Jacobian(PEPPER_HEAD_CHAIN_JOINTS);
  kdl_tcp_jacobian[1] = KDL::Jacobian(PEPPER_RIGHTARM_CHAIN_JOINTS);
  kdl_tcp_jacobian[2] = KDL::Jacobian(PEPPER_LEFTARM_CHAIN_JOINTS);
  kdl_tcp_jacobian[3] = KDL::Jacobian(PEPPER_TORSO_CHAIN_JOINTS);
  kdl_tcp_jacobian[4] = KDL::Jacobian(PEPPER_RIGHTELBOW_CHAIN_JOINTS);
  kdl_tcp_jacobian[5] = KDL::Jacobian(PEPPER_LEFTELBOW_CHAIN_JOINTS);
  for (unsigned int i=0; i<PEPPER_N_CHAINS; i++)	jacobian_solvers[i]->JntToJac(config_chains[i], kdl_tcp_jacobian[i]);  
  
  
  
  // ****************************************************************************************************************************************************************************************
  // EMOTION CONVEYANCE *********************************************************************************************************************************************************************
  
  // Extent configurations
  double cont_cfg[PEPPER_TOTAL_N_JOINTS_SIMP], ext_cfg[PEPPER_TOTAL_N_JOINTS_SIMP];
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		cont_cfg[i] = ext_cfg[i] = 0.0;
  // EXTENT = 0 --------
  //   Torso
  cont_cfg[3]  =  0.1;
  cont_cfg[4]  =  0.1;
  //   Head
  cont_cfg[7]  =  0.5;
  //   Right arm
  cont_cfg[8]  =  0.9;
  cont_cfg[9]  = -0.3;
  cont_cfg[10] =  0.0;
  cont_cfg[11] =  1.1;
  cont_cfg[12] =  1.0;
  //   Left arm
  cont_cfg[13] =  0.9;
  cont_cfg[14] =  0.3;
  cont_cfg[15] =  0.0;
  cont_cfg[16] = -1.1;
  cont_cfg[17] = -1.0;
  // EXTENT = 1 ----------
  //   Torso
  ext_cfg[3]   =  0.0;
  ext_cfg[4]   = -0.05;
  //   Head
  ext_cfg[7]   = -0.1;
  //   Right arm
  ext_cfg[8]   =  1.57;
  ext_cfg[9]   = -0.85;
  ext_cfg[10]  =  1.50;
  ext_cfg[11]  =  0.1;
  ext_cfg[12]  =  1.15;
  //   Left arm
  ext_cfg[13]  =  1.57;
  ext_cfg[14]  =  0.85;
  ext_cfg[15]  = -1.50;
  ext_cfg[16]  = -0.1;
  ext_cfg[17]  = -1.15;

  // Energetic level amplitudes  
  double KAmp[PEPPER_TOTAL_N_JOINTS_SIMP];
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)		KAmp[i] = 0.0;
  //   Torso
  KAmp[5]  =  0.15;
  //   Head
  KAmp[6]  =  0.25;
  //   Right arm
  KAmp[9]  =  0.65;
  KAmp[10] =  0.7;
  KAmp[11] =  0.3;
  KAmp[12] = -0.2;
  //   Left arm
  KAmp[14] = -0.65;
  KAmp[15] = -0.7;
  KAmp[16] = -0.3;
  KAmp[17] =  0.2; 
    
  // Symmetry
  double sym_A[5], sym_w[5], sym_sin[5], sym_sin_past[5];
  for (unsigned int i=0; i<5; i++)					sym_sin[i] = sym_sin_past[i] = 0.0;
  //   Amplitudes
  sym_A[0] = 0.0;		// Non-symmetric parts
  sym_A[1] = 0.5;		// Torso
  sym_A[2] = 0.6;		// Head
  sym_A[3] = 0.75;		// Right arm
  sym_A[4] = 0.65;		// Left arm
  //   Angular velocities
  sym_w[0] = 0.0;		// Non-symmetric parts
  sym_w[1] = (2.0*PI)/11.7;	// Torso
  sym_w[2] = (2.0*PI)/9.0;	// Head
  sym_w[3] = (2.0*PI)/10.5;	// Right arm
  sym_w[4] = (2.0*PI)/15.5;	// Left arm

  // Test values
  double w     	   = (2.0*PI)/2.25;
//   double ext   = 0.5 - 0.5*cos(w*alg_time);		// Extension
  double extension = 1.0;				// Extension
  double jerkiness = 1.0;				// Jerkiness
  double symmetry  = 1.0;				// Symmetry
  double dominance = 0.0;				// Dominance
  if		( extension < 0.0 )	extension = 0.0;
  else if	( extension > 1.0 )	extension = 1.0;
  if		( jerkiness < 0.0 )	jerkiness = 0.0;
  else if	( jerkiness > 1.0 )	jerkiness = 1.0;
  if		( symmetry  < 0.0 )	symmetry  = 0.0;
  else if	( symmetry  > 1.0 )	symmetry  = 1.0;
  if		( dominance < 0.0 )	dominance = 0.0;
  else if	( dominance > 1.0 )	dominance = 1.0;
  
  double emotion_configuration[PEPPER_TOTAL_N_JOINTS_SIMP];
  for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)
  {
    unsigned int i_sym = 0;										// Non-symmetric parts
    if      ( i == 5 )									i_sym = 1;	// Torso
    else if ( i == 6 )									i_sym = 2;	// Neck
    else if ( (i == 8)  || (i == 9)  || (i == 10) || (i == 11) || (i == 12) )		i_sym = 3;	// Right arm
    else if ( (i == 13) || (i == 14) || (i == 15) || (i == 16) || (i == 17) )		i_sym = 4;	// Left arm
    
    // Symmetry
    sym_sin[i_sym] += period;
    if (  ( sin(sym_w[i_sym]*sym_sin_past[i_sym]) <  0.0)  &&  ( sin(sym_w[i_sym]*sym_sin[i_sym]) >= 0.0)  )
    {
      sym_A[i_sym]   = 0.3 + 0.6*rnd0to1();
      sym_w[i_sym]   = (2.0*PI)/( 15.0 + 5.0*rnd0to1() );
      sym_sin[i_sym] = 0.0;
    }
    sym_sin_past[i_sym] = sym_sin[i_sym];
    double sym_offset = (1.0 - symmetry) * sym_A[i_sym] * sin( sym_w[i_sym] * sym_sin[i_sym] );      
    
    // Jerkiness
    double jerky_w      = (2*PI)/1.0;
    double jerky_offset = jerkiness * ( (1.0/6.0) * sin( jerky_w * alg_time + sym_offset) + (1.0/4.0) * cos( 1.5*jerky_w * alg_time  + sym_offset) );
    
    // Extension
//           KAmp[i] = 0.0;
    if ((i == 5) || (i == 6))	emotion_configuration[i] = ( 1.0 - extension ) * cont_cfg[i] + extension * ( ext_cfg[i] + (1.0 - symmetry) * KAmp[i] * sin( w * alg_time + jerky_offset + sym_offset ) );
    else			emotion_configuration[i] = ( 1.0 - extension ) * cont_cfg[i] + extension * ( ext_cfg[i] +                    KAmp[i] * sin( w * alg_time + jerky_offset + sym_offset ) );
  }
  double righthand_jnt_val, lefthand_jnt_val;
  righthand_jnt_val = lefthand_jnt_val = 0.25 + 0.8 * extension;
  
  // Gaze
  double pepper_poses_aux[PEPPER_N_CHAINS*7];
  compute_poses_from_config(fk_solvers, emotion_configuration, pepper_poses_aux);
  
  Eigen::Quaternion<double> emotionalHeadRot(pepper_poses_aux[6],pepper_poses_aux[3],pepper_poses_aux[4],pepper_poses_aux[5]);
  Eigen::Quaternion<double> domGazeRot(desired_poses[6],desired_poses[3],desired_poses[4],desired_poses[5]);
  
  // Head tests -------------------------------------------------------------------- TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST
  double ang_vel = PI/4;	// rad/s
  double radius = 0.5;		// m
  
  double head_stare_point[3];	// Point to look at
  head_stare_point[0] =  1.0 + radius * fabs(cos(ang_vel * alg_time));
  head_stare_point[1] =  0.0 - radius *      sin(ang_vel * alg_time);
  head_stare_point[2] =  1.1;
  
  Eigen::Matrix3d headRotMat;
  headRotMat.col(0) << Eigen::Vector3d(head_stare_point[0] - KDL_TCPs_frames[0](0,3), head_stare_point[1] - KDL_TCPs_frames[0](1,3), head_stare_point[2] - KDL_TCPs_frames[0](2,3)).normalized();
  headRotMat.col(1) << Eigen::Vector3d::UnitZ().cross(headRotMat.col(0));
  headRotMat.col(2)  = headRotMat.col(0).cross(headRotMat.col(1));
  domGazeRot = Eigen::Quaternion<double>(headRotMat);
  // Head tests -------------------------------------------------------------------- TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST TEST
   Eigen::Quaternion<double> gaze_quat = emotionalHeadRot.slerp(dominance, domGazeRot);

  desired_poses[3] = gaze_quat.x();
  desired_poses[4] = gaze_quat.y();
  desired_poses[5] = gaze_quat.z();
  desired_poses[6] = gaze_quat.w();

 
  
  // ****************************************************************************************************************************************************************************************
  // ERROR VECTOR ***************************************************************************************************************************************************************************
  
  double Kp = 1.0;
  
  Eigen::Vector3d* alg_error_p = new Eigen::Vector3d[PEPPER_N_TCP];
  Eigen::Vector3d* alg_error_w = new Eigen::Vector3d[PEPPER_N_TCP];  
  for (unsigned int n_tcp = 0; n_tcp < PEPPER_N_TCP; n_tcp++)
  {
    alg_error_p[n_tcp] = Kp * Eigen::Vector3d(desired_poses[n_tcp*7 + 0] - KDL_TCPs_frames[n_tcp](0,3), desired_poses[n_tcp*7 + 1] - KDL_TCPs_frames[n_tcp](1,3), desired_poses[n_tcp*7 + 2] - KDL_TCPs_frames[n_tcp](2,3));
   
    //alg_error_w[n_tcp] = Kp * sicil_or_error(Eigen::Quaternion<double>(desired_poses[n_tcp*7 + 6], desired_poses[n_tcp*7 + 3], desired_poses[n_tcp*7 + 4], desired_poses[n_tcp*7 + 5]), KDL_TCPs_frames[n_tcp]);
    Eigen::Quaternion<double> desired_quat(desired_poses[n_tcp*7 + 6], desired_poses[n_tcp*7 + 3], desired_poses[n_tcp*7 + 4], desired_poses[n_tcp*7 + 5]);
    Eigen::Matrix3d currRotMat;
    for (unsigned int i=0; i<3; i++)    for (unsigned int j=0; j<3; j++)	currRotMat(i,j) = KDL_TCPs_frames[n_tcp](i,j);
    Eigen::AngleAxis<double> errRotAngAxis = Eigen::AngleAxis<double>(desired_quat.toRotationMatrix() * currRotMat.transpose());
    alg_error_w[n_tcp] = Kp * errRotAngAxis.angle() * errRotAngAxis.axis();
  }  

  
  
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //  PRIORITY LEVEL - Start
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  Eigen::VectorXd configuration_inc(PEPPER_TOTAL_N_JOINTS_SIMP);  
  
  unsigned int priority_distribution_id = 3;
  switch(priority_distribution_id)
  {
	case 0:	// 1: HEAD, RIGHT ARM, LEFT ARM --------------------------------------------------------------------------------------------------------------------------------------------
	{
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)   = kdl_tcp_jacobian[0].data.block<6,6>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = kdl_tcp_jacobian[0].data.block<6,2>(0,6);		// HEAD
		jacobian.block<6,6>(6,0)   = kdl_tcp_jacobian[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,8)   = kdl_tcp_jacobian[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(12,0)  = kdl_tcp_jacobian[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,13) = kdl_tcp_jacobian[2].data.block<6,5>(0,6);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
													 dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001);
		
		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;

		break;
	}
	
	
	case 1:	// 1: HEAD, RIGHT ARM, LEFT ARM; 2: Optimization function ------------------------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  
		// Errors
		Eigen::VectorXd pose_error(6*PEPPER_N_TCP);
		pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();

		// Jacobian
		Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)   = kdl_tcp_jacobian[0].data.block<6,6>(0,0);		// HEAD
		jacobian.block<6,2>(0,6)   = kdl_tcp_jacobian[0].data.block<6,2>(0,6);		// HEAD
		jacobian.block<6,6>(6,0)   = kdl_tcp_jacobian[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(6,8)   = kdl_tcp_jacobian[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(12,0)  = kdl_tcp_jacobian[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(12,13) = kdl_tcp_jacobian[2].data.block<6,5>(0,6);		// LEFT ARM

		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(PEPPER_N_TCP*6,PEPPER_N_TCP*6);
		for (unsigned int i=0; i<2*PEPPER_N_TCP; i++)		R_S.block<3,3>(3*i,3*i) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
													 dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,PEPPER_N_TCP*6> Sel = Eigen::VectorXd::Zero(PEPPER_N_TCP*6).asDiagonal();
		for (unsigned int i=0; i<PEPPER_N_TCP*6; i++)	Sel.diagonal()[i] =  dimension_selection[i];

		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;

		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001);

		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level - OPTIMIZATION FUNCTION ++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd optfunc_error;		// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
		if (n_it == 1)	optfunc_error.setZero();

		// Jacobian
		Eigen::MatrixXd optfunc_jacobian;	// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO

		Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(18,18);
		configuration_inc = configuration_inc +
				    pinv( optfunc_jacobian * ( Id - task_jacobian_pinv * task_jacobian ) , 0.001) * ( optfunc_error - optfunc_jacobian * configuration_inc );

		break;
	}	
	
	
	case 2:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation -----------------------------------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	  
		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();

		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)  = kdl_tcp_jacobian[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,8)  = kdl_tcp_jacobian[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(6,0)  = kdl_tcp_jacobian[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,13) = kdl_tcp_jacobian[2].data.block<6,5>(0,6);		// LEFT ARM

		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
												 dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];

		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;

		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001);

		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;


		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::VectorXd pose_error_2(3);
		pose_error_2 << alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian_2.setZero();
		jacobian_2.block<3,8>(0,0) = kdl_tcp_jacobian[0].data.block<3,8>(3,0);		// HEAD
		
		// Dimension selection matrix
		Eigen::Matrix3d R_S_2 = Eigen::Quaternion<double>( dimension_selection_frames[4], dimension_selection_frames[4], 
								   dimension_selection_frames[4], dimension_selection_frames[4] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,3> Sel_2 = Eigen::Vector3d::Zero().asDiagonal();
		for (unsigned int i=3; i<6; i++)	Sel_2.diagonal()[i-3] =  dimension_selection[i];
		
		// Increment
		Eigen::MatrixXd task_jacobian_2 = R_S_2 * Sel_2 * R_S_2.transpose() * jacobian_2;
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(18,18);
		configuration_inc = configuration_inc +
				    pinv( task_jacobian_2 * ( Id - task_jacobian_pinv * task_jacobian ) , 0.001) * ( pose_error_2 - task_jacobian_2 * configuration_inc );

		break;
	}
	
	
	case 3:	// 1: RIGHT ARM, LEFT ARM; 2: HEAD Orientation; 3: Optimization function ---------------------------------------------------------------------------------------------------
	{
		// 1rst Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		Eigen::VectorXd pose_error(6*2);
		pose_error.block<6,1>(0,0) <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);	// RIGHT ARM
		pose_error.block<6,1>(6,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2);	// LEFT ARM
		if (n_it == 1)	pose_error.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian(6*2, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian.setZero();
		jacobian.block<6,6>(0,0)  = kdl_tcp_jacobian[1].data.block<6,6>(0,0);		// RIGHT ARM
		jacobian.block<6,5>(0,8)  = kdl_tcp_jacobian[1].data.block<6,5>(0,6);		// RIGHT ARM
		jacobian.block<6,6>(6,0)  = kdl_tcp_jacobian[2].data.block<6,6>(0,0);		// LEFT ARM
		jacobian.block<6,5>(6,13) = kdl_tcp_jacobian[2].data.block<6,5>(0,6);		// LEFT ARM
		
		// Dimension selection matrix
		Eigen::MatrixXd R_S = Eigen::MatrixXd::Identity(2*6,2*6);
		for (unsigned int i=2; i<6; i++)	R_S.block<3,3>(3*(i-2),3*(i-2)) = Eigen::Quaternion<double>( dimension_selection_frames[4*i+3], dimension_selection_frames[4*i+0], 
												 dimension_selection_frames[4*i+1], dimension_selection_frames[4*i+2] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,12> Sel = Eigen::VectorXd::Zero(12).asDiagonal();
		for (unsigned int i=6; i<18; i++)	Sel.diagonal()[i-6] =  dimension_selection[i];
		
		Eigen::MatrixXd task_jacobian = R_S * Sel * R_S.transpose() * jacobian;
		
		Eigen::MatrixXd task_jacobian_pinv = pinv( task_jacobian , 0.001);
		
		// Inverse Kinematics
		configuration_inc = task_jacobian_pinv * pose_error;

		//   Check configuration velocity limits
		saturate_joint_velocity(configuration_inc, period);
		

		// 2nd Priority Level +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		
		// Errors
		Eigen::VectorXd pose_error_2(3);
		pose_error_2 << alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);	// HEAD
		if (n_it == 1)	pose_error_2.setZero();
		
		// Jacobian
		Eigen::MatrixXd jacobian_2(3, PEPPER_TOTAL_N_JOINTS_SIMP);
		jacobian_2.setZero();
		jacobian_2.block<3,8>(0,0) = kdl_tcp_jacobian[0].data.block<3,8>(3,0);		// HEAD
		
		// Dimension selection matrix
		Eigen::Matrix3d R_S_2 = Eigen::Quaternion<double>( dimension_selection_frames[4], dimension_selection_frames[4], 
								   dimension_selection_frames[4], dimension_selection_frames[4] ).toRotationMatrix();
		Eigen::DiagonalMatrix<double,3> Sel_2 = Eigen::Vector3d::Zero().asDiagonal();
		for (unsigned int i=3; i<6; i++)	Sel_2.diagonal()[i-3] =  dimension_selection[i];
		
		// Increment
		Eigen::MatrixXd task_jacobian_2 = R_S_2 * Sel_2 * R_S_2.transpose() * jacobian_2;
		
// 		configuration_inc = pinv( task_jacobian_2 , 0.001) * pose_error_2;
		Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(18,18);
		configuration_inc = configuration_inc + pinv( task_jacobian_2 * ( Id - task_jacobian_pinv * task_jacobian ) , 0.001) * ( pose_error_2 - task_jacobian_2 * configuration_inc );

		//   Check configuration velocity limits
		saturate_joint_velocity(configuration_inc, period);


		// 3rd Priority Level - OPTIMIZATION FUNCTION +++++++++++++++++++++++++++++++++++++++++++++++++

		// Errors
		double Km = 1.0;
		Eigen::VectorXd optfunc_error(PEPPER_TOTAL_N_JOINTS_SIMP);
		if (n_it == 1)		optfunc_error.setZero();
		else			for (unsigned int i=0; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)	optfunc_error(i) = Km * ( emotion_configuration[i] - current_configuration[i] );
		
		// Jacobian
		Eigen::MatrixXd optfunc_jacobian(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP);
		optfunc_jacobian = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP);
		
		Eigen::MatrixXd task_jacobian_jacobian_1_2(15,PEPPER_TOTAL_N_JOINTS_SIMP);
		task_jacobian_jacobian_1_2.block<12,PEPPER_TOTAL_N_JOINTS_SIMP>(0,0) = task_jacobian;
		task_jacobian_jacobian_1_2.block<3,PEPPER_TOTAL_N_JOINTS_SIMP>(12,0) = task_jacobian_2;
		
		configuration_inc = configuration_inc +
				    pinv(  optfunc_jacobian * ( Id - pinv(task_jacobian_jacobian_1_2,0.001) * task_jacobian_jacobian_1_2  ) , 0.001) * ( optfunc_error - optfunc_jacobian * configuration_inc );

		break;
	}
  }
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //  PRIORITY LEVEL - End
  // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  

  
  
  // ****************************************************************************************************************************************************************************************
  // BOUNDARY CHECKING **********************************************************************************************************************************************************************

  //   Check configuration velocity limits
  saturate_joint_velocity(configuration_inc, period);
  
  //   Check configuration acceleration limits
//   if (n_it == 1)	past_configuration_inc = configuration_inc;  
//   saturate_joint_acceleration(configuration_inc, past_configuration_inc, period);
//   past_configuration_inc = configuration_inc;
 
  //   Compute new configuration
  for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)    current_configuration[i] = current_configuration[i] + configuration_inc(i);
  
  //   Saturate joint values
  saturate_joint_angles(current_configuration);
  
  
  
  
  
  
  
  
  
  // --------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // GRASP TEST ---------------------------------------------------------------------------------------------------------------------------------------------------------
  if (trajectory_id == OBJECT_BOTH_HANDS_NO_HEAD_HANDS_POS_GRASPCOORD)							//      Right arm [X,Y,Z, orientation] & Head [Orientation] 
  //   12 - H00 R33 L33 - Grasp Coordination
  {
    //  GRASP:  In intertial frame ref G_i^T = R_i^T * P_i^T = P_i^T (because the contact is rigid so R_i^T will multiply R_i and cancel when pseudoinverting:
    //    H R^T G^T v = H R^T J dq
    //    Contact is rigid, thus H = Id 
    //    R^T G^T v = R^T J dq
    //    G^T v = J dq
    //    v = G^-T J dq
    //    dq = (G^-T J)^+ v
    //
    //    But we have G in reference object [G]_B so G = [G]_N = C_N^B [G]_B (C_N^B)^-1
    //    thus
    //    [dq]_N = (  [G]_N^-T [J]_N  )^+ [v]_N
    //    [dq]_N = (  ( C_N^B [G]_B (C_N^B)^-1 )^-T [J]_N  )^+ [v]_N

    unsigned int n_err = 12;	
    unsigned int n_cfg = 16;
    Eigen::MatrixXd partJac(n_err,n_cfg);	
    Eigen::MatrixXd pInvPartJac(n_cfg, n_err);	
    Eigen::VectorXd partErr(n_err);	
    Eigen::VectorXd partCfg(n_cfg);
    
//     std::cout << "a1 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    Eigen::Vector3d dRightRefContactPoint, dLeftRefContactPoint;
    dRightRefContactPoint << -0.125, -0.17, 0.0;
    dLeftRefContactPoint  << -0.125,  0.17, 0.0;
    Eigen::VectorXd dRightRefContactPointExt(4), dLeftRefContactPointExt(4);
    dRightRefContactPointExt << dRightRefContactPoint(0), dRightRefContactPoint(1), dRightRefContactPoint(2), 0.0;
    dLeftRefContactPointExt  <<  dLeftRefContactPoint(0),  dLeftRefContactPoint(1),  dLeftRefContactPoint(2), 0.0;
    
//     std::cout << "a2 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    Eigen::MatrixXd ChangeBasisFromRightContactPointToWorld(4,4);
    ChangeBasisFromRightContactPointToWorld.col(0) << KDL_TCPs_frames[1](0,0), KDL_TCPs_frames[1](1,0), KDL_TCPs_frames[1](2,0), 0.0;
    ChangeBasisFromRightContactPointToWorld.col(1) << KDL_TCPs_frames[1](0,1), KDL_TCPs_frames[1](1,1), KDL_TCPs_frames[1](2,1), 0.0;
    ChangeBasisFromRightContactPointToWorld.col(2) << KDL_TCPs_frames[1](0,2), KDL_TCPs_frames[1](1,2), KDL_TCPs_frames[1](2,2), 0.0;
    ChangeBasisFromRightContactPointToWorld.col(3) << KDL_TCPs_frames[1](0,3), KDL_TCPs_frames[1](1,3), KDL_TCPs_frames[1](2,3), 1.0;
    Eigen::MatrixXd ChangeBasisFromLeftContactPointToWorld(4,4);
    ChangeBasisFromLeftContactPointToWorld.col(0) << KDL_TCPs_frames[2](0,0), KDL_TCPs_frames[2](1,0), KDL_TCPs_frames[2](2,0), 0.0;
    ChangeBasisFromLeftContactPointToWorld.col(1) << KDL_TCPs_frames[2](0,1), KDL_TCPs_frames[2](1,1), KDL_TCPs_frames[2](2,1), 0.0;
    ChangeBasisFromLeftContactPointToWorld.col(2) << KDL_TCPs_frames[2](0,2), KDL_TCPs_frames[2](1,2), KDL_TCPs_frames[2](2,2), 0.0;
    ChangeBasisFromLeftContactPointToWorld.col(3) << KDL_TCPs_frames[2](0,3), KDL_TCPs_frames[2](1,3), KDL_TCPs_frames[2](2,3), 1.0;
    
//     std::cout << "a3 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    Eigen::Vector3d dRightRefWorld, dLeftRefWorld;
    dRightRefWorld = (ChangeBasisFromRightContactPointToWorld * dRightRefContactPointExt).block<3,1>(0,0);
    dLeftRefWorld  = ( ChangeBasisFromLeftContactPointToWorld *  dLeftRefContactPointExt).block<3,1>(0,0);

//     std::cout << "a4 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    Eigen::MatrixXd graspRightRefWorld(6,6);
    graspRightRefWorld = Eigen::MatrixXd::Identity(6,6);
    graspRightRefWorld.block<3,3>(3,0) = cross_product_matrix_S(dRightRefWorld);
    
//     std::cout << "a5 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    Eigen::MatrixXd graspLeftRefWorld(6,6);
    graspLeftRefWorld = Eigen::MatrixXd::Identity(6,6);
    graspLeftRefWorld.block<3,3>(3,0) = cross_product_matrix_S(dLeftRefWorld);
    
//     std::cout << "a6 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    Eigen::MatrixXd graspTransposeRefWorld(n_err,6);
    graspTransposeRefWorld.block<6,6>(0,0) = graspRightRefWorld.transpose();
    graspTransposeRefWorld.block<6,6>(6,0) =  graspLeftRefWorld.transpose();

//     std::cout << "a7 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    // REBLOCK
    Eigen::MatrixXd jacobian(6*PEPPER_N_TCP, PEPPER_TOTAL_N_JOINTS_SIMP);
    jacobian.setZero();
    partJac = Eigen::MatrixXd::Zero(n_err,n_cfg);
    //   Right arm
    //     Torso
    partJac.block<6,6>(0,0) = jacobian.block<6,6>(6,0);
    //     Arm
    partJac.block<6,5>(0,6) = jacobian.block<6,5>(6,8);
    //   Left arm
    //     Torso
    partJac.block<6,6>(6,0) = jacobian.block<6,6>(12,0);
    //     Arm
    partJac.block<6,5>(6,11) = jacobian.block<6,5>(12,13);
    
//     std::cout << "a8 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    unsigned int n_obj = 3;	// Only command by object position
    Eigen::MatrixXd graspJac(n_obj,n_cfg);
    graspJac = Eigen::MatrixXd::Zero(n_obj,n_cfg);
    graspJac = ( pinv(graspTransposeRefWorld,0.001) * partJac ).block<3,16>(0,0);
    
//     std::cout << "a9 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    pInvPartJac.resize(n_cfg, n_obj);
    pInvPartJac = pinv( graspJac , 0.001);
//     std::cout << "a10 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
     
    // ERROR
    KDL::Frame KDL_object_frame;
    fk_solvers[1]->JntToCart(config_chains[1], KDL_object_frame);
    Eigen::Vector3d rightarm_position;
    rightarm_position(0) = KDL_object_frame(0,3);
    rightarm_position(1) = KDL_object_frame(1,3);
    rightarm_position(2) = KDL_object_frame(2,3);
    Eigen::Vector3d rightarm_x_axis;
    rightarm_x_axis(0) = KDL_object_frame(0,0);
    rightarm_x_axis(1) = KDL_object_frame(1,0);
    rightarm_x_axis(2) = KDL_object_frame(2,0);
    Eigen::Vector3d rightarm_z_axis; 
    rightarm_z_axis(0) = KDL_object_frame(0,2);
    rightarm_z_axis(1) = KDL_object_frame(1,2);
    rightarm_z_axis(2) = KDL_object_frame(2,2);
    Eigen::Vector3d object_position;
    object_position = rightarm_position + 0.125 * rightarm_x_axis - 0.17 * rightarm_z_axis;
//     std::cout << "a11 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    //       Position error
    Eigen::Vector3d object_position_error(desired_poses[7] - object_position(0),  desired_poses[8] - object_position(1), desired_poses[9] - object_position(2));
    //       Orientation error
    Eigen::Quaternion<double> object_desired_quat;
    object_desired_quat.x() = desired_poses[10];
    object_desired_quat.y() = desired_poses[11];
    object_desired_quat.z() = desired_poses[12];
    object_desired_quat.w() = desired_poses[13];
    Eigen::Vector3d object_orientation_error = sicil_or_error(object_desired_quat, KDL_object_frame);    
    
//     std::cout << "a12 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    Eigen::Vector3d object_alg_error_p = Kp * object_position_error;
    Eigen::Vector3d object_alg_error_w = Kp * object_orientation_error;
    Eigen::VectorXd object_alg_error(6);
    object_alg_error << object_alg_error_p(0), object_alg_error_p(1), object_alg_error_p(2), object_alg_error_w(0), object_alg_error_w(1), object_alg_error_w(2);
    
//     std::cout << "a13 ";	// AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa
    partErr = Eigen::VectorXd::Zero(n_obj);
    for (unsigned int i=0; i<n_obj; i++)		partErr(i) = object_alg_error(i);
    
    
    partCfg = pInvPartJac * partErr;
    
    // REBLOCK
    for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)		configuration_inc(i) = 0.0;
    //   Torso
    for (unsigned int i=0; i<6; i++)			configuration_inc(i) = partCfg(i);
    //   Right arm
    for (unsigned int i=8; i<13; i++)			configuration_inc(i) = partCfg(i-2);
    //   Left arm
    for (unsigned int i=13; i<18; i++)			configuration_inc(i) = partCfg(i-2);
    
    // Weighted jacobian
  //   configuration_inc = invSqrtmD(W) * configuration_inc;
    
    saturate_joint_velocity(configuration_inc, period);
    
    //   Check configuration acceleration limits
  //   if (n_it == 1)	past_configuration_inc = configuration_inc;  
  //   saturate_joint_acceleration(configuration_inc, past_configuration_inc, period);
  //   past_configuration_inc = configuration_inc;
  
    //   Compute new configuration
    for (unsigned int i=0; i<PEPPER_PLATFORM_N_DOF + PEPPER_N_JOINTS; i++)    current_configuration[i] = current_configuration[i] + configuration_inc(i);
    
    //   Saturate joint values
    saturate_joint_angles(current_configuration);
  }
  // GRASP TEST ---------------------------------------------------------------------------------------------------------------------------------------------------------

  

  return;
}




// OLD CODE BACKUP ************************************************************************************************************************************************************************


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ERROR COMPUTATION

//   // ****************************************************************************************************************************************************************************************
//   // ERROR VECTOR ***************************************************************************************************************************************************************************
//   double Kp = 1.5;
//   double Kv = 0.0;
//   
//   //   Compute errors ----------------------------------------------------------------------------------------------------------------------------
//   Eigen::Vector3d* alg_error_p = new Eigen::Vector3d[PEPPER_N_TCP];
//   Eigen::Vector3d* alg_error_w = new Eigen::Vector3d[PEPPER_N_TCP];  
//   for (unsigned int n_tcp = 0; n_tcp < PEPPER_N_TCP; n_tcp++)
//   {
//     double curr_pose[7], past_pose[7];
//     for (unsigned int i=0; i<7; i++)
//     {
// 	curr_pose[i] = desired_poses[n_tcp*7 + i];
// 	past_pose[i] = past_desired_poses[n_tcp*7 + i];
//     }    
//     
//     //     Error
//     Eigen::Vector3d tcp_position_error( curr_pose[0] - KDL_TCPs_frames[n_tcp](0,3), curr_pose[1] - KDL_TCPs_frames[n_tcp](1,3), curr_pose[2] - KDL_TCPs_frames[n_tcp](2,3) );
//     Eigen::Vector3d tcp_orientation_error = sicil_or_error(Eigen::Quaternion<double>(curr_pose[6], curr_pose[3], curr_pose[4], curr_pose[5]), KDL_TCPs_frames[n_tcp]);
//     
// //     //     Velocity 
// //     Eigen::VectorXd tcp_desired_velocity_aux = compute_pose_velocity(curr_pose, past_pose, period, Kv);
// //     Eigen::Vector3d tcp_desired_linear_velocity( tcp_desired_velocity_aux(0), tcp_desired_velocity_aux(1), tcp_desired_velocity_aux(2) );
// //     Eigen::Vector3d tcp_desired_angular_velocity( tcp_desired_velocity_aux(3), tcp_desired_velocity_aux(4), tcp_desired_velocity_aux(5) );
// 
// //     //     Compute errors with L
// //     Eigen::Matrix3d Lm = Lmat( KDL_TCPs_frames[n_tcp], Eigen::Quaternion<double>(curr_pose[6], curr_pose[3], curr_pose[4], curr_pose[5]) );
// //     alg_error_p[n_tcp] =                                   tcp_desired_linear_velocity  + Kp * tcp_position_error;
// //     alg_error_w[n_tcp] = Lm.inverse() * ( Lm.transpose() * tcp_desired_angular_velocity + Kp * tcp_orientation_error );
// 
//     //     Compute errors without L 
// //     alg_error_p[n_tcp] = tcp_desired_linear_velocity  + Kp * tcp_position_error;
// //     alg_error_w[n_tcp] = tcp_desired_angular_velocity + Kp * tcp_orientation_error;
//     
//     //     Only position error 
//     alg_error_p[n_tcp] = Kp * tcp_position_error;
//     alg_error_w[n_tcp] = Kp * tcp_orientation_error;
//   }
//   
//   //     Unify position & orientation errors
//   Eigen::VectorXd pose_error(6*PEPPER_N_CHAINS);
//   pose_error.block<6,1>(0,0)  <<   alg_error_p[0](0), alg_error_p[0](1), alg_error_p[0](2),   alg_error_w[0](0), alg_error_w[0](1), alg_error_w[0](2);
//   pose_error.block<6,1>(6,0)  <<   alg_error_p[1](0), alg_error_p[1](1), alg_error_p[1](2),   alg_error_w[1](0), alg_error_w[1](1), alg_error_w[1](2);
//   pose_error.block<6,1>(12,0) <<   alg_error_p[2](0), alg_error_p[2](1), alg_error_p[2](2),   alg_error_w[2](0), alg_error_w[2](1), alg_error_w[2](2); 
//   if (n_it == 1)	pose_error.setZero();



// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// WEIGHTED JACOBIAN

// Weighted Jacobian ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//   Eigen::MatrixXd W = Eigen::MatrixXd::Identity(PEPPER_TOTAL_N_JOINTS_SIMP,PEPPER_TOTAL_N_JOINTS_SIMP);
//   for (unsigned int i=3; i<PEPPER_TOTAL_N_JOINTS_SIMP; i++)
//   {	
//     double diffH = fabs( pow( (pepper_joint_limits_max[i] - pepper_joint_limits_min[i])/(( pepper_joint_limits_max[i] - cfg(i) )*( cfg(i) - pepper_joint_limits_min[i] )) ,2) *
//       ( 2*cfg(i) - pepper_joint_limits_max[i] - pepper_joint_limits_min[i] ) );
// //     past_diffH[i] = diffH;    
// //     
// //     double incH;
// //     if (n_it == 1)	incH = 0.0;
// //     else		incH = diffH - past_diffH[i];
// //     
// //     if (incH >= 0)	W(i,i) = 1.0 + diffH;
// //     else		W(i,i) = 1.0;
//     W(i,i) = 1.0 + diffH;
//     W(i,i) = 1.0;
//   }
//   jacobian = jacobian * invSqrtmD(W);
// 
// 
// Weighted jacobian ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//   configuration_inc = invSqrtmD(W) * configuration_inc;