// ----------------------------------------------------------------------------------------------------------------------------
//
// Auxiliar funcions
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



#include <pepper_nullemo_aux.hpp>



void update_time_increment(void){
  past_time_ = current_time_;
  gettimeofday(&current_time_, NULL);
  time_increment = ( (current_time_.tv_sec*1e6 + current_time_.tv_usec) - (past_time_.tv_sec*1e6 + past_time_.tv_usec) ) / 1e6;
}

/* Returns the amount of milliseconds elapsed since the UNIX epoch. Works on both
 * windows and linux. */
double GetTimeMs64()
{
 /* Linux */
 struct timeval tv;

 gettimeofday(&tv, NULL);

 double ret = (double)tv.tv_usec;
 /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
 ret /= 1000.0;

 /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
 return (ret + (tv.tv_sec * 1000.0));
}


double rnd0to1()
{
  return ((double)rand() / (double)(RAND_MAX));
}


Eigen::Vector3d sicil_or_error(const Eigen::Quaternion<double> desired_quaternion, const KDL::Frame KDL_current_rotation){
  
  return ( 0.5 * ( cross_product_matrix_S(Eigen::Vector3d( KDL_current_rotation(0,0), KDL_current_rotation(1,0), KDL_current_rotation(2,0) )) * desired_quaternion.toRotationMatrix().col(0) + 
		   cross_product_matrix_S(Eigen::Vector3d( KDL_current_rotation(0,1), KDL_current_rotation(1,1), KDL_current_rotation(2,1) )) * desired_quaternion.toRotationMatrix().col(1) + 
		   cross_product_matrix_S(Eigen::Vector3d( KDL_current_rotation(0,2), KDL_current_rotation(1,2), KDL_current_rotation(2,2) )) * desired_quaternion.toRotationMatrix().col(2) ) ).eval();
}

Eigen::Vector3d sicil_or_error(const Eigen::Quaternion<double> desired_quaternion, const Eigen::Quaternion<double> current_quaternion){
  
  return ( 0.5 * ( cross_product_matrix_S(current_quaternion.toRotationMatrix().col(0)) * desired_quaternion.toRotationMatrix().col(0) + 
		   cross_product_matrix_S(current_quaternion.toRotationMatrix().col(1)) * desired_quaternion.toRotationMatrix().col(1) + 
		   cross_product_matrix_S(current_quaternion.toRotationMatrix().col(2)) * desired_quaternion.toRotationMatrix().col(2) ) ).eval();
}


Eigen::VectorXd compute_pose_velocity(const double* current_pose, const double* past_pose, const double time_increment_, const double Kv){
  
  Eigen::VectorXd pose_vel(6);
  
  // Position
  for (unsigned int i=0; i<3; i++)	pose_vel(i) = Kv*(current_pose[i] - past_pose[i])/time_increment_;
  
  // Orientation
  Eigen::Quaternion<double> q_curr(current_pose[6], current_pose[3], current_pose[4], current_pose[5]);
  Eigen::Quaternion<double> q_past(past_pose[6], past_pose[3], past_pose[4], past_pose[5]);
  
  Eigen::AngleAxis<double> axis_ang( q_curr.toRotationMatrix() * q_past.toRotationMatrix().transpose() );
  pose_vel(3) = Kv*(axis_ang.angle()/time_increment_)*axis_ang.axis()(0);
  pose_vel(4) = Kv*(axis_ang.angle()/time_increment_)*axis_ang.axis()(1);
  pose_vel(5) = Kv*(axis_ang.angle()/time_increment_)*axis_ang.axis()(2);
  
  return pose_vel;
}


Eigen::MatrixXd Lmat(const KDL::Frame KDL_current_rotation, const Eigen::Quaternion<double> rot_desired){
  
  // Current  
  Eigen::MatrixXd Sne(3,3), Sse(3,3), Sae(3,3);
  Sne = cross_product_matrix_S(Eigen::Vector3d( KDL_current_rotation(0,0), KDL_current_rotation(1,0), KDL_current_rotation(2,0) ));
  Sse = cross_product_matrix_S(Eigen::Vector3d( KDL_current_rotation(0,1), KDL_current_rotation(1,1), KDL_current_rotation(2,1) ));
  Sae = cross_product_matrix_S(Eigen::Vector3d( KDL_current_rotation(0,2), KDL_current_rotation(1,2), KDL_current_rotation(2,2) ));
  
  // Desired
  Eigen::MatrixXd Snd(3,3), Ssd(3,3), Sad(3,3);
  Snd = cross_product_matrix_S(rot_desired.toRotationMatrix().col(0));
  Ssd = cross_product_matrix_S(rot_desired.toRotationMatrix().col(1));
  Sad = cross_product_matrix_S(rot_desired.toRotationMatrix().col(2));
  
  return ( -0.5 * ( Snd*Sne + Ssd*Sse + Sad*Sae ) ).eval();
}

Eigen::MatrixXd Lmat(const Eigen::Quaternion<double> rot_current, const Eigen::Quaternion<double> rot_desired){
  
  // Current  
  Eigen::MatrixXd Sne(3,3), Sse(3,3), Sae(3,3);
  Sne = cross_product_matrix_S(rot_current.toRotationMatrix().col(0));
  Sse = cross_product_matrix_S(rot_current.toRotationMatrix().col(1));
  Sae = cross_product_matrix_S(rot_current.toRotationMatrix().col(2));
  
  // Desired
  Eigen::MatrixXd Snd(3,3), Ssd(3,3), Sad(3,3);
  Snd = cross_product_matrix_S(rot_desired.toRotationMatrix().col(0));
  Ssd = cross_product_matrix_S(rot_desired.toRotationMatrix().col(1));
  Sad = cross_product_matrix_S(rot_desired.toRotationMatrix().col(2));
  
  return ( -0.5 * ( Snd*Sne + Ssd*Sse + Sad*Sae ) ).eval();
}


Eigen::Matrix3d cross_product_matrix_S(const Eigen::Vector3d vector){
  
    Eigen::Matrix3d rotmat_aux;
  
    for (unsigned int i=0; i<3; i++)	rotmat_aux(i,i) = 0;
    
    rotmat_aux(2,1) = vector(0);
    rotmat_aux(1,2) = -vector(0);    

    rotmat_aux(0,2) = vector(1);
    rotmat_aux(2,0) = -vector(1);
    
    rotmat_aux(1,0) = vector(2);
    rotmat_aux(0,1) = -vector(2);
    
    return rotmat_aux;
}


Eigen::MatrixXd pinv(const Eigen::MatrixXd mat, const double pinv_tolerance){
  // Function that computes the Moore-Penrose pseudoinverse of matrix 'mat' using the SVD decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
  
  unsigned int min_dim = std::min(mat.rows(), mat.cols());
  Eigen::MatrixXd pS = Eigen::MatrixXd::Zero(min_dim, min_dim);
  for (unsigned int n_sv = 0; n_sv < min_dim; n_sv++){
	  if (svd.singularValues()[n_sv] > pinv_tolerance)
		  pS(n_sv,n_sv) = 1/svd.singularValues()[n_sv]; 
  }
  
//   std::cout << "sing val:   ";
//   for (unsigned int n_sv = 0; n_sv < min_dim; n_sv++){
// 	  std::cout << "  " << svd.singularValues()[n_sv];
//   }
//   std::cout << std::endl;

  Eigen::MatrixXd p_inv = svd.matrixV() * pS * svd.matrixU().transpose();

  return p_inv;
}


Eigen::MatrixXd pinv(const Eigen::MatrixXd mat, const double pinv_tolerance, Eigen::VectorXd &singular_values){
  // Function that computes the Moore-Penrose pseudoinverse of matrix 'mat' using the SVD decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
  
  unsigned int min_dim = std::min(mat.rows(), mat.cols());
  Eigen::MatrixXd pS = Eigen::MatrixXd::Zero(min_dim, min_dim);
  for (unsigned int n_sv = 0; n_sv < min_dim; n_sv++){
    singular_values(n_sv) = svd.singularValues()[n_sv];
    if (singular_values(n_sv) > pinv_tolerance)	pS(n_sv,n_sv) = 1.0/svd.singularValues()[n_sv]; 
  }
  Eigen::MatrixXd p_inv = svd.matrixV() * pS * svd.matrixU().transpose();

  return p_inv;
}


Eigen::MatrixXd sqrtmD(const Eigen::MatrixXd mat )
{
  unsigned int nc = mat.rows();
  Eigen::MatrixXd sMat( nc, nc );
  sMat = mat;
  
  for (unsigned int i=0; i<nc; i++)	sMat(i,i) = sqrt(mat(i,i));
  
  return sMat;
}


Eigen::MatrixXd invSqrtmD( Eigen::MatrixXd mat )
{
  unsigned int nc = mat.rows();
  Eigen::MatrixXd iMat( nc, nc );
  iMat = mat;
  
  for (unsigned int i=0; i<nc; i++)	iMat(i,i) = 1.0/sqrt(mat(i,i));
  
  return iMat;
}