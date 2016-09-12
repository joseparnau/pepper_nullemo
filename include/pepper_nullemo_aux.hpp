// ----------------------------------------------------------------------------------------------------------------------------
//
// Header file
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


#ifndef PEPPER_NULLEMO_AUX_HPP
#define PEPPER_NULLEMO_AUX_HPP


#include <sys/time.h> 
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>



class statAccount {
    unsigned int n_it;
    double average;
    double stdev;
    double last_value;
  public:
    statAccount(void){
      n_it = 0;
      average = 0.0;
      stdev = 0.0;
    }
    void computeNextStatistics(double value){
      last_value = value;
      n_it++;
      average = (((double)n_it-1.0)/n_it)*average + (1.0/(double)n_it)*value;
      if (n_it == 1)	stdev = 0.0;
      else		stdev = sqrt( (((double)n_it-1.0)/n_it)*stdev*stdev + (1.0/((double)n_it-1.0))*(value - average)*(value - average) );
    }
    double get_last_value(void){	return last_value;	};
    double get_average(void){		return average;		};
    double get_stdev(void){		return stdev;		};
};

timeval past_time_, current_time_;
double time_increment = 0.0;
void update_time_increment(void);

double GetTimeMs64();

double rnd0to1();

Eigen::Vector3d sicil_or_error(const Eigen::Quaternion<double> desired_quaternion, const KDL::Frame KDL_frame);
Eigen::Vector3d sicil_or_error(const Eigen::Quaternion<double> desired_quaternion, const Eigen::Quaternion<double> current_quaternion);

Eigen::VectorXd compute_pose_velocity(const double* current_pose, const double* past_pose, const double time_increment_, const double Kv);

Eigen::MatrixXd Lmat(const KDL::Frame KDL_current_rotation, const Eigen::Quaternion<double> rot_desired);
Eigen::MatrixXd Lmat(const Eigen::Quaternion<double> rot_current, const Eigen::Quaternion<double> rot_desired);

Eigen::Matrix3d cross_product_matrix_S(const Eigen::Vector3d vector);

Eigen::MatrixXd pinv(const Eigen::MatrixXd mat, const double pinv_tolerance);
Eigen::MatrixXd pinv(const Eigen::MatrixXd mat, const double pinv_tolerance, Eigen::VectorXd &singular_values);

Eigen::MatrixXd sqrtmD(const Eigen::MatrixXd mat );	// Square root of diagonal square matrix

Eigen::MatrixXd invSqrtmD( Eigen::MatrixXd mat );	// Insverse of the square root of diagonal square matrix



#endif