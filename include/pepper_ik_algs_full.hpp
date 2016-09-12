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


#ifndef PEPPER_IK_ALGS_HPP
#define PEPPER_IK_ALGS_HPP


#include <pepper_core.hpp>
#include <pepper_emotional.hpp>

#include <pepper_nullemo_aux.hpp>

void compute_next_configuration(double* current_configuration, 
				unsigned int* dimension_selection, double* dimension_selection_frames,
				Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, KDL::Jacobian* KDL_jacobians,
				double Km, double* emotion_configuration, double* past_emotion_configuration,
				const unsigned int priority_distribution_id,
				const unsigned int n_it, const double period,
				Eigen::VectorXd &configuration_inc,
				const bool gather_task_error,
				double* desired_poses, 
				KDL::ChainFkSolverPos_recursive** fk_solvers,
				Eigen::MatrixXd &task_priority_errors, Eigen::MatrixXd & task_priority_norm_errors, Eigen::MatrixXd& pose_errors_norm_errors, double *emotion_errors,
				Eigen::MatrixXd &singular_values,
				double* next_configuration,
				const unsigned int ik_tp_alg_formulation
			       );

void compute_next_configuration_siciliano_taskpriority(double* current_configuration, 
						       unsigned int* dimension_selection, double* dimension_selection_frames,
						       Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, KDL::Jacobian* KDL_jacobians,
						       double Km, double* emotion_configuration, double* past_emotion_configuration,
						       const unsigned int priority_distribution_id,
						       const unsigned int n_it,
						       Eigen::VectorXd &configuration_inc,
						       const bool gather_task_error,
						       Eigen::MatrixXd &task_priority_errors, Eigen::MatrixXd& task_priority_norm_errors, Eigen::MatrixXd& pose_errors_norm_errors, double *emotion_errors,
						       Eigen::MatrixXd &singular_values
						      );

void compute_next_configuration_siciliano_recursive_taskpriority(double* current_configuration, 
								 unsigned int* dimension_selection, double* dimension_selection_frames,
								 Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, KDL::Jacobian* KDL_jacobians,
								 double Km, double* emotion_configuration, double* past_emotion_configuration,
								 const unsigned int priority_distribution_id,
								 const unsigned int n_it,
								 Eigen::VectorXd &configuration_inc,
								 const bool gather_task_error,
								 Eigen::MatrixXd &task_priority_errors, Eigen::MatrixXd& task_priority_norm_errors, Eigen::MatrixXd& pose_errors_norm_errors, double *emotion_errors,
								 Eigen::MatrixXd &singular_values
								);

void compute_next_configuration_chiaverini_taskpriority(double* current_configuration, 
							unsigned int* dimension_selection, double* dimension_selection_frames,
							Eigen::Vector3d* alg_error_p, Eigen::Vector3d* alg_error_w, KDL::Jacobian* KDL_jacobians,
							double Km, double* emotion_configuration, double* past_emotion_configuration,
							const unsigned int priority_distribution_id,
							const unsigned int n_it,
							Eigen::VectorXd &configuration_inc,
							const bool gather_task_error,
							Eigen::MatrixXd &task_priority_errors, Eigen::MatrixXd& task_priority_norm_errors, Eigen::MatrixXd& pose_errors_norm_errors, double *emotion_errors,
							Eigen::MatrixXd &singular_values
						       );

#endif