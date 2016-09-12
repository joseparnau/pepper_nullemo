#ifndef PEPPER_IK_ALGS_HPP
#define PEPPER_IK_ALGS_HPP


#include <pepper_core.hpp>

#include <pepper_nullemo_aux.hpp>

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
 			      );

#endif