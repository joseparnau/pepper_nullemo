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


#ifndef PEPPER_EMOTIONAL_HPP
#define PEPPER_EMOTIONAL_HPP


#include <pepper_core.hpp>
#include <pepper_nullemo_aux.hpp>


struct emotionalInitData {
  
  // Extension
  double w_estension;
  double cont_cfg[PEPPER_TOTAL_N_JOINTS_SIMP];
  double ext_cfg[PEPPER_TOTAL_N_JOINTS_SIMP];
  double KAmp[PEPPER_TOTAL_N_JOINTS_SIMP];
  
  // Jerkiness
  double jerky_w_sin;
  double jerky_w_cos;
  double jerky_A_sin;
  double jerky_A_cos;
  
  // Symmetry
  double sym_A_min;
  double sym_A_span;
  double sym_T_min;
  double sym_T_span;
  
  // Hands
  double hand_jnt_val_min;
  double hand_jnt_val_span;
};


void initialize_emotional_data(emotionalInitData *emo_data);

void compute_emotional_configuration(double* current_configuration, double* current_poses,
				     KDL::ChainFkSolverPos_recursive** fk_solvers, 
				     double* emotion_configuration, double* desired_poses, double &righthand_jnt_val, double &lefthand_jnt_val, double *desired_gazed_position, 
				     const double period, const double alg_time,
				     const double extension, const double jerkiness, const double symmetry, const double dominance,
				     double* sym_sin, double* sym_sin_past, double* sym_A, double* sym_w
				    );

#endif	// PEPPER_EMOTIONAL_INIT_HPP