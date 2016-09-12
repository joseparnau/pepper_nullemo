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


#ifndef PEPPER_NULLEMO_TASKS_HPP
#define PEPPER_NULLEMO_TASKS_HPP


#include <pepper_core.hpp>
#include <pepper_nullemo_aux.hpp>
#include <pepper_trajectories_data.hpp>

void compute_next_task(double* current_poses,
		       double* pepper_poses, unsigned int* dimension_selection, double* dimension_selection_frames, double* gazed_position, 
		       const double trajectory_time, const unsigned int trajectory_id
		      );


#endif