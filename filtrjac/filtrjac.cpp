// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <sys/time.h>

#include <filtrjac.hpp>


#ifndef PI
#define PI 3.141592
#endif


using namespace Eigen;
using namespace std;


filtrJac::filtrJac(const unsigned int filter_type, const double cut_frequency){
  filter_type_ = filter_type;
  cut_frequency_ = cut_frequency;

  init_filter();
}


filtrJac::filtrJac(const unsigned int filter_type, const unsigned int mean_window_size){
  filter_type_ = filter_type;
  mean_window_size_ = mean_window_size;

  init_filter();  
}


filtrJac::filtrJac(const unsigned int filter_type, const unsigned int mean_window_size, unsigned int recursive_layers){
  filter_type_ = filter_type;
  mean_window_size_ = mean_window_size;
  n_recursive_layers_ = recursive_layers;

  init_filter();  
  
  movingAverageFilters = new filtrJac[n_recursive_layers_];
  for (unsigned int i=0; i<n_recursive_layers_; i++)		movingAverageFilters[i] = filtrJac((unsigned int)1, (unsigned int)mean_window_size);
}


void filtrJac::init_filter(void){
  switch (filter_type_)
  {
  case 0:	// Low pass filter
      n_ = 2;
      break;
  case 1:	// Moving average
      n_ = mean_window_size_;
      break;
  case 2:	// Recursive moving average
      n_ = mean_window_size_;
      break;      
  default:	// Default
      n_ = 2;
      break;
  }  
  
  first_input_data_read_ = false;
  n_iteration_ = 0;
  
  time_increment_ = 0;
  gettimeofday(&current_time_, NULL);   
}


VectorXd filtrJac::filter(const VectorXd x){
  if (!first_input_data_read_){
    x_dim_ = x.size();
    historic_x_.resize(n_,x_dim_);
    historic_y_.resize(n_,x_dim_);
    store_data_(Eigen::VectorXd::Zero(x_dim_), historic_y_);
    first_input_data_read_ = true;
  }  
  
  // Save/update historia data
  store_data_(x, historic_x_);

  compute_time_increment();
  
  // Compute y
  new_y_ = compute_new_y();
  store_data_(new_y_, historic_y_);
  
  n_iteration_++;
  
  return new_y_;
}


void filtrJac::store_data_(const VectorXd data, MatrixXd &registry){

  if (n_iteration_ < n_)	for (unsigned int row=n_iteration_; row>0; row--)		registry.row(row) = registry.row(row-1);
  else				for (unsigned int row=n_-1; row>0; row--)			registry.row(row) = registry.row(row-1);
  
  registry.row(0) = data;  
}


VectorXd filtrJac::compute_new_y(void){
  switch (filter_type_)
  {
  case 0:	// Low pass filter
      return y_low_pass();
  case 1:	// Moving average
      return y_moving_average();  
  case 2:	// Recursive moving average
      return y_recursive_moving_average(); 
  default:	// Default
      return y_low_pass();
  }  
}


VectorXd filtrJac::y_low_pass(void){
  
  double RC = 1/(2*PI*cut_frequency_);
  alpha_ = time_increment_/(time_increment_ + RC);
  
//   std::cout << "DATA:  f / T / RC  / alpha :   " << cut_frequency_ << " / " << time_increment_ << " / " << RC << " / " << alpha_ << std::endl;
  
  if (n_iteration_ = 0)		return historic_x_.row(0);
  else				return (alpha_*historic_x_.row(0) + (1-alpha_)*historic_y_.row(1));	// y[i] = a*x[i] + (1-a)*y[i-1]
}


VectorXd filtrJac::y_moving_average(void){
  
  // y(i) = ( SUM_{i-N+1}_to_{i} x(i) )/N;
  if (n_iteration_+1 < mean_window_size_){
    MatrixXd reduced_data(n_iteration_+1,x_dim_);
    for (unsigned int i=0; i<n_iteration_+1; i++)      reduced_data.row(i) = historic_x_.row(i);    
    return mean((MatrixXd)reduced_data);
  }
  else{
    return mean((MatrixXd)historic_x_);
  }
}


VectorXd filtrJac::y_recursive_moving_average(void){
  
  VectorXd x(x_dim_);
  
  x = historic_x_.row(0);
  for (unsigned int i=0; i<n_recursive_layers_; i++){
//     std::cout << "A " << n_recursive_layers_ << " : " << "   " <<  x << std::endl;
    x = movingAverageFilters[i].filter(x);  
  }    
  
  return x;
}


VectorXd filtrJac::mean(const MatrixXd data){

  unsigned int n_means = data.row(0).size();
  
  VectorXd meanVector(n_means);
  
  for (unsigned int i=0; i<n_means; i++)	meanVector(i) = mean((VectorXd)data.col(i));
  
//   std::cout << "mean data:  " << std::endl << data << std::endl << "   -  " << std::endl;  
  
  return meanVector;
}


double filtrJac::mean(const VectorXd data){

  unsigned int n_elements = data.size();
  
  
  double mean = 0.0;
  for (unsigned int i=0; i<n_elements; i++)	mean += data(i);
  
//   std::cout << "mean data:  " << data.transpose() << "   -  " << mean/n_elements << std::endl;
  
  
  return (mean/n_elements);
}