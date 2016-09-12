#ifndef FILTRJAC_HPP
#define FILTRJAC_HPP


using namespace Eigen;
using namespace std;


class filtrJac {
  
private:

  unsigned int filter_type_;
  // 0 - low pass
  // 1 - moving average
  // 2 - recursive moving average  

  // time
  timeval past_time_, current_time_;
  double time_increment_;  
  
  // low pass
  double cut_frequency_;
  unsigned int n_;
  double alpha_;
  
  // moving average & recursive moving average
  unsigned int mean_window_size_;
  unsigned int n_recursive_layers_;
  filtrJac *movingAverageFilters;
  
  bool first_input_data_read_;
  unsigned int n_iteration_;
  
  unsigned int x_dim_;
  MatrixXd historic_x_, historic_y_;
  VectorXd new_y_;
  
  
  void init_filter(void);
  
  void store_data_(const VectorXd data, MatrixXd &registry);
  
  VectorXd compute_new_y(void);
  VectorXd y_low_pass(void);  
  VectorXd y_moving_average(void);
  VectorXd y_recursive_moving_average(void);
    
  double compute_time_increment(void){
    past_time_ = current_time_;
    gettimeofday(&current_time_, NULL);
    time_increment_ = ( (current_time_.tv_sec*1e6 + current_time_.tv_usec) - (past_time_.tv_sec*1e6 + past_time_.tv_usec) ) / 1e6;
    return time_increment_;
  }
public:
  
  // Constructors
  filtrJac(){	filtrJac(0, 10.0);	};
  filtrJac(const unsigned int filter_type, const double cut_frequency);
  filtrJac(const unsigned int filter_type, const unsigned int mean_window_size);
  filtrJac(const unsigned int filter_type, const unsigned int mean_window_size, unsigned int recursive_layers);
  
  double alpha(void){	return alpha_;	};
  
  VectorXd filter(const VectorXd x);
  
  MatrixXd historic(){	return historic_x_;	};
  
  VectorXd mean(const MatrixXd data);
  double mean(const VectorXd data);
};



#endif