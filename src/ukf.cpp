#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.0175;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.1;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  n_z_ = 3;
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1); 
  Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);
  
  Zsig_ = MatrixXd(n_z_, 2*n_aug_+1);
  z_pred_ = VectorXd(n_z_);
  S_ = MatrixXd(n_z_, n_z_);
  
  InitWeights();
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!is_initialized_)
  {
	if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      if(!use_radar_)
	    return;
	  float rho = meas_package.raw_measurements_[0];
	  float phi = meas_package.raw_measurements_[1];
	  float rho_dot = meas_package.raw_measurements_[2];
	  x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
    }else{
	  if(!use_laser_)
        return;		  
      x_ << meas_package.raw_measurements_[0], 
	            meas_package.raw_measurements_[1], 0, 0, 0;
    }   
	
	is_initialized_ = true;
	time_us_ = meas_package.timestamp_;
    return;  
  }
  // check if data is from lidar or radar
  
  double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
  
  
  
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
	if(use_radar_){
        Prediction(delta_t); 		
		UpdateRadar(meas_package);	
		time_us_ = meas_package.timestamp_;
	}		
  }else{
	if(use_laser_)
	  Prediction(delta_t); 	
      UpdateLidar(meas_package);
	  time_us_ = meas_package.timestamp_;	
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  AugmentedSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  PredictRadarMeasurement();
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.fill(0.0);

  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

    while(z_diff(1) > M_PI) {z_diff(1) -=2. * M_PI;}
    while(z_diff(1) < -M_PI) {z_diff(1) +=2. * M_PI;}

	VectorXd x_diff = Xsig_pred_.col(i) - x_;

    while(x_diff(1) > M_PI) {
		x_diff(1) -=2. * M_PI;
	}
    while(x_diff(1) < -M_PI) {
		x_diff(1) += 2. * M_PI;
	}
	
    Tc += weights_(i) * x_diff * z_diff.transpose();	
  }
  
  MatrixXd K = Tc * S_.inverse();
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred_;

  while(z_diff(1) > M_PI) z_diff(1) -=2. * M_PI;
  while(z_diff(1) < -M_PI) z_diff(1) +=2. * M_PI;
  
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_ * K.transpose();
}

/**
  * Generate Sigma Points 
  * @param {MatrixXd} Xsig_out
*/
void UKF::AugmentedSigmaPoints(){
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  
  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  
  double sqrt_lambda_n_aug = sqrt(lambda_ + n_aug_); 
  MatrixXd lambda_n_aug_p = sqrt_lambda_n_aug * L;
  
  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug_.col(i+1) = x_aug + lambda_n_aug_p.col(i);
	Xsig_aug_.col(i+1+n_aug_) = x_aug - lambda_n_aug_p.col(i);
  }
}

/**
  * Generate Sigma Points 
  * @param {MatrixXd, Matrix, double} Xsig_out, Xsig_in, delta_t
*/
void UKF::SigmaPointPrediction(double delta_t){
  //create matrix with predicted sigma points as columns
  
  double delta_t2 = delta_t * delta_t;
  for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0, i);
    double p_y = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double yaw = Xsig_aug_(3, i);
	double yawd = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);
    //predicted state values
    double px_p, py_p;	
	//avoid divsion by zero
	if(fabs(yawd) > 0.001){
	  px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
	  py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
	}else{
	  px_p = p_x + v*delta_t*cos(yaw);
	  py_p = p_y + v*delta_t*sin(yaw);
	}
	double v_p = v;
	double yaw_p = yaw + yawd*delta_t;
	double yawd_p = yawd;
	
	//add noise
	px_p = px_p + 0.5 * nu_a * delta_t2 * cos(yaw);
	py_p = py_p + 0.5 * nu_a * delta_t2 * sin(yaw);
	v_p = v_p + nu_a * delta_t;
	
	yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t2;
	yawd_p = yawd_p + nu_yawdd * delta_t;
	
	//write predicted sigma point into right column
	Xsig_pred_(0,i) = px_p;
	Xsig_pred_(1,i) = py_p;
	Xsig_pred_(2,i) = v_p;
	Xsig_pred_(3,i) = yaw_p;
	Xsig_pred_(4,i) = yawd_p;
  }
}

  /**
    *PredictMeanAndCovariance 
  */
void UKF::PredictMeanAndCovariance()
{ 
  //predict state mean
  x_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  //predicted state covariance matrix
  P_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while(x_diff(3) > M_PI) {x_diff(3) -= 2. * M_PI;}
    while(x_diff(3) < -M_PI) {x_diff(3) += 2. * M_PI;}
	P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}
  /**
    *PredictMeanAndCovariance 
	*@param {VectorXd, MatrixXd} z_out, S_out
  */
void UKF::PredictRadarMeasurement() { 
  for(int i = 0; i < 2*n_aug_+1; ++i)
  {
    double p_x = Xsig_pred_(0,i);
	double p_y = Xsig_pred_(1,i);
	double v = Xsig_pred_(2, i);
	double yaw = Xsig_pred_(3,i);
	
	double v1 = cos(yaw)*v;
	double v2 = sin(yaw)*v;
	
	// measurement model
	Zsig_(0,i) = sqrt(p_x*p_x + p_y*p_y);
	Zsig_(1,i) = atan2(p_y, p_x);
	Zsig_(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);
  }
  
  z_pred_.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++)
  {
    z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
  }
 
  S_.fill(0.0);
  for(int i = 0; i < 2*n_aug_ +1; i++)
  {
    VectorXd z_diff = Zsig_.col(i) - z_pred_;

	while(z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
	while(z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
	
	S_ += weights_(i) * z_diff * z_diff.transpose();
  }
  
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R << std_radr_*std_radr_, 0, 0,
           0, std_radphi_*std_radphi_, 0,
		   0, 0, std_radrd_*std_radrd_;
		   
  S_ += R;
  }
  
void UKF::InitWeights()
{
  // set weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  double denorminator = lambda_ + n_aug_;
  double weight_0 = lambda_/denorminator;
  weights_(0) = weight_0;
  for(int i = 1; i < 2 * n_aug_ + 1; ++i)
  {
    weights_(i) = 0.5/denorminator;  	
  }
}

