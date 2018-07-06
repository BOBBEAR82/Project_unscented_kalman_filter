#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // previous timestamp
  long long previous_timestamp_;
  
  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

	///* For Prediction
	// Augmented state vector
	VectorXd x_aug;
	
	// Augmented state covariance matrix
	MatrixXd P_aug;	
	
	// Augmented sigma points matrix
	MatrixXd Xsig_aug;	
	
	///* For Lidar update
	// Dimension of lidar measurement
	int n_z_Lidar;	
	
	// predicted sigma points in lidar measurement
	MatrixXd Zsig_Lidar;

	// Predicted state mean in lidar measurement
	VectorXd z_pred_Lidar;	
	
	// Lidar measurement covariance matrix
	MatrixXd S_Lidar;
	
	// Lidar measurement noise covariance matrix
	MatrixXd R_Lidar;
	
	// Cross correlation matrix for lidar measurements
	MatrixXd Tc_Lidar;
	
	// Kalman gain for lidar measurements
	MatrixXd K_Lidar;
	
	// NIS of lidar estimation
	float NIS_Lidar;
	
	// NIS of lidar estimation log
	std::ofstream NIS_Lidar_log;
	
	///* For Radar update
	// Dimension of radar measurement
	int n_z_Radar;
	
	// predicted sigma points in radar measurement
	MatrixXd Zsig_Radar;
	
	// Predicted state mean in radar measurement
	VectorXd z_pred_Radar;
	
	// Radar measurement covariance matrix
	MatrixXd S_Radar;
	
	// Radar measurement noise covariance matrix
	MatrixXd R_Radar;
	
	// Cross correlation matrix for radar measurements
	MatrixXd Tc_Radar;
	
	// Kalman gain for radar measurements
	MatrixXd K_Radar;
	
	// NIS of radar estimation
	float NIS_Radar;
	
	// NIS of radar estimation log
	std::ofstream NIS_Radar_log;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* UKF_H */
