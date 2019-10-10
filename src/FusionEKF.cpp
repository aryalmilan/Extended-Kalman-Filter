#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 10,  0,   0,   0,
             0,   10,  0,   0,
             0,   0,   100, 0,
             0,   0,   0,   100;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //initialize the measurements
      double rho     = measurement_pack.raw_measurements_[0]; // range
      double phi     = measurement_pack.raw_measurements_[1]; // bearing
      double rho_dot = measurement_pack.raw_measurements_[2]; // rate

      

      // make sure phi is between [-pi, pi]
      while (phi > M_PI)  phi -= 2.0 * M_PI;
      while (phi < -M_PI) phi += 2.0 * M_PI;

      // Polar coordinate to rectangular
      double x  = rho * cos(phi);
      double y  = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);

      //store in state vector
      ekf_.x_ << x, y, vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //measurements 
      double x = measurement_pack.raw_measurements_[0];
      double y = measurement_pack.raw_measurements_[1];
      ekf_.x_ << x, y, 0, 0;

    }

    //avoid any divide by zero 
    if (fabs(ekf_.x_(0)) < 0.0001 and fabs(ekf_.x_(1)) < 0.001){
      ekf_.x_(0) = 0.0001;
      ekf_.x_(1) = 0.0001;
    }
      previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  //time between measurements
  double dt = (measurement_pack.timestamp_ - previous_timestamp_);
  dt /= 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1,  0,  dt, 0,
             0,  1,  0,  dt,
             0,  0,  1,  0,
             0,  0,  0,  1;

// Noise Covatiance for matrix Q
  double noise_ax = 9.0;
  double noise_ay = 9.0;

  // Pre-calculate some variables for the matrix
  double dt2_ = dt * dt;
  double dt3_ = dt2_ * dt;
  double dt4_ = dt3_ * dt;
  double dt4_4_ = dt4_ / 4;
  double dt3_2_ = dt3_ / 2;

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt4_4_* noise_ax,                  0,  dt3_2_ * noise_ax,                 0,
                            0,  dt4_4_ * noise_ay,                  0, dt3_2_ * noise_ay,
            dt3_2_ * noise_ax,                  0,    dt2_ * noise_ax,                 0,
                             0,   dt3_2_*noise_ay,                  0,     dt2_*noise_ay; 
  ekf_.Predict();

  //Update based on sensor EKF-> RADAR, KF -> LASER

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    //Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
