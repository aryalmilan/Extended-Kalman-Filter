#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * Prdict the state and covariance 
   */
  x_ = F_ * x_;
  MatrixXd FT_ = F_.transpose();
  P_ = F_ * P_ * FT_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * Update the state by using Kalman Filter equations and measurements
   */
  VectorXd y_hat_ = z - H_ * x_;
  FilterUpdates(y_hat_);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * Update the state by using Extended Kalman Filter equations
   */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho_hat_ = sqrt(px * px + py * py);
  double theta_hat_ = atan2(py, px);
  double rho_dot_hat_;

  if (fabs(rho_hat_)<0.0001){
    rho_dot_hat_ = 0;
  }else{
   rho_dot_hat_= (px * vx + py * vy)/rho_hat_;
  }

  VectorXd h = VectorXd(3);
  h << rho_hat_, theta_hat_, rho_dot_hat_;
  VectorXd y_hat_ = z - h;
  
}

void KalmanFilter::FilterUpdates(const VectorXd &y){
  MatrixXd HT_ = H_.transpose();
  MatrixXd S_ = H_ * P_ * HT_ + R_;
  MatrixXd S_inv_ = S_.inverse();
  MatrixXd K_  = P_ * HT_ * S_inv_;

  //Latest states
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + (K_ * y);
  P_ = (I - K_ * H_) * P_ ;

}
