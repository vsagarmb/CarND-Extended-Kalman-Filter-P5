#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
   * TODO: predict the state
   */

  x_ = F_ * x_ ;                    // There is no external motion, so, we do not have to add u
  
  MatrixXd Ft = F_.transpose();
  
  P_ = F_ * P_ * Ft + Q_;  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */  
  
  VectorXd y = z - H_ * x_; // error calculation
  
  KF_Common(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  // Recalculate x object state to rho, theta, rho_dot coordinates
  double rho = sqrt(px*px + py*py);
  double theta = atan2(py, px);
  double rho_dot;
  
  if (fabs(rho) < 0.0001) {  
    rho_dot = 0;
  }
  else {
    rho_dot = (px*vx + py*vy) / rho;
  }
  
  VectorXd h = VectorXd(3); // h(x_)
  h << rho, theta, rho_dot;
  
  VectorXd y = z - h;
  
  // Calculations are essentially the same to the Update function
  KF_Common(y);  
}

// Common update equations between Kalman Filter and Extended Kalman Filter update steps.
void KalmanFilter::KF_Common(const VectorXd &y){

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // New state
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;
}