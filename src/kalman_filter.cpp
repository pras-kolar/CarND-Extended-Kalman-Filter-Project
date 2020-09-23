#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

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
  x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;  
}

VectorXd CartesianToPolarCalculation(const VectorXd &x_){

  float px,py,vx,vy;
  px = x_[0];
  py = x_[1];
  vx = x_[2];
  vy = x_[3];
  
  double rho;
  double phi;
  double rho_dot;
  
  rho = sqrt(px*px + py*py);
  phi = atan2(py, px);
  // To eliminate DIV by zero during rho_dot the derivative, 
  // assign minimum value to rho
  if(rho < 0.000001)
  {
    rho = 0.000001;
  }
  
  rho_dot = (px*vx + py*vy)/rho;
  
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  
  return z_pred;

  
}

// Standard Kalman Filter
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  MatrixXd Ht = H_.transpose();
  MatrixXd P_Ht = P_ * Ht;
  MatrixXd S = H_ * P_Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_Ht * Si;
  
  VectorXd y = z - H_ * x_;
  
  // New state values are
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  /*
  * Radar measurements conversion from (x,y,vx,vy) - cartesian 
  * to (rho,phi,rho_dot) - polar coordinates
  */
  
  VectorXd z_p = CartesianToPolarCalculation(x_);   // For the Prediction values
  
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  
  VectorXd y = z - z_p;
  
  double rho = sqrt(px*px + py*py);
  double theta = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;
  VectorXd h = VectorXd(3);
  
  h << rho, theta, rho_dot;
  cout << "H : " << h << endl;
  
 // the values in EKF tend to be between PI and -PI hence normalize them
 // for effectveness
  
  while ( M_PI < y(1)){
    y(1) -= 2*M_PI;
  }
  
  while ( -M_PI > y(1)){
    y(1) += 2*M_PI;
  }
  
  // Kalman Gain calculations
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  
  // State update
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}