#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);
}

void KalmanFilter::Predict() {
  /*
  * KF Prediction step
  */
  x_ = F_ * x_; // + u;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /*
   * KF Measurement update step
   */
  VectorXd y = z - H_ * x_;
  _UpdateEKF(z, y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /*
   * KF Measurement update step
   */
  VectorXd y = z - _hEKF(x_);
  _UpdateEKF(z, y);
}

void KalmanFilter::_UpdateEKF(const VectorXd &z, const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}

VectorXd KalmanFilter::_hEKF(const VectorXd &x){
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  VectorXd ret = VectorXd(3);

  ret << sqrt(px*px + py*py),
         atan2(py, px),
         (px*vx + py*vy) / (sqrt(px*px+py*py));

  return ret;

}