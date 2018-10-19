#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */
  /*
   * For this project, however, we do not need to use the f function or Fj.
   * If we had been using a non-linear model in the prediction step, we would need to replace the F matrix with its Jacobian, Fj.
   * However, we are using a linear model for the prediction step. So, for the prediction step, we can still use the regular
   * Kalman filter equations and the F matrix rather than the extended Kalman filter equations.
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  Eigen::VectorXd y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  Eigen::MatrixXd temp = K * H_;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(temp.rows(),temp.cols());
  P_ = (I - temp) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Conversion cartesian to polar
  Eigen::VectorXd prediction(3);
  prediction(0) = std::sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  prediction(1) = std::atan2(x_(1)/x_(0));
  prediction(2) = (x_(0)*x_(2)+x_(1)*x_(3))/prediction(0);

  Eigen::VectorXd y = z - prediction;
  Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  Eigen::MatrixXd temp = K * H_;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(temp.rows(),temp.cols());
  P_ = (I - temp) * P_;
}
