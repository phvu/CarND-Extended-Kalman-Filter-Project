#include "kalman_filter.h"

#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in) {
  VectorXd y = z - H_in * x_;
  Update_(y, H_in, R_in);
}

void KalmanFilter::UpdateEKF(const VectorXd &z, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in) {
  Tools tools;

  VectorXd y = z - tools.cartesianToPolar(x_);
  y(1) = tools.normalizeAngle(y(1));
  Update_(y, H_in, R_in);
}

void KalmanFilter::Update_(const Eigen::VectorXd &y, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in) {
  MatrixXd Ht = H_in.transpose();
  MatrixXd S = H_in * P_ * Ht + R_in;
  MatrixXd K = P_ * Ht * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  x_ = x_ + (K * y);
  P_ = (I - K * H_in) * P_;
}


