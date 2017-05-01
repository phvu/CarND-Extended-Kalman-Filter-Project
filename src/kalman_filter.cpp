#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

double normalizeAngle(double angle) {
  double v = angle;
  while (v > M_PI) {
    v -= 2 * M_PI;
  }
  while (v < -M_PI) {
    v += 2 * M_PI;
  }
  return v;
}

VectorXd cartesianToPolar(const VectorXd &cartesian) {
  double px = cartesian[0];
  double py = cartesian[1];
  double vx = cartesian[2];
  double vy = cartesian[3];

  double rho = sqrt(px * px + py * py);
  double phi = 0;
  double rho_dot = 0;

  // avoid division by zero
  if (fabs(px) < 0.001) {
    cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << endl;
    px = 0.001;
  }
  phi = normalizeAngle(atan2(py, px));

  // avoid division by zero
  if (rho < 0.0001) {
    cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << endl;
    rho = 0.001;
  }
  rho_dot = (px * vx + py * vy) / rho;

  VectorXd polar(3);
  polar << rho, phi, rho_dot;
  return polar;
}


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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  Update_(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd y = z - cartesianToPolar(x_);
  y(1) = normalizeAngle(y(1));
  Update_(y);
}

void KalmanFilter::Update_(const Eigen::VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  MatrixXd I(x_.size(), x_.size());
  I.setIdentity();

  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}


