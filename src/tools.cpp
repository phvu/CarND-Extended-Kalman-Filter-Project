#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse = VectorXd::Zero(4);

  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {

  MatrixXd Hj(3, 4);
  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  double c1 = px * px + py * py;
  if (fabs(c1) < 0.0001) {
    std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    c1 = 0.0001;
  }

  double c2 = sqrt(c1);
  double c3 = (c1 * c2);

  //check division by zero

  //compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;
}

double Tools::normalizeAngle(double angle) {
  double v = angle;
  while (v > M_PI) {
    v -= 2 * M_PI;
  }
  while (v < -M_PI) {
    v += 2 * M_PI;
  }
  return v;
}

VectorXd Tools::cartesianToPolar(const VectorXd &cartesian) {
  double px = cartesian[0];
  double py = cartesian[1];
  double vx = cartesian[2];
  double vy = cartesian[3];

  double rho = sqrt(px * px + py * py);
  double phi = 0;
  double rho_dot = 0;

  // avoid division by zero
  if (fabs(px) < 0.001) {
    std::cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << std::endl;
    px = 0.001;
  }
  phi = normalizeAngle(atan2(py, px));

  // avoid division by zero
  if (rho < 0.0001) {
    std::cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << std::endl;
    rho = 0.001;
  }
  rho_dot = (px * vx + py * vy) / rho;

  VectorXd polar(3);
  polar << rho, phi, rho_dot;
  return polar;
}