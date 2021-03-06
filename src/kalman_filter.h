#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   */
  void Update(const Eigen::VectorXd &z, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   */
  void UpdateEKF(const Eigen::VectorXd &z, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in);

private:

  /**
   * Update the state, given the error vector
   * Private method, will be used by Update and UpdateEKF
   */
  void Update_(const Eigen::VectorXd &y, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in);
};

#endif /* KALMAN_FILTER_H_ */
