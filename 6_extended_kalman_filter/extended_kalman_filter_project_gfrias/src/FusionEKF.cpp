#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  //measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  Eigen::VectorXd raw_measurements = measurement_pack.raw_measurements_;

  float epsilon = 1E-4;
  if (fabs(raw_measurements(0)) < epsilon && fabs(raw_measurements(1)) < epsilon) {
    raw_measurements(0) = epsilon;
    raw_measurements(1) = epsilon;
  }

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;

    //state covariance matrix P
    MatrixXd P_in = MatrixXd(4, 4);
    P_in << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    //process covariance matrix
    MatrixXd Q_in = MatrixXd(4, 4);

    //the initial transition matrix F_
    MatrixXd F_in = MatrixXd(4, 4);
    F_in << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

    VectorXd x_in = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      x_in = tools.Polar2Cartesian(raw_measurements);

      Hj_ = tools.CalculateJacobian(x_in);
      ekf_.Init(x_in, P_in, F_in, Hj_, R_radar_, Q_in);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_in << raw_measurements[0], raw_measurements[1], 0, 0;

      ekf_.Init(x_in, P_in, F_in, H_laser_, R_laser_, Q_in);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //Update the state transition matrix F according to the new elapsed time.
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  /*
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  */
  float dt2 = dt*dt;
  float dt3 = dt*dt2/2;
  float dt4 = dt2*dt2/4;

  float noise_ax = 9;
  float noise_ay = 9;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4*noise_ax, 0, dt3*noise_ax, 0,
        0, dt4*noise_ay, 0, dt3*noise_ay,
        dt3*noise_ax, 0, dt2*noise_ax, 0,
        0, dt3*noise_ay, 0, dt2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    VectorXd x_in = tools.Polar2Cartesian(raw_measurements);

    ekf_.H_ = tools.CalculateJacobian(x_in); //Hj
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(raw_measurements);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(raw_measurements);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}