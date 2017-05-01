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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // initialize state transition matrix
  ekf_.F_ = MatrixXd(4, 4);

  // laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // initialize process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);

  // Initialize state covariance matrix
  ekf_.P_ = MatrixXd(4, 4);


  //set the acceleration noise components
  noise_ax = 9.0;
  noise_ay = 9.0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    // first measurement
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      float x = rho * cos(phi);
      float y = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      ekf_.x_ << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Initialize the covariance matrix with somewhat arbitrary values
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    // Initialize previous timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // Set initialized flag true
    is_initialized_ = true;

    // done initializing, no need to predict or update
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Calculate the timestep between measurements in seconds for the state transition matrix
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  
  // Form the state transitoin matrix using delta time
  ekf_.F_ << 1,  0, dt,  0,
             0,  1,  0,  dt,
             0,  0,  1,  0,
             0,  0,  0,  1;

  // Update the previous timestamp to current
  previous_timestamp_ = measurement_pack.timestamp_;

  // Pre-calculations for the process covariance
  float c = dt * dt;
  float a = c*c/4;
  float b = c*dt/2;

  // Form the process covariance matrix Q using noise estimates and delta time
  ekf_.Q_ <<  a*noise_ax, 0,          b*noise_ax, 0,
              0,          a*noise_ay, 0,          b*noise_ay,
              b*noise_ax, 0,          c*noise_ax, 0,
              0,          b*noise_ay, 0,          c*noise_ay;


  // Complete prediction step
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
  }

}
