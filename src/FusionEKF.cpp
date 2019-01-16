#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

#define EPSILON 0.0001     // Define a small number to check float equality to zero 

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_      = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

//the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0,    0,
             0, 1, 0,    0,
             0, 0, 1000, 0,
             0, 0, 0,    1000;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /***********************************************************************
   *                              Initialization
   ***********************************************************************/
  if (!is_initialized_) 
  {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float ro      = measurement_pack.raw_measurements_[0];
      float theta   = measurement_pack.raw_measurements_[1];
      float ro_dot  = measurement_pack.raw_measurements_[2];

      ekf_.x_(0) = (ro * cos(theta)),       // px
      ekf_.x_(1) = (ro * sin(theta)),       // py
      ekf_.x_(2) = (ro_dot * cos(theta)),   // vx                  
      ekf_.x_(3) = (ro_dot * sin(theta));   // vy
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      // set the state with the initial location and zero velocity
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];   // px
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];   // py
    }

    // Check to avoid divisibility by Zero
    if (fabs(ekf_.x_(0)) < EPSILON and fabs(ekf_.x_(1)) < EPSILON) {
      ekf_.x_(0) = EPSILON;
      ekf_.x_(1) = EPSILON;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /********************************************************************
   *                            Prediction
   ********************************************************************/

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;   // dt is in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update State Transition Matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Noise values from the task
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  
  // Precompute some usefull values to speed up calculations of Q
  float dt_square = (dt * dt);
  float dt_4_by_4 = (dt_square * dt_square) / 4;
  float dt_3_by_2 = (dt_square * dt) / 2;

  // Noise covariance matrix computation  
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_by_4 * noise_ax, 0, dt_3_by_2 * noise_ax, 0,
	           0, dt_4_by_4 * noise_ay, 0, dt_3_by_2 * noise_ay,
	           dt_3_by_2 * noise_ax, 0, dt_square * noise_ax, 0,
             0, dt_3_by_2 * noise_ay, 0, dt_square * noise_ay;

  ekf_.Predict();

  /***************************************************************
   *                    Update - Step
   ***************************************************************/

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    // Use Jacobian instead of H

	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  
  } else {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
