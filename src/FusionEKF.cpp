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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1,0,0,0,
              0,1,0,0;


  noise_ax = 9;
  noise_ay = 9;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1; # no need for this anymore, proper initialization below

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      //float phi_dot = measurement_pack.raw_measurements_(2);

      ekf_.x_ << (cos(phi) * ro) + 0.00000001, //dx
                 (sin(phi) * ro) + 0.00000001, //dy
                 0, //try later cos(phi_dot) * ro, //Vx
                 0; //try later sin(phi_dot) * ro;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0] + 0.00000001,
                 measurement_pack.raw_measurements_[1] + 0.00000001,
                 0,
                 0;


    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

  float ro = measurement_pack.raw_measurements_(0);
  float phi = measurement_pack.raw_measurements_(1);
  float ro_dot = measurement_pack.raw_measurements_(2);

  VectorXd x_state(4);
  x_state << cos(phi) * ro, //dx
             sin(phi) * ro, //dy
             cos(phi) * ro_dot, //Vx
             sin(phi) * ro_dot; //Vy

  Tools tools;
  //cout<<"ro phi"<<ro << " "<< phi<<endl;
  //cout<<"Jacobian Input: "<<x_state<<endl;
  Hj_ = tools.CalculateJacobian(x_state);
  //cout<<"Jacobian Output: "<<Hj_<<endl;

  ekf_.R_ = R_radar_;

  ekf_.H_ = Hj_;

  ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates

  //ekf_.R_ = MatrixXd(2, 2);
  ekf_.R_ = R_laser_;

  //ekf_.H_ = MatrixXd(2,4);
  ekf_.H_ = H_laser_;

  ekf_.Update(measurement_pack.raw_measurements_);
  //cout<<"Measurement Updated"<< endl;
  }

  // print the output
  //cout << "dt = " << dt << endl;
  //cout << "measurement_pack" << measurement_pack.raw_measurements_ << endl;
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
