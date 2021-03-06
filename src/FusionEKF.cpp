#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
 /**
  Q_=MatrixXd(4, 4);
  
  // the initial transition matrix F_
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  
  // state covariance matrix P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 500, 0,
        0, 0, 0, 500;
   */
  
  // measurement matrix
  H_laser_ << 1, 0, 0, 0,
       		  0, 1, 0, 0;

  Hj_ << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0;
  
 
    
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
	
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    ekf_.Q_=MatrixXd(4, 4);
  
  // the initial transition matrix F_
  	ekf_.F_ = MatrixXd(4, 4);
  	ekf_.F_<< 1, 0, 1, 0,
        	 0, 1, 0, 1,
       		 0, 0, 1, 0,
       		 0, 0, 0, 1;
  
  // state covariance matrix P
  	ekf_.P_= MatrixXd(4, 4);
  	ekf_.P_ << 1, 0, 0, 0,
       		  0, 1, 0, 0,
        	  0, 0, 500, 0,
        	  0, 0, 0, 500;
    //ekf_.F_=F_;
    //ekf_.P_=P_;
    //ekf_.Q_=Q_;
    //previous_timestamp_=measurement_pack.timestamp_;
    //is_initialized_ = true;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //  Convert radar from polar to cartesian coordinates 
      //         and initialize state.
     double ro=measurement_pack.raw_measurements_(0);
     double phi=measurement_pack.raw_measurements_(1);
     double ro_dot=measurement_pack.raw_measurements_(2);
      
     double px=ro*cos(phi);
     double py=ro*sin(phi);
     double vx=ro_dot*cos(phi);
     double vy=ro_dot*sin(phi);
     
      px=(px<0.01)? 0.01:px;
      py=(py<0.01)? 0.01:py;
     
     ekf_.x_<<px,py,vx,vy;


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //  Initialize state.
      ekf_.x_<<measurement_pack.raw_measurements_(0),
      		   measurement_pack.raw_measurements_(1),
      		   0,
      		   0;

    }
    // done initializing, no need to predict or update
    previous_timestamp_=measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  //ransform time from microseconds to seconds
  float dt=(measurement_pack.timestamp_- previous_timestamp_ )/ 1000000.0;
  //cout<<"delta t:"<< dt<<endl;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_4=pow(dt,4);
  float dt_3=pow(dt,3);
  float dt_2=pow(dt,2);
  //ekf_.F_=F_;
  ekf_.F_(0,2)=dt;
  ekf_.F_(1,3)=dt;
  
  //ekf_.P_=P_;
  
  ekf_.Q_<<dt_4*noise_ax/4,0,dt_3*noise_ax/2,0,
    			0,dt_4*noise_ay/4,0,dt_3*noise_ay/2,
    			dt_3*noise_ax/2,0,dt_2*noise_ax,0,
    			0,dt_3*noise_ay/2,0,dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    	//Hj_
    	ekf_.H_=Hj_;
    	ekf_.R_=R_radar_;
    	ekf_.H_=tools.CalculateJacobian(ekf_.x_);
    	if (ekf_.H_.isZero(0)){
         return;
        }
    	
    	//VectorXd z;
        //z <<measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1),measurement_pack.raw_measurements_(2);
    	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    	//ekf_.UpdateEKF(z);

  } else {
    // TODO: Laser updates
      	ekf_.R_=R_laser_;
    	ekf_.H_=H_laser_;
    	//VectorXd z;
        //z<<measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1);
      	ekf_.Update(measurement_pack.raw_measurements_);
    	//ekf_.Update(z);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
