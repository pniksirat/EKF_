#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  Calculate the RMSE here.
   */
  VectorXd RMSE(4);
  RMSE<<0,0,0,0;
  vector<VectorXd> error_vector;
  VectorXd diff;
  
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return RMSE;
  }
  
  //if ((estimations.size() == ground_truth.size()) & (estimations.size()!=0 || ground_truth.size()!=0)){
  for (unsigned int n=0; n< estimations.size();++n){
  			diff=(estimations[n]-ground_truth[n]).array()*(estimations[n]-ground_truth[n]).array();
          	error_vector.push_back(diff);
          	RMSE += diff;
  		} 
  //}		
  
     
  RMSE=((RMSE/estimations.size()).array()).sqrt();
  return RMSE;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */
  /**
  	MatrixXd Hj = MatrixXd(3, 4);
  	double px = x_state(0);
    double py = x_state(1);
    double vx  = x_state(2);
    double vy = x_state(3);
  	
  	double d=pow(px,2)+pow(py,2);
  	//double d_sqrt=pow(d,0.5);
    double d_sqrt=sqrt(d);
    double d_=pow(d,3/2);
  
  
 	 Hj<< px/d_sqrt , py/d_sqrt,0,0,
  				-py/d, px/d, 0,0,
  				py*(vx*py-vy*px)/d_,px*(vy*px-vx*px)/d_,px/d_sqrt,py/d_sqrt;
  
  	return Hj;
  */
  
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
  
}
