#include "kalman_filter.h"
#include <math.h> 
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  /**
   * TODO: predict the state
   */
  x_=F_*x_;
  P_=F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
 // MatrixXd I = MatrixXd::Identity(P_.rows(),P_.cols());
  /**
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
 
  VectorXd y;
  MatrixXd S, K;
  cout<<z;
  y=z-H_*x_;
  S=H_*P_*H_.transpose()+R_;
  K=P_*H_.transpose()*S.inverse();
  x_=x_+K*y;
  P_=(I-K*H_)*P_;
  */
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * H(x)=(ro, phi, phi_dot)
   */
  // for radar cartisian needs to be converted to polar coordinates
  /**
  P_x=x_(0,0);
  P_y=x_(1,0);
  V_x=x_(2,0);
  V_y=x_(3,0);
  */
  
  //MatrixXd I = MatrixXd::Identity(P_.rows(),P_.cols());
 
  float ro=sqrt(pow(x_[0],2)+pow(x_[1],2));
  double phi=0;
  float ro_dot=0;
  
  
  if (fabs(ro)<0.001) {
    ro=0.001;  
  }
  ro_dot=(x_[0]*x_[2]+x_[1]*x_[3])/ro;
  
   
  if (fabs(x_(0,0))>0.0f) {
  	phi=atan2(x_[1],x_[0]);
  }
  else{
    phi=z(2);
  }
  
  //ro_dot=(x_(0,0)*x_(2,0)+x_(1,0)*x_(3,0))/ro;
  
  VectorXd h_x =VectorXd(3);
  //MatrixXd K;
  //MatrixXd S;
  
  
  h_x<< ro, phi, ro_dot;
  
  VectorXd  y=z-h_x;
  
  //adjust angle at y[1]

  if (y(1)>M_PI){
  	y(1)-=2*M_PI;
   // cout<<"here"<<endl;
  }
  else if (y(1)<-M_PI){
  	y(1)+=2*M_PI;
   // cout<<"here2"<<endl;;
  }
 
 // cout<<"y is :"<<y(1)<<endl;
  /**
  S=H_*P_*H_.transpose()+R_;
  K=P_*H_.transpose()*S.inverse();
  x_=x_+K*y;
  P_=(I-K*H_)*P_;
  */
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}
