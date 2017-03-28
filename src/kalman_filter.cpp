#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
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
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    //std::cout<< "z: "<< z << std::endl;
    VectorXd hHt_x = VectorXd(3);

    if (fabs(x_(0)) < 0.0001)
       x_(0) = 0.0001;

    if (fabs(x_(1)) < 0.0001)
       x_(1) = 0.0001;

    float px2 = x_(0)*x_(0);
    float py2 = x_(1)*x_(1);
    float sqrt_px2_py2 = sqrt(px2 + py2);

    hHt_x << sqrt_px2_py2,
             atan((x_(1)/x_(0))),
             (x_(2)*x_(0) + x_(3)*x_(1))/ (sqrt_px2_py2);
    //std::cout<< "hHt_x: "<< hHt_x << std::endl;

    VectorXd new_z = VectorXd(3);
    new_z << z(0),
             atan2 ( sin ( z(1) ),cos ( z(1) ) ),
             z(2);

    //std::cout<< "new_z "<< new_z << std::endl;

    VectorXd y = new_z - hHt_x;
    //std::cout<< "y: "<< y << std::endl;

    //new estimate
    x_ = x_ + (K * y);
    //std::cout<< "new x_: "<< x_ << std::endl;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;


}
