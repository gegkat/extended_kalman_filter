#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


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
    x_ = F_ * x_; // set u to 0
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    VectorXd y = z - H_ * x_;
    KF(y);

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    Tools tools;
    VectorXd h(3);
    MatrixXd Hj(3,4);
    tools.CalculateJacobian(x_, Hj, h);
    H_ = Hj;
    //cout << "H: " << H_ << endl;
    //H_ = tools.CalculateJacobian(x_);
    //MatrixXd h = tools.h(x_);
    VectorXd y = z - h; 

    cout << "        z: " << z(0) << ", " << z(1) << ", " << z(2) << endl; 
    cout << "        h: " << h(0) << ", " << h(1) << ", " << h(2) << endl; 
    cout << "        y: " << y(0) << ", " << y(1) << ", " << y(2) << endl; 

    KF(y);
}

void KalmanFilter::KF(VectorXd &y){

    if (y(1) < -M_PI) {
      // y << y(0), y(1) + 2*3.14159263, y(2);
      y(1) = y(1) + 2*M_PI;
    }

    if (y(1) > 3.14159263) {
      // y << y(0), y(1) - 2*3.14159263, y(2);
      y(1) = y(1) - 2*M_PI;
      // y[1] = y(1) - 2*3.14159263;
    }


    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    MatrixXd I = MatrixXd::Identity(4, 4);

    //new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;

}
