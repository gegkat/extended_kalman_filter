#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

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
    //H_ = tools.CalculateJacobian(x_);
    //MatrixXd h = tools.h(x_);
    VectorXd y = z - h; 

    KF(y);
}

void KalmanFilter::KF(const VectorXd &y){

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    MatrixXd I = MatrixXd::Identity(4, 4);

    //new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;

}
