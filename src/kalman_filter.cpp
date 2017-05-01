#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    x_ = F_ * x_; // set u to 0
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    // Calculate y for standard kalman filter
    VectorXd y = z - H_ * x_;

    // Call common Kalman Filter update function 
    UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    // Update Jacobian and h vector
    Tools tools;
    VectorXd h(3);
    H_ = MatrixXd(3, 4);
    tools.CalculateJacobian(x_, H_, h);

    // Calculate y specific to extended kalman filter
    VectorXd y = z - h; 

    // Call common Kalman Filter update function 
    UpdateCommon(y);
}

void KalmanFilter::UpdateCommon(VectorXd &y){

    // Ensure rho_error is between -pi and pi
    Tools tools;
    y(1) = tools.ModPi(y(1));

    // Generic Kalman or Extended Kalman Filter update calculations
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    MatrixXd I = MatrixXd::Identity(4, 4);

    // new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}
