#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  void CalculateJacobian(const Eigen::VectorXd& x_state, Eigen::MatrixXd& Hj, Eigen::VectorXd& h);

  /**
  * function to force x between -pi and pi
  */
  float ModPi(float x);

};

#endif /* TOOLS_H_ */
