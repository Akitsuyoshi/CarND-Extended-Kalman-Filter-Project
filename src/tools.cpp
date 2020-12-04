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
   * TODO: Calculate the RMSE here.
   */
  // Init rmse
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check the validity if estimations and ground_truth vector are the same size
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
     cout << "Not a valid size, both paramaters should be the same size" << endl;
     return rmse;
  }

  for (unsigned int i = 0; i < estimations.size(); i++) {
     VectorXd res = estimations[i] - ground_truth[i];
     res = res.array() * res.array();
     rmse += res;
  }

  return (rmse / estimations.size()).array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
