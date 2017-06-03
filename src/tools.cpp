#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  int N = estimations.size();
  /**
  DONE:
    * Calculate the RMSE here.
  */
  //Sanity checks
  if(estimations.size() <= 0 || (estimations.size() != ground_truth.size()))
  {
      cout << "Sizes dont match or are zero!!! RMSE cannot be calculated." << endl;
      return rmse;
  }

  for(int i = 0; i < N; i++)
  {
      VectorXd diff(4);
      VectorXd diff2(4);
      diff = estimations[i] - ground_truth[i];
      diff2 = diff.array() * diff.array();
      rmse = rmse + diff2;
  }
  //calculate the mean
  rmse = (1.0/N)*rmse.array();
  //calculate the square root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  DONE:
    * Calculate a Jacobian here.
  */

    MatrixXd Hj(3,4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    if(fabs(c1) < 0.0001){
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;

}
