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
   * Calculation of RMSE.
   */
	VectorXd RMSE(4);
	RMSE << 0,0,0,0;

	if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    	cout << "Invalid estimation or ground_truth data" << endl;
    return RMSE;
  }
  for (unsigned int i = 0; i<estimations.size();++i){
		VectorXd error = estimations[i] - ground_truth[i];
		error = error.array() * error.array();
		RMSE += error; 
  } 
  RMSE /= estimations.size();
  RMSE = RMSE.array().sqrt();

	return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate Jacobian matrix for RADAR 
   */
	MatrixXd Jacobian(3,4);

  //State parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  if (fabs(px) < 0.0001 and fabs(py) < 0.0001){
    px = 0.0001;
    py = 0.0001;
  }

  
  float c1 = px*px+py*py;
  //check division by zero
  if(fabs(c1) < 0.000001){
    c1 = 0.000001;
  }

  double c1_sqrt = sqrt(c1);
  double c2 = (c1*c1_sqrt);

  //compute the Jacobian matrix
  Jacobian << (px/c1_sqrt),              (py/c1_sqrt),                0,    0,
        -(py/c1),             (px/c1),                0,     0,
        py*(vx*py - vy*px)/c2, px*(px*vy - py*vx)/c2, px/c1_sqrt, py/c1_sqrt;

  return Jacobian;
}
