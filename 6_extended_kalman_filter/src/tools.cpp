#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
							  const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	//  * the estimation vector size should not be zero
	if (estimations.size() == 0){
	    return rmse;
	}
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()){
	    return rmse;
	}

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
		VectorXd diff = estimations[i] - ground_truth[i];
		rmse = rmse.array() + diff.array() * diff.array();
	}

	//calculate the mean
	rmse *= 1.0/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);

  float vx = x_state(2);
  float vy = x_state(3);

  float norm2 = px*px+py*py;
  float norm  = sqrt(norm2);
  float norm3_2 = pow(norm, 3);

  //check division by zero
  if (fabs(norm) < 1E-8) {
      std::cout << "Division by zero" << std::endl;
      assert(false);
  }

  Hj << px/norm, py/norm, 0, 0,
       -py/norm2, px/norm2, 0, 0,
       py*(vx*py-vy*px)/norm3_2, px*(vy*px-vx*py)/norm3_2, px/norm, py/norm;

  return Hj;
}

VectorXd Tools::Polar2Cartesian(const VectorXd& x_state) {
	float rho = x_state(0);
	float phi = x_state(1);
	float rho_dot = x_state(2);

	VectorXd ret = VectorXd(4);

	ret << rho*cos(phi),
			rho*sin(phi),
			rho_dot*cos(phi),
			rho_dot*sin(phi);

	return ret;
}
