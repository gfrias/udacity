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
    
    //the estimation vector size should not be zero
    assert(estimations.size() != 0);
    
    //the estimation vector size should equal ground truth vector size
    assert (estimations.size() == ground_truth.size());
    
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

VectorXd Tools::Polar2Cartesian(const VectorXd& x_state) {
  float rho = x_state(0);
  float phi = x_state(1);
  float rho_dot = x_state(2);
  
  VectorXd ret = VectorXd(5);
  
  ret << rho*cos(phi),
  rho*sin(phi),
  rho_dot,
  0,
  0;
  
  return ret;
}

VectorXd Tools::NormalizeVector(const VectorXd &v) {
  VectorXd ret = v;
  
  if (ret.size() == 3) {
    ret(1) = NormalizeAngle(ret(1));
  } else if (ret.size() == 5 || ret.size() == 7) {
    ret(3) = NormalizeAngle(ret(3));
    ret(4) = NormalizeAngle(ret(4));
  } else if (ret.size() == 2) { }
  else {
    assert(false);
  }
  
  return ret;
}

double Tools::NormalizeAngle(double ang) {
  double val = ang;
  if (val > M_PI)
    val = fmod(val - M_PI, 2*M_PI) - M_PI;
  if (val < -M_PI)
    val = fmod(val + M_PI, 2*M_PI) + M_PI;
  
  assert (val <= M_PI);
  assert (val >= -M_PI);
  
  return val;
}
