#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

struct Config {
  double ref_v;
  double ref_epsi;
  double ref_cte;
  
  double w_cte;
  double w_epsi;
  double w_v;
  
  double w_delta;
  double w_a;
  
  double w_deltadot;
  double w_adot;
  
  int solver_N;
  double solver_dt;
  double solver_timeout;
  
  double delay;
//  double p_steering_limit;
};

class MPC {
private:
  Config config;
 public:
  MPC(Config config);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
