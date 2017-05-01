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
  Eigen::VectorXd Polar2Cartesian(const Eigen::VectorXd& x_state);

  Eigen::VectorXd NormalizeVector(const Eigen::VectorXd &v);
  double NormalizeAngle(double val);
};

#endif /* TOOLS_H_ */
