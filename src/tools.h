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
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
    
  /**
   * A helper method to convert from polar to cartesian
   */
  Eigen::VectorXd Polar2Cartesian(const Eigen::VectorXd& Polar_data);
  
  /**
  * A helper method to convert from polar to cartesian
  */
  Eigen::VectorXd Cartesian2Polar(const Eigen::VectorXd& Cartesian_data);

    
    
};

#endif /* TOOLS_H_ */
