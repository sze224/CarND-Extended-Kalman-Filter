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
  
    /* initialize RMSE output */
    VectorXd RMSE(4);
    RMSE << 0, 0, 0, 0;
    
    /* check if estimation is empty or if ground_truth and estimations are different length */
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
        cout << "Error: Estimation is a size 0 or Estimations and ground truth are different size" << endl;
    }else{
    /* if the inputs look good, compute the RMSE */
        for (int i = 0; i < estimations.size(); i++){
            VectorXd residual = estimations[i] - ground_truth[i];
            residual = residual.array() * residual.array();
            RMSE = RMSE + residual;
        }
        RMSE = RMSE / estimations.size();
        RMSE = RMSE.array().sqrt();
    }
    return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
 
    /* initialize the Jacobian output */
    MatrixXd Hj(3,4);
    
    /* get states from x_state */
    float px = x_state[0];
    float py = x_state[1];
    float vx = x_state[2];
    float vy = x_state[3];
   
    /* calculate reusable values */
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = c1 * c2;
    
    /* check if the calculation will contain dividion by 0 */
    if (fabs(c1) < 0.0001){
        cout << "Error: Division by 0" << endl;
        
    }else{
    /* if no division by zero, calculate the Jacobian */
        Hj <<  (px/c2), (py/c2), 0, 0,
               -(py/c1), (px/c1), 0, 0,
        py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;
    }
    return Hj;
}

VectorXd Tools::Polar2Cartesian(const VectorXd& Polar_data){
    VectorXd c_state(4);
    
    float rho = Polar_data[0];
    float phi = Polar_data[1];
    float rho_dot = Polar_data[2];
    
    float px = rho * cos(phi);
    float py = rho * sin(phi);
    float vx = rho_dot * cos(phi);
    float vy = rho_dot * sin(phi);
    
    c_state << px, py, vx, vy;
    return c_state;
}

VectorXd Tools::Cartesian2Polar(const VectorXd& Cartesian_data){
    VectorXd p_state(3);
    
    float px = Cartesian_data[0];
    float py = Cartesian_data[1];
    float vx = Cartesian_data[2];
    float vy = Cartesian_data[3];
    
    float rho = sqrt(px * px + py * py);
    float phi = atan2(py,px);
    float rho_dot = (px * vx + py * vy) / rho;
    
    p_state << rho, phi, rho_dot;
    
    return p_state;
    
}




