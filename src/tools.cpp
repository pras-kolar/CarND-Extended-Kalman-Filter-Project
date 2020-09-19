#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::vector;


Tools::Tools() {}

Tools::~Tools() {}


/*
// Lamda type processing - WIP
auto fnRMSE = [](const vector<VectorXd> &a, const vector<VectorXd> &b) {
    auto e = a - b; 
    return e*e;
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
  auto sum = std::transform_reduce(estimations.begin(), estimations.end(), ground_truth.begin(), 0, std::plus<>(), fnRMSE);
  
  return std::sqrt(sum / estimations.size());
}
*/
// Conventional RMSE calculation - more simplistic
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if(estimations.size() == 0){
      cout << "Function ERROR - CalculateRMSE () - The estimations vector is empty" << endl;
      return rmse;
    }

    if(ground_truth.size() == 0){
      cout << "Function ERROR - CalculateRMSE () - The ground-truth vector is empty" << endl;
      return rmse;
    }

  	unsigned int e_size = estimations.size();
  	unsigned int g_size = ground_truth.size();
    if(e_size != g_size){
      cout << "Function ERROR - CalculateRMSE () - Sizes of ground-truth and estimations vectors must be same." << endl;
      return rmse;
    }
 
    for(unsigned int i=0; i < estimations.size(); ++i){
      VectorXd diff = estimations[i] - ground_truth[i];
      diff = diff.array()*diff.array();
      rmse += diff;
    }

    rmse = rmse / e_size;
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

  if ( x_state.size() != 4 ) {
    cout << "Function ERROR - CalculateJacobian () - The state vector must have size 4." << endl;
    return Hj;
  }
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//Reduction in repeated calculation
	double c1 = px*px+py*py;
	double c2 = sqrt(c1);
	double c3 = (c1*c2);

	//Verify division by zero
	if(fabs(c1) < 0.001){
		cout << "Function ERROR - CalculateJacobian () - Division by Zero" << endl;
		return Hj;
	}
	//Jacobian matrix computation
	Hj << (px/c2), (py/c2), 0, 0, -(py/c1), (px/c1), 0, 0, py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
	return Hj;
}
