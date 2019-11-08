#include "tools.h"
#include <iostream>
using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) 
{
	//from github
    //this function returns VectorXd type
    //need to deal with comparison between signed and unsigned integer expressions
    //refer and follow to lecture answer 
    //refer coefficient-wise multiplication
    
    /**
	* TODO: Calculate the RMSE here.
	*/
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	
	int n = estimations.size();
	
	VectorXd residual;
	
	for(unsigned i = 0; i < estimations.size(); i++)
	{
		residual = estimations[i] - ground_truth[i];
		residual = residual.array()*residual.array();
		
		rmse = rmse + residual;
	}
	
	
	rmse = rmse / n;
	rmse = rmse.array().sqrt();
	 
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
    /**
    * TODO:
    * Calculate a Jacobian here.
    */
    //from github
    //this function returns MatrixXd type
    
    //if s < 0.0001
    //Hj_ = MatrixXd::Zero(3, 4);
    
    
    MatrixXd Hj_ = MatrixXd(3, 4);
    double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);
	
	double  s = sqrt(px*px + py*py);
	//Hj_ = MatrixXd::Zero(3, 4);
	if(s < 0.001)
	{
		return Hj_;
	}
	
    Hj_ << px/s, py/s, 0, 0,
        -py/(s*s), px/(s*s), 0, 0,
           py*(vx*py - vy*px)/(s*s*s), px*(vx*px - vy*py)/(s*s*s), px/s, py/s; 
    return Hj_;

}
