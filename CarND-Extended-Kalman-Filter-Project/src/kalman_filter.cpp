#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>
using namespace std;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
    x_ = x_in; 
    P_ = P_in; 
    F_ = F_in; 
    H_ = H_in; 
    R_ = R_in; 
    Q_ = Q_in; 
}

void KalmanFilter::Predict() 
{
    /**
    * TODO: predict the state
    */
    
    //from github 
    //u is zero vector
    
    x_ = F_*x_;
    P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
    /**
	* TODO: update the state by using Kalman Filter equations
	*/
	
	MatrixXd I(4,4);
	
	I   <<   1, 0, 0, 0, 
	         0, 1, 0, 0,
	         0, 0, 1, 0,
	         0, 0, 0, 1;
	         
	//VectorXd y, 
	//MatrixXd S, K;  
	
	VectorXd y = z  - H_*x_;
	MatrixXd S = H_*P_*H_.transpose() + R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();
	    
	x_ = x_ + K*y;
	P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
    /**
    * TODO: update the state by using Extended Kalman Filter equations
    */
    
    //from github
    //MatrixXd need const
    
    //need to normalize y
    
    //for vector y
    //y = z - x_polar
    
    MatrixXd I(4,4);
	I   <<   1, 0, 0, 0, 
	         0, 1, 0, 0,
	         0, 0, 1, 0,
	         0, 0, 0, 1;
	         
    double px = x_(0);
	double py = x_(1);
	double vx = x_(2);
	double vy = x_(3);
	
	double ro = sqrt(px*px + py*py);
    double theta = atan2(py,px);
	double ro_dot = (px*vx + py*vy)/sqrt(px*px + py*py);
	
	VectorXd x_polar(3);
	x_polar << ro, theta, ro_dot;
	VectorXd y = z  - x_polar;
	
	while(y(1) > M_PI)
	{
		y(1) = y(1) - 2*M_PI;
	}
	while(y(1) < -M_PI)
	{
		y(1) = y(1) + 2*M_PI;
	}
	
	MatrixXd S = H_*P_*H_.transpose() + R_;
    MatrixXd K = P_*H_.transpose()*S.inverse();
    
    x_ = x_ + K*y;
	P_ = (I - K*H_)*P_;
}
