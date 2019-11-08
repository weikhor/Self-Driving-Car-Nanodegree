#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

#define _USE_MATH_DEFINES

/**
 * Constructor.
 */
FusionEKF::FusionEKF() 
{
  	is_initialized_ = false;

  	previous_timestamp_ = 0;

  	// initializing matrices
  	R_laser_ = MatrixXd(2, 2);
 	R_radar_ = MatrixXd(3, 3);
  
  	H_laser_ = MatrixXd(2, 4);
  	Hj_ = MatrixXd(3, 4);

  	//measurement covariance matrix - laser
 	R_laser_ << 0.0225, 0,
              	0, 0.0225;

  	//measurement covariance matrix - radar
  	R_radar_ << 0.09, 0, 0,
             	0, 0.0009, 0,
              	0, 0, 0.09;

  	/**
   	* TODO: Finish initializing the FusionEKF.
   	* TODO: Set the process and measurement noises
   	*/
    //from github
    //not need to define p_x, p_y, v1, v2
    //double p_x = fusionEKF.ekf_.x_(0);
    //double p_y = fusionEKF.ekf_.x_(1);
    //double v1  = fusionEKF.ekf_.x_(2);
    //double v2 = fusionEKF.ekf_.x_(3);
    
    //double s = sqrt(p_x*p_x + p_y*p_y);
	
	//Hj_<< 1,1,0,0,
    //      1,1,0,0,
    //      1,1,1,1;
    
    //use atan2(y, x)
    
    H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
              
    Hj_ << 1, 1, 0, 0,
           1, 1, 0, 0,
           1, 1, 1, 1; 
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() 
{
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) 
{
   /**
   * Initialization
   */
    double ro, theta, ro_dot, px, py, vx, vy;
	if (!is_initialized_) 
	{
	    /**
	     * TODO: Initialize the state ekf_.x_ with the first measurement.
	     * TODO: Create the covariance matrix.
	     * You'll need to convert radar from polar to cartesian coordinates.
	     */
	    // first measurement
	    //from github
	    //randomly define ekf_.x << 1, 1, 1, 1;
	    //not need to define noise u
	    //randomly define ekf_.P_
	    //P <<  1, 0, 0, 0, 
	    //      0, 1, 0, 0,
	    //    	0, 0, 1000, 0,
	    //    	0, 0, 0, 1000;
	    
	    //not need to calculate vx , vy;
	    previous_timestamp_ = measurement_pack.timestamp_;
	    
	    cout << "EKF: " << endl;
	   
	    VectorXd x(4);
	    
	    MatrixXd P(4,4);
	    P   <<  1, 0, 0, 0, 
	            0, 1, 0, 0,
	         	0, 0, 1000, 0,
	         	0, 0, 0, 1000;
	         	
	    ekf_.P_ = P;
	      
	    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
		{
		    // TODO: Convert radar from polar to cartesian coordinates 
		    //         and initialize state.
		    ro = measurement_pack.raw_measurements_(0);
		    theta = measurement_pack.raw_measurements_(1);
		    //ro_dot = measurement_pack.raw_measurements_(2);
		    
		    while(theta > M_PI)
		    {
		    	theta = theta - 2*M_PI;
			}
			
			while(-M_PI > theta)
			{
				theta = theta + 2*M_PI;
			}
		    
		    px = ro * cos(theta);
		    py = ro * sin(theta);
		    //vx = ro_dot * cos(theta);
		    //vy = ro_dot * sin(theta);
		    
		    x << px, py, 0, 0;
		    ekf_.x_ = x;
		    
	    }
	    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
		{
	        // TODO: Initialize state.
	        px = measurement_pack.raw_measurements_(0);
            py = measurement_pack.raw_measurements_(1);
            x << px, py, 0, 0;
		    ekf_.x_ = x;
	    }
	
	    // done initializing, no need to predict or update
	    is_initialized_ = true;
	    return;
    }

	/**
	* Prediction
	*/
	
	/**
	* TODO: Update the state transition matrix F according to the new elapsed time.
	* Time is measured in seconds.
	* TODO: Update the process noise covariance matrix.
	* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/
	
	//from github 
	//is 1000000.0, not 1000000 because dt is double
	double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
	previous_timestamp_  = measurement_pack.timestamp_;
	
	MatrixXd F(4,4);
	F   <<  1, 0, dt, 0,
	        0, 1, 0, dt,
		    0, 0, 1, 0,
		    0, 0, 0, 1;
	ekf_.F_ = F;
	//from github
    // use Noise values from the task
    // Noise values from the task
    // nx = 9.0. ny = 9.0
    double nx, ny;
	nx = 9.0;
	ny = 9.0; 
    
    MatrixXd Q(4,4);
	Q   <<  (dt*dt*dt*dt/4)*nx*nx, 0, (dt*dt*dt/2)*nx*nx, 0,
	        0,(dt*dt*dt*dt/4)*ny*ny, 0, (dt*dt*dt/2)*ny*ny,
		    (dt*dt*dt/2)*nx*nx, 0, dt*dt*nx*nx, 0,
		    0, (dt*dt*dt/2)*ny*ny, 0, dt*dt*ny*ny;
	ekf_.Q_ = Q;

  	ekf_.Predict();

	/**
	* Update
	*/

	/**
	* TODO:
	* - Use the sensor type to perform the update step.
	   * - Update the state and covariance matrices.
	   */
	      
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
	{
	    // TODO: Radar updates
	    //from github
	    //is tools.CalculateJacobian(ekf.x_), not tools.CalculateJacobian(xstate)
	    ro = measurement_pack.raw_measurements_(0);
		theta = measurement_pack.raw_measurements_(1);
		ro_dot = measurement_pack.raw_measurements_(2);
		
		while(theta > M_PI)
		{
		    theta = theta - 2*M_PI;
		}
			
		while(-M_PI > theta)
		{
			theta = theta + 2*M_PI;
		}
		    
		      
		/*px = ro * cos (theta);
		py = ro * sin (theta);
		vx = ro_dot * cos (theta);
		vy = ro_dot * sin (theta);
		
	    MatrixXd xstate = VectorXd(4);
		xstate << px, py, vx, vy;*/
		
		ekf_.H_ = tools.CalculateJacobian(ekf_.x_) ;
	    ekf_.R_ = R_radar_;
	    
	    VectorXd z = VectorXd(3);
	    z << ro, theta, ro_dot;
	    ekf_.UpdateEKF(z);
	}
	
	else 
	{
	    // TODO: Laser updates
	    px = measurement_pack.raw_measurements_(0);
	    py = measurement_pack.raw_measurements_(1); 
	    
	    ekf_.H_ = H_laser_;
	    ekf_.R_ = R_laser_;
	    
		VectorXd z = VectorXd(2);
	    z << px, py;
	    ekf_.Update(z);
	}

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
