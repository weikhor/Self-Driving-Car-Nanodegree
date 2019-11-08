#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

//By Khor Chean Wei 
//From Penang, Malaysia

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() 
{
    // if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;
	
	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;
	
	// initial state vector
	x_ = VectorXd(5);
	
	// initial covariance matrix
	P_ = MatrixXd(5, 5);
	
	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 30;
	
	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 30;
	  
	/**
	* DO NOT MODIFY measurement noise values below.
	* These are provided by the sensor manufacturer.
	*/
	
	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;
	
	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;
	
	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.3;
	
	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.03;
	
	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ = 0.3;
	  
	/**
	* End DO NOT MODIFY section for measurement noise values 
	*/
	  
	/**
	* TODO: Complete the initialization. See ukf.h for other member properties.
	* Hint: one or more values initialized above might be wildly off...
	*/
	is_initialized_ = false;
	
	n_x_ = 5;
	n_aug_ = 7;
	
	P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
	      0, 0, 1, 0, 0,
	      0, 0, 0, 1, 0,
	      0, 0, 0, 0, 1;
	      
	lambda_ = 3 - n_x_;
	
	weights_ = VectorXd(1+2*n_aug_);
	weights_(0) = lambda_/(lambda_ + n_aug_);
	
	for(int i = 1; i < 2*n_aug_+1; i++)
	{
	    weights_(i) = 1/(2*(lambda_ + n_aug_));
	}
	
	Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

	
}

UKF::~UKF() 
{
	
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
    /**
	* TODO: Complete this function! Make sure you switch between lidar and radar
	* measurements.
	*/
	
	if(is_initialized_ == false)
	{
		is_initialized_ = true;
		if(meas_package.sensor_type_ == MeasurementPackage::LASER)
		{
			float px, py;
			long long current_time;
			
			px = meas_package.raw_measurements_(0);
			py = meas_package.raw_measurements_(1);
			x_ << px, py, 0, 0, 0;
			
			time_us_ = meas_package.timestamp_;
		}
		else if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
		{
			float ro, theta, ro_dot; 
			float px, py, v;
			long long current_time;
			
			ro = meas_package.raw_measurements_(0);
			theta = meas_package.raw_measurements_(1);
			ro_dot =  meas_package.raw_measurements_(2);
			
			while (theta > M_PI)
			{
				theta = theta - 2*M_PI;
			}
			while (theta < -M_PI)
			{
				theta = theta + 2*M_PI;
			}
			
			px = ro * cos(theta);
			py = ro * sin(theta);
			x_ << px, py, ro_dot, theta, 0;
			
			time_us_ =  meas_package.timestamp_;
		}
	}
	else
	{
		//form previous kalman filter project
	    //time is in mega unit
	    //needed to be divided by 1000000.0
		long long current_time = meas_package.timestamp_;
		double delta_t = (current_time - time_us_)/1000000.0;
		time_us_ = current_time;
		this->Prediction(delta_t);
		
		//from github 
        //need to use use_laser_ and use_radar_ to determine which sensor needed to be used
		if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_== true)
		{
			this->UpdateLidar(meas_package);
		}
		else if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_== true)
		{
			this->UpdateRadar(meas_package);
		}
	}	

}

void UKF::Prediction(double delta_t) 
{
    /**
	* TODO: Complete this function! Estimate the object's location. 
	* Modify the state vector, x_. Predict sigma points, the state, 
    * and the state covariance matrix.
    */
    
    //Prediction 
	//Create sigma point matrix
    // augmented state vector
    VectorXd x_aug_ = VectorXd(n_aug_);
    for(int i = 0 ; i < n_x_; i++)
    {
    	x_aug_(i) = x_(i);
	}
	x_aug_(5) = 0;
	x_aug_(6) = 0;
    
	MatrixXd x_sig_aug_ = MatrixXd(n_aug_, 2*n_aug_ + 1);
	//  augmented covariance matrix
	MatrixXd Q_ = MatrixXd(2, 2);
	MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
	P_aug_.setZero(n_aug_, n_aug_);
	
    //create augmented covariance matrix
    for(int i = 0; i < n_x_; i++)
    {
        for(int j = 0; j < n_x_; j++)
        {
            P_aug_(i, j) = P_(i, j);
		}
	}
	Q_<< std_a_*std_a_, 0,
          0, std_yawdd_*std_yawdd_;
                 
    for(int i = 0; i < n_aug_-n_x_; i++)
    {
        for(int j = 0; j < n_aug_-n_x_; j++)
        {
            P_aug_(n_x_ + i, n_x_ + j) = Q_(i, j);
		}
	}
	
	//square root matrix
	MatrixXd x1 = MatrixXd::Zero(n_aug_, n_aug_);
    MatrixXd x2 = MatrixXd::Zero(n_aug_, n_aug_);
            
    MatrixXd A = P_aug_.llt().matrixL();
	x1 = sqrt(lambda_ + n_aug_) * A;
    x2 = sqrt(lambda_ + n_aug_) * A;
    
    
	for(int i = 0; i < n_x_; i++)
	{
		x_sig_aug_(i,0) = x_(i);
	}
	
	
	for(int i = 0; i < n_aug_; i++)
	{
		for(int j = 0; j < n_aug_; j++)
		{
		  	x_sig_aug_(j, i+1) = x_aug_(j) + x1(j,i);
		  	x_sig_aug_(j, i+1+n_aug_) = x_aug_(j) - x2(j ,i);
		}
	}
	
	//Sigma point prediction
	//MatrixXd x_sig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
	//MatrixXd P_sig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
	
	float px, py, v, phi, phi_dot, v_a, v_phi_dot_dot = 0;
	for(int i = 0; i < 2*n_aug_+1; i++)
	{
		px = x_sig_aug_(0,i);
		py = x_sig_aug_(1,i);
		v = x_sig_aug_(2,i);
		phi = x_sig_aug_(3,i);
		phi_dot = x_sig_aug_(4,i);
		v_a = x_sig_aug_(5,i);
		v_phi_dot_dot = x_sig_aug_(6,i);
		
		if(phi_dot > 0.001)
		{
			Xsig_pred_(0,i) = px + v/phi_dot*(sin(phi+phi_dot*delta_t)-sin(phi)) + (1/2.0)*(delta_t)*(delta_t)*cos(phi)*v_a;
			Xsig_pred_(1,i) = py + v/phi_dot*(-cos(phi+phi_dot*delta_t)+cos(phi)) + (1/2.0)*(delta_t)*(delta_t)*sin(phi)*v_a;
			Xsig_pred_(2,i) = v + 0 + delta_t*v_a;
			Xsig_pred_(3,i) = phi + phi_dot*delta_t + (1/2.0)*(delta_t)*(delta_t)*v_phi_dot_dot;
			Xsig_pred_(4,i) = phi_dot + 0 + delta_t*v_phi_dot_dot;
		}
		else
		{
			Xsig_pred_(0,i) = px + v*cos(phi)*delta_t + (1/2.0)*(delta_t)*(delta_t)*cos(phi)*v_a;
			Xsig_pred_(1,i) = py + v*sin(phi)*delta_t + (1/2.0)*(delta_t)*(delta_t)*sin(phi)*v_a;
			Xsig_pred_(2,i) = v + 0 + delta_t*v_a;
			Xsig_pred_(3,i) = phi + phi_dot*delta_t + phi_dot*delta_t + (1/2.0)*(delta_t)*(delta_t)*v_phi_dot_dot;
			Xsig_pred_(4,i) = phi_dot + 0 + delta_t*v_phi_dot_dot;
		}
		
	}
	//Predicted mean from sigma predicted state vector
	x_ = Xsig_pred_*weights_;
	while(x_(3) > M_PI)
	{
		x_(3) = x_(3) - 2*M_PI;
    }
	while(x_(3) < -M_PI)
	{
		x_(3) = x_(3) + 2*M_PI;
	}
	//Predicted covariance from sigma predicted state vector
	MatrixXd X_transpose = MatrixXd(n_x_, 2 * n_aug_ + 1);
	MatrixXd X_no_transpose = MatrixXd(n_x_, 2*n_aug_+1);
	
    for(int i = 0; i < 2*n_aug_+1; i++)
    {
    	for(int j = 0; j < n_x_; j++)  
    	{
    		float diff = Xsig_pred_(j, i) - x_(j);
    		if(j == 3)
    		{
				while(diff > M_PI)
		    	{
		    		diff = diff - 2*M_PI;
				}
		    	while(diff < -M_PI)
		    	{
		    		diff = diff + 2*M_PI;
				}
				X_transpose(j, i) = diff;
    		    X_no_transpose(j, i) = diff * weights_(i);
			}
			else
			{
				X_transpose(j, i) = diff;
    		    X_no_transpose(j, i) = (diff) * weights_(i);
			}
		}
    }
    P_ = X_no_transpose*X_transpose.transpose();
	
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
    * TODO: Complete this function! Use lidar data to update the belief 
    * about the object's position. Modify the state vector, x_, and 
    * covariance, P_.
    * You can also calculate the lidar NIS, if desired.
    */
    //get predicted measurement mean from sigma point prediction
    
    //from github 
    //need to use unscented kalman filters same as radar
    
    
    int n_z_ = 2;
    
    MatrixXd Z_ = MatrixXd::Zero(n_z_, 2*n_aug_+1);
    VectorXd z_pred = VectorXd::Zero(n_z_);
    
    float px, py; 
    
    for(int i = 0; i < 2*n_aug_+1; i++)
    {
	    px = Xsig_pred_(0, i);
		py = Xsig_pred_(1, i);
  	    
  	    Z_(0, i) = px;
  	    Z_(1, i) = py;
	}
	z_pred = Z_*weights_;
	
	//get predicted measurement covariance from sigma point prediction
	MatrixXd S_;  
    MatrixXd S_transpose = MatrixXd(n_z_, 2*n_aug_+1);
	MatrixXd S_no_transpose = MatrixXd(n_z_, 2*n_aug_+1);
	
	MatrixXd R_ = MatrixXd(n_z_, n_z_);

	R_ << std_laspx_*std_laspx_, 0,
	      0, std_laspy_*std_laspy_;
	
	for(int i = 0; i < 2*n_aug_+1; i++)
    {
    	for(int j = 0; j < n_z_; j++)  
    	{
    		float diff = Xsig_pred_(j, i) - x_(j);
    		if(j == 1)
    		{
				while(diff > M_PI)
		    	{
		    		diff = diff - 2*M_PI;
				}
		    	while(diff < -M_PI)
		    	{
		    		diff = diff + 2*M_PI;
				}
				S_transpose(j, i) = diff;
    		    S_no_transpose(j, i) = diff * weights_(i);
			}
			else
			{
				S_transpose(j, i) = diff;
    		    S_no_transpose(j, i) = diff * weights_(i);
			}
		}
    }
    
    S_ = S_no_transpose*S_transpose.transpose();
    S_ = S_  + R_;
    
    //Update state
    //calculate cross correlation matrix
    
    MatrixXd A = MatrixXd(n_x_, 2 * n_aug_ + 1);
    MatrixXd B = MatrixXd(n_z_, 2 * n_aug_ + 1);
    MatrixXd Tc = MatrixXd(n_x_, n_z_);
    double ang1;
    double ang2;
    
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    {
    	for(int j = 0; j < n_x_; j++)
        {
        	ang1 = Xsig_pred_(j,i) - x_(j);
        	if(j == 3)
        	{
        		while(ang1 > M_PI)
        	    {
        		    ang1 = ang1 - 2*M_PI;
			    }
        	    while(ang1 < -M_PI)
        	    {
        		    ang1 = ang1 + 2*M_PI;
			    }
			}
        	A(j,i) = (ang1)*weights_(i);
        }
        
        for(int k = 0; k < n_z_; k++)
        {
        	ang2 = Z_(k,i) - z_pred(k);
			B(k,i) = ang2;
        }
    }
    Tc = A*B.transpose();
    
    //calculate Kalman gain K;
    MatrixXd K = Tc*S_.inverse();
    
    //update state mean
    VectorXd z = VectorXd(n_z_);
    z(0) = meas_package.raw_measurements_(0);
	z(1) =  meas_package.raw_measurements_(1);
	
    VectorXd Z = z - z_pred;
	
    x_ = x_ + K*Z;
    
    while(x_(3) > M_PI)
    {
        x_(3) = x_(3) - 2*M_PI;
	}
    while(x_(3) < -M_PI)
    {
        x_(3) = x_(3) + 2*M_PI;
	}
	
    //update covariance matrix
    P_ = P_ - K*S_*K.transpose();
    
}

void UKF::UpdateRadar(MeasurementPackage meas_package) 
{
    /**
    * TODO: Complete this function! Use radar data to update the belief 
    * about the object's position. Modify the state vector, x_, and 
    * covariance, P_.
    * You can also calculate the radar NIS, if desired.
    */
    
    //get predicted measurement mean from sigma point prediction
    
    int n_z_ = 3;
    
    MatrixXd Z_ = MatrixXd::Zero(n_z_, 2*n_aug_+1);
    VectorXd z_pred = VectorXd::Zero(n_z_);
    
    float ro, theta, ro_dot; 
    float px, py, v, phi, phi_dot;
    
    for(int i = 0; i < 2*n_aug_+1; i++)
    {
	    px = Xsig_pred_(0, i);
		py = Xsig_pred_(1, i);
		v = Xsig_pred_(2, i);
		phi = Xsig_pred_(3, i);
		phi_dot = Xsig_pred_(4, i);
		
		ro = sqrt(px*px + py*py);
  	    theta = atan2(py, px);
  	    ro_dot = (px*cos(phi)*v + py*sin(phi)*v) / (sqrt(px*px + py*py));
  	    
  	    Z_(0, i) = ro;
  	    Z_(1, i) = theta;
  	    Z_(2, i) = ro_dot;
	}
	z_pred = Z_*weights_;
	
	
	//get predicted measurement covariance from sigma point prediction
	MatrixXd S_;
	MatrixXd S_transpose = MatrixXd(n_z_, 2*n_aug_+1);
	MatrixXd S_no_transpose = MatrixXd(n_z_, 2*n_aug_+1);
	
	MatrixXd R_ = MatrixXd(n_z_, n_z_);

	R_ << std_radr_*std_radr_, 0, 0,
	      0, std_radphi_*std_radphi_, 0,
	      0, 0, std_radrd_*std_radrd_;
	
	for(int i = 0; i < 2*n_aug_+1; i++)
    {
    	for(int j = 0; j < n_z_; j++)  
    	{
    		float diff = Z_(j, i) - z_pred(j);
    		if(j == 1)
    		{
				while(diff > M_PI)
		    	{
		    		diff = diff - 2*M_PI;
				}
		    	while(diff < -M_PI)
		    	{
		    		diff = diff + 2*M_PI;
				}
				S_transpose(j, i) = diff;
    		    S_no_transpose(j, i) = diff * weights_(i);
			}
			else
			{
				S_transpose(j, i) = diff;
    		    S_no_transpose(j, i) = diff * weights_(i);
			}
		}
    }
    
    S_ = S_no_transpose*S_transpose.transpose();
    S_ = S_  + R_;
    
    //Update state
    //calculate cross correlation matrix
    MatrixXd A = MatrixXd(n_x_, 2 * n_aug_ + 1);
    MatrixXd B = MatrixXd(n_z_, 2 * n_aug_ + 1);
    MatrixXd Tc = MatrixXd(n_x_, n_z_);
    double ang1;
    double ang2;
    
    for(int i = 0; i < 2 * n_aug_ + 1; i++)
    {
    	for(int j = 0; j < n_x_; j++)
        {
        	ang1 = Xsig_pred_(j,i) - x_(j);
        	if(j == 3)
        	{
        		while(ang1 > M_PI)
        	    {
        		    ang1 = ang1 - 2*M_PI;
			    }
        	    while(ang1 < -M_PI)
        	    {
        		    ang1 = ang1 + 2*M_PI;
			    }
			}
        	A(j,i) = (ang1)*weights_(i);
        }
        
        for(int k = 0; k < n_z_; k++)
        {
        	ang2 = Z_(k,i) - z_pred(k);
        	if(k == 1)
        	{
        		while(ang2 > M_PI)
        	    {
        		    ang2 = ang2 - 2*M_PI;
			    }
        	    while(ang2 < -M_PI)
        	    {
        		    ang2 = ang2 + 2*M_PI;
			    }
			}
			B(k,i) = ang2;
        }
    }
    Tc = A*B.transpose();
    
    //calculate Kalman gain K;
    MatrixXd K = Tc*S_.inverse();
    
    //update state mean
    VectorXd z = VectorXd(n_z_);
    z(0) = meas_package.raw_measurements_(0);
	z(1) =  meas_package.raw_measurements_(1);
	z(2) =  meas_package.raw_measurements_(2);
	
    VectorXd Z = z - z_pred;
    while(Z(1) > M_PI)
    {
        Z(1) = Z(1) - 2*M_PI;
	}
    while(Z(1) < -M_PI)
    {
        Z(1) = Z(1) + 2*M_PI;
	}
	
    x_ = x_ + K*Z;
    while(x_(3) > M_PI)
    {
        x_(3) = x_(3) - 2*M_PI;
	}
    while(x_(3) < -M_PI)
    {
        x_(3) = x_(3) + 2*M_PI;
	}

    //update covariance matrix
    P_ = P_ - K*S_*K.transpose();
       
}
