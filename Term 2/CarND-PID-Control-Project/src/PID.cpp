#include "PID.h"

/*Read other github code*/ 
/*Reason: To check whether it is fine to add is_initialized,prev_cte,prop_term,integral_term 
 ,diff_term and dt as new variable in code*/ 

/*All functions in PID.cpp are very simple and short.*/ 
/*All code written here are not copied from solution in github (self written)*/

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */
#include <iostream>
using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) 
{
    /**
    * TODO: Initialize PID coefficients (and errors, if needed)
    */
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    
    is_initialized = false;
    prev_cte = 0;
    
    prop_term = 0;
	integral_term = 0;
    diff_term = 0;
    
    dt = 1;
}

void PID::UpdateError(double cte) 
{
    /**
    * TODO: Update PID errors based on cte.
    */
    prop_term = cte;
    
    integral_term = integral_term + cte;
    
    if(is_initialized == false)
    {
    	diff_term = 0;
    	is_initialized = true;
	}
    else
    {
    	diff_term = (cte - prev_cte)/dt;
	}
	
	prev_cte = cte;
}

double PID::TotalError() 
{
    /**
    * TODO: Calculate and return the total error
    */
    p_error = - Kp*prop_term;
    i_error = - Ki*integral_term;
    d_error = - Kd*diff_term;
    // TODO: Add your total error calc here!
    /*cout<<"wei"<<Kp<<"wei"<<Ki<<"wei"<<Kd<<endl;*/
    return p_error+i_error+d_error;
}

void PID::updateValue(int index, double dp)
{
	if(index == 0)
	{
		Kp = Kp + dp;
	}
	else if(index == 1)
	{
		Ki = Ki + dp;
	}    
	else if(index == 2 )
	{
		Kd = Kd + dp;
	}
}

void PID::Debug_Print()
{
	cout<<Kp<<" "<<Ki<<" "<<Kd<<endl;
}
