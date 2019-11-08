/*Khor Chean Wei*/
/*Penang, Malaysia*/

/************************************************************************/
/*Read website relating to this project*/
/*https://medium.com/intro-to-artificial-intelligence/pid-controller-udacitys-self-driving-car-nanodegree-c4fd15bdc981*/

/*Reason: To check whether twiddle implementation is possible in this main.cpp
        and learn more about the project*/
/*Twiddle algorithm is written in this website*/

/*Passage in this website*/
/*I modified the main.cpp to implement Twiddle algorithm. 
When twiddle variable set to true, simulator runs the car with 
confidents till the maximum steps set initially and go through the 
twiddle algorithm. After competition of each iteration, simulator reset to initial stage 
and car runs starts from the beginning to maximum steps. 
This process continuous until tol value below the allowed value.*/

/*Just read some code from there. (Already forget what is written there)*/

/************************************************************************/
/*Read Github code*/
/*https://github.com/benjaminsoellner/CarND_09_PIDController*/
/*Reason: Not sure how to write twiddle code*/

/*The twiddle code is written in PID::Twiddle(...)*/
/*Do not read code directly from github*/

/************************************************************************/
/*Refer this code from github*/
/*https://github.com/pierluigiferrari/PID_controller/blob/master/src/PID.cpp*/
/*
void PID::Restart(uWS::WebSocket<uWS::SERVER> ws){
  // Reset the errors
  p_error_ = 0.0;
  i_error_ = 0.0;
  d_error_ = 0.0;
  // Reset the simulator
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}*/
/************************************************************************/
/*Redo this main.cp and PID.cpp after reading many reference from website*/
/*All code written here are not copied from solution in website (self written)*/

#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
//#include <dos.h>

#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  	uWS::Hub h;

  	PID pid;
  	/**
   	* TODO: Initialize the pid variable.
  	*/
  	double Kp_ = 0;
  	double Ki_ = 0;
  	double Kd_ = 0;
  	pid.Init(Kp_,Ki_,Kd_);
  
  	int step = 0;

  	double p[] = {2.7951, 0, 0};
  	double dp[] = {1.4641, 0.81, 1};
  	int index_dp = 0;
  	
  	bool use_first = true;
  	
  	double err1;
  	double err2;
  	double error = 0;
  	double best_error = INFINITY;
  	
  	bool is_second_initialized = false;
  	
  	bool twiddle = true;
  	
  	if(twiddle == true)
  	{
  		h.onMessage([&pid,&step,&p,&dp,&index_dp,&use_first,&err1,&err2,&best_error,&error
		  ,&is_second_initialized](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
                     	
            auto s = hasData(string(data).substr(0, length));	
		    if (s != "") 
			{
		       	auto j = json::parse(s);
		
		        string event = j[0].get<string>();
		
		        if (event == "telemetry") 
				{
		          
		          	double cte = std::stod(j[1]["cte"].get<string>());
		          	double speed = std::stod(j[1]["speed"].get<string>());
			        double angle = std::stod(j[1]["steering_angle"].get<string>());
			        double steer_value;
			        
			        pid.UpdateError(cte);
			        steer_value = pid.TotalError() ;
			        if(steer_value > 1)
			        { 
					    steer_value = 1;
					}
					if(steer_value < -1)
					{
					   	steer_value = -1;
					}
					//cout<<steer_value<<endl;
			        
			        //std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
			            //<< std::endl;
			            
			        if(step > 100)
					{
						double sum_dp = 0;
						for(double part_dp : dp)
						{
							sum_dp = sum_dp + part_dp;
						}
					
						bool keep_running = true;
						while(sum_dp > 0.1 && keep_running == true)
						{
							if(use_first == true)
							{			
								pid.updateValue(index_dp, dp[index_dp]);
								keep_running = false;
								
								use_first = false;
							}
							else
							{
								if(error < best_error)
								{
									dp[index_dp] = 1.1*dp[index_dp];
									best_error = error;
									use_first = true;
								}
								else
								{
									if(is_second_initialized == false)
									{
										pid.updateValue(index_dp, -2*dp[index_dp]);	
										keep_running = false;
										
										is_second_initialized = true;		
									}
									else
									{
										if(error < best_error)
										{
											dp[index_dp] = 1.1*dp[index_dp];
											best_error = error;
										}
										else
										{
											pid.updateValue(index_dp, dp[index_dp]);
											dp[index_dp] = 0.9*dp[index_dp];
										}					
										use_first = true;
									    
									    is_second_initialized = false;
									    
									}
									
									index_dp = index_dp + 1;
									if(index_dp == 3)
									{
									    index_dp = 0;
									}
								}     
							}
						}
						
						if(sum_dp < 0.1)
						{
							cout<<"Finish"<<endl;
							pid.Debug_Print();
							ws.close();
						}
						
						json msgJson;
						string reset_msg = "42[\"reset\",{}]";
						ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
						step = 0;
  						pid.Debug_Print();
  						cout<<dp[0]<<" "<<dp[1]<<" "<<dp[2]<<endl;
  						error = 0;
  						//is_initialized = false;
					}
					else
					{
						error = error + cte*cte;
						step = step + 1;
						
						json msgJson;
						msgJson["steering_angle"] = steer_value;
						msgJson["throttle"] = 0.3;
						auto msg = "42[\"steer\"," + msgJson.dump() + "]";
						//std::cout << msg << std::endl;
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
		        }  
			}
	      				      
		}); 
		
		h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		    std::cout << "Connected!!!" << std::endl;
		});
		
		h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
		                         char *message, size_t length) {
		    ws.close();
		    std::cout << "Disconnected" << std::endl;
		});
		
		int port = 4567;
		if (h.listen(port)) 
		{
		    std::cout << "Listening to port " << port << std::endl;
		} 
		else 
		{
			std::cerr << "Failed to listen to port" << std::endl;
		    return -1;
		}
		  
		h.run();
		
  	}
  	else
  	{
  		h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
	    // "42" at the start of the message means there's a websocket message event.
	    // The 4 signifies a websocket message
	    // The 2 signifies a websocket event
	    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
	      auto s = hasData(string(data).substr(0, length));
	
	      if (s != "") {
	        auto j = json::parse(s);
	
	        string event = j[0].get<string>();
	
	        if (event == "telemetry") 
			{
	          	// j[1] is the data JSON object
	          	double cte = std::stod(j[1]["cte"].get<string>());
	          	double speed = std::stod(j[1]["speed"].get<string>());
		        double angle = std::stod(j[1]["steering_angle"].get<string>());
		        double steer_value;
		        /**
		        * TODO: Calculate steering value here, remember the steering value is
		        *   [-1, 1].
		        * NOTE: Feel free to play around with the throttle and speed.
		        *   Maybe use another PID controller to control the speed!
		        */
		        pid.UpdateError(cte);
		        steer_value = pid.TotalError() ;
		        if(steer_value > 1)
		        { 
				    steer_value = 1;
				}
				if(steer_value < -1)
				{
				   	steer_value = -1;
				}
			
		        // DEBUG
		        std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
		                    << std::endl;
		
		        json msgJson;
		        msgJson["steering_angle"] = steer_value;
		        msgJson["throttle"] = 0.3;
		        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
		        std::cout << msg << std::endl;
		        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	        }  // end "telemetry" if
	      } else {
	        // Manual driving
	        string msg = "42[\"manual\",{}]";
	        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	      }
	    }  // end websocket message if
	  }); // end h.onMessage
	
	  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
	    std::cout << "Connected!!!" << std::endl;
	  });
	
	  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
	                         char *message, size_t length) {
	    ws.close();
	    std::cout << "Disconnected" << std::endl;
	  });
	
	  int port = 4567;
	  if (h.listen(port)) {
	    std::cout << "Listening to port " << port << std::endl;
	  } else {
	    std::cerr << "Failed to listen to port" << std::endl;
	    return -1;
   }
	  
	  h.run();
	}
  
}


/*

double sum_dp = 0;
						for(double part_dp : dp)
						{
							sum_dp = sum_dp + part_dp;
						}
						
						if(sum_dp > 0.2)
						{
							if(use_first == true)
							{
								if(is_initialized == false)
								{
									pid.updateValue(index_dp, dp[index_dp]);
									is_initialized = true;
								}
								else
								{
									pid.updateValue(index_dp, dp[index_dp]);
									err1 = error; 
									
								}
								use_first = false;
							}
							else
							{
								pid.updateValue(index_dp, -2*dp[index_dp]);
								err2 = error;
								
								if(err1 < best_error)
								{
									dp[index_dp] = 1.1*dp[index_dp];
									best_error = err1;
								}
								else
								{
									pid.updateValue(index_dp, dp[index_dp]);
									dp[index_dp] = 0.9*dp[index_dp];
								}
								
								use_first = true;
							}
						}
						else
						{
							ws.close();
						}*/
