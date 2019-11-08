           /**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */
 
/*Khor Chean Wei*/
/*Penang, Malaysia*/

#include <iostream>
#include "particle_filter.h"
#include <string>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

//# define M_PI           3.14159265358979323846  /* pi */

using namespace std;

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
    /**
    * TODO: Set the number of particles. Initialize all particles to 
    *   first position (based on estimates of x, y, theta and their uncertainties
    *   from GPS) and all weights to 1. 
    * TODO: Add random Gaussian noise to each particle.
    * NOTE: Consult particle_filter.h for more information about this method 
    *   (and others in this file).
    */
    /*initialize the particle filter */
    /*Look at other github code for checking*/
    /*Some of number of particles initialized is 100*/
	 
    is_initialized = true;
	num_particles = 100;  // TODO: Set the number of particles
	
	double std_gps_x = std[0];
	double std_gps_y = std[1];
	double std_gps_theta = std[2];
	
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std_gps_x);
	normal_distribution<double> dist_y(y, std_gps_y);
	normal_distribution<double> dist_theta(theta, std_gps_theta);
	
	for(int i = 0 ; i < num_particles; i++)
	{
		Particle new_particle;
		int sample_id;
		double sample_x, sample_y, sample_theta, sample_weight;
		
		sample_id = i;
		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);
		sample_weight = 1.0;
		
		new_particle.id = sample_id;
		new_particle.x = sample_x;
		new_particle.y = sample_y;
		
		new_particle.theta = sample_theta;
		new_particle.weight = sample_weight;
		
		particles.push_back(new_particle);
	}
}

void ParticleFilter::prediction(double delta_t,double std_pos[],double velocity,double yaw_rate) 
{
    /**
    * TODO: Add measurements to each particle and add random Gaussian noise.
    * NOTE: When adding noise you may find std::normal_distribution 
    *   and std::default_random_engine useful.
    *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    *  http://www.cplusplus.com/reference/random/default_random_engine/
    */
    /*Look at other github code for checking*/
    /*fabs(yaw_rate) > 0.001*/
    
    /*Do not use while(particle_theta > 2*M_PI) 
	Possible Reason: Car theta shows the same meaning even use while(particle_theta > 2*M_PI)*/
    
    double particle_x, particle_y, particle_theta;
    
    double std_gps_x = std_pos[0];
	double std_gps_y = std_pos[1];
	double std_gps_theta = std_pos[2];
	
	default_random_engine gen;
	
    for(int i = 0 ; i < num_particles; i++)
	{
		particle_x = particles[i].x;
		particle_y = particles[i].y;
		particle_theta = particles[i].theta;
	    
		if(yaw_rate < 0.000001 && yaw_rate > -0.000001)
		{
			normal_distribution<double> dist_x(particle_x + velocity*delta_t*cos(particle_theta), std_gps_x);
	        normal_distribution<double> dist_y(particle_y + velocity*delta_t*sin(particle_theta), std_gps_y);
	        normal_distribution<double> dist_theta(particle_theta, std_gps_theta);
			
			particle_theta = dist_theta(gen);
			particle_x = dist_x(gen);
			particle_y = dist_y(gen);
		}
		else
		{
			normal_distribution<double> dist_x(particle_x + (velocity/yaw_rate)*( sin(particle_theta+yaw_rate*delta_t) - sin(particle_theta)), std_gps_x);
	        normal_distribution<double> dist_y(particle_y + (velocity/yaw_rate)*( cos(particle_theta) - cos(particle_theta+yaw_rate*delta_t)), std_gps_y);
	        normal_distribution<double> dist_theta(particle_theta + yaw_rate*delta_t, std_gps_theta);
	        
	    	particle_theta = dist_theta(gen);
			particle_x = dist_x(gen);
			particle_y = dist_y(gen);
		}
		
		particles[i].x = particle_x;
	    particles[i].y = particle_y;
		particles[i].theta = particle_theta;
	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,vector<LandmarkObs>& observations) 
{      
    /**
	* TODO: Find the predicted measurement that is closest to each 
    *   observed measurement and assign the observed measurement to this 
    *   particular landmark.
    * NOTE: this method will NOT be called by the grading code. But you will 
    *   probably find it useful to implement this method and use it as a helper 
    *   during the updateWeights phase.
    */
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) 
{
    /**
    * TODO: Update the weights of each particle using a mult-variate Gaussian 
    *   distribution. You can read more about this distribution here: 
    *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    * NOTE: The observations are given in the VEHICLE'S coordinate system. 
    *   Your particles are located according to the MAP'S coordinate system. 
    *   You will need to transform between the two systems. Keep in mind that
    *   this transformation requires both rotation AND translation (but no scaling).
    *   The following is a good resource for the theory:
    *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    *   and the following is a good resource for the actual equation to implement
    *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
    */
    /*Look at other github code for checking*/
    /*Purpose:  To check whether it is needed to use normal_distribution<double> for observations from the car. 
	  Ans: The github code does contain normal_distribution<double> for observations from the car.
	  This may be due to standard deviation position x and y of a landmark are used in  Multivariate_normal_distribution*/
    
    double p_x, p_y, p_theta;

	double l_map_x, l_map_y;
	int l_map_id;
	
	double p_length;

    
    double std_car_x = std_landmark[0];
	double std_car_y = std_landmark[1];
	
	double p_car_x, p_car_y;
	double sum_weight;
	//int car_id;
	
	default_random_engine gen;
	
	int obs_size = observations.size();
	int map_size = map_landmarks.landmark_list.size();
	
	for(int p = 0; p < num_particles; p++)
    {
    	p_x = particles[p].x;
	    p_y = particles[p].y;
	    p_theta = particles[p].theta;
	    
	    double prob = 1.0;
	    
	    particles[p].associations.clear();
		particles[p].sense_x.clear();
		particles[p].sense_y.clear();
	    
	    for(int i = 0; i < obs_size ; i++)
		{
			
	        p_car_x =  observations[i].x;
	        p_car_y =  observations[i].y;
	        
	        double x_m, y_m;
		    x_m = cos(p_theta)*p_car_x - sin(p_theta)*p_car_y + p_x;
		    y_m = sin(p_theta)*p_car_x + cos(p_theta)*p_car_y + p_y;
	        
	        
	        double min_dis = std::numeric_limits<double>::infinity();
	        int min_id = 0;
	        double min_l_map_x, min_l_map_y;
	        
	        for(int i = 0; i < map_size ; i++)
			{
				//LandmarkObs newlandmarkobs;
			    l_map_x = map_landmarks.landmark_list[i].x_f;
		    	l_map_y = map_landmarks.landmark_list[i].y_f;
		    	l_map_id = map_landmarks.landmark_list[i].id_i;
		    	
		    	p_length = sqrt( pow ((l_map_x - x_m), 2.0) + pow ((l_map_y - y_m), 2.0) );
		    	
		    	if(p_length <=  sensor_range && min_dis > p_length)
		    	{
		    		min_dis = p_length;
					min_id = l_map_id;
					min_l_map_x = l_map_x;
					min_l_map_y = l_map_y;
				}
		    }	    
		    particles[p].associations.push_back(min_id);
		    particles[p].sense_x.push_back(min_l_map_x);
		    particles[p].sense_y.push_back(min_l_map_y);
		    
		    double sum = (pow((x_m-min_l_map_x),2)/(2.0*std_car_x*std_car_x)) + (pow((y_m-min_l_map_y),2)/(2.0*std_car_y*std_car_y)) ;
		    prob = prob*(1/(2.0*M_PI*std_car_x*std_car_y))*exp(-sum);	
		}
		particles[p].weight = prob;	
		sum_weight = sum_weight + particles[p].weight;
    }
	
    for(int p = 0; p < num_particles; p++)
    {
    	particles[p].weight = particles[p].weight/sum_weight;
	}
}

void ParticleFilter::resample() 
{
    /**
    * TODO: Resample particles with replacement with probability proportional 
    *   to their weight. 
    * NOTE: You may find std::discrete_distribution helpful here.
    *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    */
    
    /*Look at other github code for checking*/
	/*Purpose:  To check whether it is needed to implement resampling wheel method. 
	  Ans: Most of the solutions from github contain resampling wheel method but do not use discrete_distribution function from c++ std*/ 
	  
    default_random_engine gen;
    vector<double> particle_set;
    for(int p = 0; p < num_particles; p++) 
	{
        particle_set.push_back(particles[p].weight);
    }
    discrete_distribution<> part(particle_set.begin(), particle_set.end());    
    
	vector<Particle> newparticles;
    for(int p = 0; p < num_particles; p++)
	{
		newparticles.push_back(particles[part(gen)]);
	}
	particles = newparticles;
	
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) 
{
    // particle: the particle to which assign each listed association, 
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) 
{
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) 
{
    vector<double> v;

    if(coord == "X") 
	{
    	v = best.sense_x;
  	} 
	else 
	{
    	v = best.sense_y;
  	}

  	std::stringstream ss;
  	copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  	string s = ss.str();
  	s = s.substr(0, s.length()-1);  // get rid of the trailing space
  	return s;
}






