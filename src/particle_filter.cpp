/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  // Standard deviations for x, y, and theta
  double std_x = std[0]; 
  double std_y = std[1]; 
  double std_theta = std[2];
  
  // create a normal Gaussian distribution from the input data to sample from 
  // for initialising each particle. 
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  num_particles = 100; 
  
  // create each random particle
  for (int par=0; par < num_particles; par++) {
    Particle particle;
    particle.id = par;
    particle.x = dist_x(gen);
    particle.y = dist_y(gen);
	particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    
    particles.push_back(particle);
    weights.push_back(1.0);
  }
  
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  // Standard deviations for x, y, and theta
  double std_x = std_pos[0]; 
  double std_y = std_pos[1]; 
  double std_theta = std_pos[2];
  
  default_random_engine gen;
  
  // predict each particle after the motion
  for (int par = 0; par < num_particles; par++) {
    double new_x, new_y, new_theta;
    
    if(yaw_rate == 0){
      new_x = particles[par].x + velocity*delta_t*cos(particles[par].theta);
      new_y = particles[par].y + velocity*delta_t*sin(particles[par].theta);
      new_theta = particles[par].theta;   
    } else {
      new_x = particles[par].x + (velocity/yaw_rate)*(sin(particles[par].theta+yaw_rate*delta_t)-sin(particles[par].theta));
      new_y = particles[par].y + (velocity/yaw_rate)*(cos(particles[par].theta)-cos(particles[par].theta+yaw_rate*delta_t));
      new_theta = particles[par].theta + yaw_rate*delta_t;
    }
    
    normal_distribution<double> dist_x(new_x, std_x);
    normal_distribution<double> dist_y(new_y, std_y);
    normal_distribution<double> dist_theta(new_theta, std_theta);
    
    particles[par].x = dist_x(gen);
    particles[par].y = dist_y(gen);
    particles[par].theta = dist_theta(gen);
  } 
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  //cout << "ParticleFilter::dataAssociation" << endl;
  
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  
  for(int par=0; par < num_particles; par++) {
    double x_obs, y_obs;
    double p_theta = particles[par].theta;
    double p_x = particles[par].x;
    double p_y = particles[par].y;
    
    // transform the obstacle readings to the world coordinates
    vector<LandmarkObs> transform_obs;
    transform_obs.clear();
    for(int obs=0; obs < observations.size(); obs++) {
      x_obs = observations[obs].x;
      y_obs = observations[obs].y;
      
      LandmarkObs lm;
      //X*cos(theta) - Y*sin(theta) + Xshift
      lm.x = x_obs * cos(p_theta) - y_obs * sin(p_theta) + p_x;
      // X*sin(theta) + Y*cos(theta) + Yshift
      lm.y = x_obs * sin(p_theta) + y_obs * cos(p_theta) + p_y;
      
      transform_obs.push_back(lm); 
    }
    
    particles[par].associations.clear();
    particles[par].sense_x.clear();
    particles[par].sense_y.clear();
    
    double dist, best_dist;
    int best_lm=0; // tracks closest observation landmark
    for(int obs=0; obs < transform_obs.size(); obs++) {  
      best_dist = 1000;
      best_lm = 0;
      
      for(int mark=0; mark < map_landmarks.landmark_list.size(); mark++) {
        double x_diff = transform_obs[obs].x - map_landmarks.landmark_list[mark].x_f;
        double y_diff = transform_obs[obs].y - map_landmarks.landmark_list[mark].y_f;

        dist = sqrt(x_diff * x_diff + y_diff * y_diff);
        if(dist < best_dist) {
          // store the current landmark as the closest landmark to the observation
          best_lm = mark;
          best_dist = dist;
        } 
      }
      particles[par].associations.push_back(best_lm);
      particles[par].sense_x.push_back(transform_obs[best_lm].x);
      particles[par].sense_y.push_back(transform_obs[best_lm].y);
    }
    
    particles[par].weight = 1;
    for(int i=0; i < transform_obs.size(); i++) {
      // calculate the sum of the particles weight using Multivariate-Gaussian probability
      // n * exp(-(error_X + error_Y))
      // n = 1 / (2 * PI * std_x * std_y)
      // error_X = (observed_X - landmark_X) ^ 2 / (2 * std_x ^ 2)
      // error_Y = (observed_Y - landmark_Y) ^ 2 / (2 * std_y ^ 2)
      double x_diff = transform_obs[i].x - map_landmarks.landmark_list[particles[par].associations[i]].x_f;
      double y_diff = transform_obs[i].y - map_landmarks.landmark_list[particles[par].associations[i]].y_f;
      double x_error = (x_diff * x_diff) / (2 * std_x * std_x);
      double y_error = (y_diff * y_diff) / (2 * std_y * std_y);
      double normalise = 1 / (2 *  M_PI * std_x * std_y);
      double calc_weight = normalise * exp(-(x_error + y_error));
      
      if(calc_weight > 0) {
        particles[par].weight *= calc_weight;
      }
    }

    weights[par] = particles[par].weight;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  discrete_distribution<int> dist_weights(weights.begin(), weights.end());
  default_random_engine gen;
  
  vector<Particle> new_particles;
  //new_particles.reserve(num_particles);

  for(int i=0; i < num_particles; i++) {
    new_particles.push_back(particles[dist_weights(gen)]);
  }
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
