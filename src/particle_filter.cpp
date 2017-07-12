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

/**
  * init Initializes particle filter by initializing particles to Gaussian
  *   distribution around first position and all the weights to 1.
  * @param x Initial x position [m] (simulated estimate from GPS)
  * @param y Initial y position [m]
  * @param theta Initial orientation [rad]
  * @param std[] Array of dimension 3 [standard deviation of x [m], 
  *   standard deviation of y [m], standard deviation of yaw [rad]]
  */
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  
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
  
  num_particles = 200; // number of system particles 
  
  // create each particle randomly distributed around the input data
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

/**
  * prediction Predicts the state for the next time step
  *   using the process model.
  * @param delta_t Time between time step t and t+1 in measurements [s]
  * @param std_pos[] Array of dimension 3 [standard deviation of x [m], 
  *   standard deviation of y [m], standard deviation of yaw [rad]]
  * @param velocity Velocity of car from t to t+1 [m/s]
  * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
  */
void ParticleFilter::prediction(double delta_t, double std_pos[], 
    double velocity, double yaw_rate) {
  
  // Standard deviations for x, y, and theta
  double std_x = std_pos[0]; 
  double std_y = std_pos[1]; 
  double std_theta = std_pos[2];
  double new_x, new_y, new_theta;
  
  default_random_engine gen;
  
  // predict each particle after the motion
  for (int par = 0; par < num_particles; par++) {
    double theta = particles[par].theta;
    
    // check yaw_rate to prevent divide by zero 
    // use a bicycle motion model to predict to vehicles movement
    if((yaw_rate < 0.0001) && (yaw_rate > -0.0001)) {
      // Xf = X0 + v(dt)(cos(theta0))
      new_x = particles[par].x + velocity*delta_t*cos(theta);
      new_y = particles[par].y + velocity*delta_t*sin(theta);
      new_theta = theta;   
    } else {
      double psi = theta + yaw_rate * delta_t;
      // X_f = X_0 + v/yaw_rate [sin(theta_0 + yaw_rate(dt) - sin(theta_0)]
      // X_f = X_0 + v/yaw_rate [cos(theta_0) - cos(theta_0 + yaw_rate(dt)]
      // theta_f = theta_0 + yaw_rate(dt)
      new_x = particles[par].x + (velocity/yaw_rate)*(sin(psi)-sin(theta));
      new_y = particles[par].y + (velocity/yaw_rate)*(cos(theta)-cos(psi));
      new_theta = psi;
    }
    
    // add Gaussian noise to the prediction to account for model uncertainty
    normal_distribution<double> dist_x(new_x, std_x);
    normal_distribution<double> dist_y(new_y, std_y);
    normal_distribution<double> dist_theta(new_theta, std_theta);
    
    particles[par].x = dist_x(gen);
    particles[par].y = dist_y(gen);
    particles[par].theta = dist_theta(gen);
  } 
}

/**
  * dataAssociation Finds which observations correspond to which landmarks 
  *   (using a nearest-neighbor distance association).
  * @param predicted Vector of predicted landmark observations
  * @param observations Vector of landmark observations
  */
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, 
    std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed 
  // measurement and assign the 
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will 
  //   probably find it useful to implement this method and use it as a helper
  //   during the updateWeights phase.
  
}

/**
  * updateWeights Updates the weights for each particle based on the likelihood  
  *   of the observed measurements. 
  * @param sensor_range Range [m] of sensor
  * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
  *   standard deviation of bearing [rad]]
  * @param observations Vector of landmark observations
  * @param map Map class containing map landmarks
  */
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

  // Standard deviations for x, y, and theta
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  
  for(int par=0; par < num_particles; par++) {
    double x_obs, y_obs;
    double p_theta = particles[par].theta;
    double p_x = particles[par].x; // particles x position
    double p_y = particles[par].y;
    
    // transform the detected obstacles readings from vehicle coordinates to
    // the world coordinates using rotation about the z-axis and translation.
    // http://planning.cs.uiuc.edu/node99.html equation 3.33
    // https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    vector<LandmarkObs> transform_obs;
    transform_obs.clear();
    for(int obs=0; obs < observations.size(); obs++) {
      x_obs = observations[obs].x;
      y_obs = observations[obs].y;
      
      LandmarkObs lm;
      // |---------      Rotation        ----------| Translation |
      // X(object)*cos(theta) - Y(object)*sin(theta) + X(particle)
      lm.x = x_obs * cos(p_theta) - y_obs * sin(p_theta) + p_x;
      // X(object)*sin(theta) + Y(object)*cos(theta) + Y(particle)
      lm.y = x_obs * sin(p_theta) + y_obs * cos(p_theta) + p_y;
      
      transform_obs.push_back(lm); 
    }
    
    particles[par].associations.clear();
    //particles[par].sense_x.clear();
    //particles[par].sense_y.clear();
    
    double dist, best_dist;
    bool isfirst;
    int best_lm=0; // tracks closest observation landmark

    for(int obs=0; obs < transform_obs.size(); obs++) {  
      isfirst = true;
    
      // find the current observations closest world landmark
      for(int mark=0; mark < map_landmarks.landmark_list.size(); mark++) {
        double x_diff = map_landmarks.landmark_list[mark].x_f - transform_obs[obs].x;
        double y_diff = map_landmarks.landmark_list[mark].y_f - transform_obs[obs].y;

        // find the error between sensor reading and landmark using Pythagoras   
        dist = sqrt(x_diff*x_diff + y_diff*y_diff);
        
        // check if the landmark is the closest to the sensor reading and store 
        if(isfirst || dist < best_dist) {
          isfirst = false;
          best_dist = dist;
          best_lm = mark;
        }
      }
      // store the closest landmark readings
      particles[par].associations.push_back(best_lm);
      //particles[par].sense_x.push_back(transform_obs[obs].x);
      //particles[par].sense_y.push_back(transform_obs[obs].y);
    }
    
    // update the particle weights based on all the sensor observations  
    particles[par].weight = 1;
    for(int i=0; i < transform_obs.size(); i++) {
      // calculate the sum of the particles weight using Multivariate-Gaussian 
      // probability
      // prob = n * exp(-(error_X + error_Y))
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
        // update the current particles weight
        particles[par].weight *= calc_weight;
      }
    }
    // update the weight vector containing each particles final weight
    weights[par] = particles[par].weight;
  }
}

/**
  * resample's the predicted and updated set of particles to form
  *   the new set of particles.
  */
void ParticleFilter::resample() {
  // copied from the "Self-Driving Car Project Q&A | Kidnapped Vehicle" video
  
  discrete_distribution<int> dist_weights(weights.begin(), weights.end());
  default_random_engine gen;
  
  // Resample particles with replacement based on the probability proportional 
  // to their weights. 
  vector<Particle> resampled_particles;
  for(int i=0; i < num_particles; i++) {
    resampled_particles.push_back(particles[dist_weights(gen)]);
  }
  particles = resampled_particles;
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
