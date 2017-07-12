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
  cout << "ParticleFilter::init" << endl;
  
  // Standard deviations for x, y, and theta
  double std_x = *std; 
  double std_y = *(std+1);
  double std_theta = *(std+2); 
  
  // create a normal Gaussian distribution from the input data to sample from 
  // for initialising each particle. 
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  num_particles = 100; 
  particles.reserve(num_particles); // set the size of the particle vector
  weights.reserve(num_particles); // vector of each particles weight
  
  // create each random particle
  for (int i = 0; i < num_particles; i++) {
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
	particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
    
    weights[i] = 1.0;
  }
  
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  cout << "ParticleFilter::prediction" << endl;
  
  // Standard deviations for x, y, and theta
  double std_x = *std_pos; 
  double std_y = *(std_pos + 1); 
  double std_theta = *(std_pos + 2);
  
  default_random_engine gen;
  
  //cout << "std_x " << std_x << "  std_y " << std_y << "    t " << std_theta << endl;
  
  double vel, yaw_dt, new_x, new_y, new_theta, theta;
  // predict each particle after the motion
  for (int i = 0; i < num_particles; i++) {
    if(yaw_rate == 0){
      new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      new_theta = particles[i].theta;   
    } else {
      //yaw_dot = yaw_rate + dist_yaw_rate(gen);
      //yaw_dt = yaw_rate * delta_t;
      //vel = velocity / yaw_rate;
      theta = particles[i].theta;
      
      new_x = particles[i].x + (velocity / yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
      new_y = particles[i].y + (velocity / yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t)));
      new_theta = theta + (yaw_rate * delta_t);
    }
    
    normal_distribution<double> dist_x(new_x, std_x);
    normal_distribution<double> dist_y(new_y, std_y);
    normal_distribution<double> dist_theta(new_theta, std_theta);
    
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    
    //cout << "x " << particles[i].x << "    y " << particles[i].y << "     t " << particles[i].theta << endl;
  } 
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  cout << "ParticleFilter::dataAssociation" << endl;
  
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
  cout << "ParticleFilter::updateWeights " << endl;
      
  double std_x = *std_landmark;
  double std_y = *(std_landmark + 1);
  
  cout << "std_x " << std_x << "  std_y " << std_y << endl;

  for(int i=0; i < num_particles; i++) {
    double x_obs, y_obs, x_obj_pos, y_obj_pos;
    double p_theta = particles[i].theta;
    double p_x = particles[i].x;
    double p_y = particles[i].y;
    
    particles[i].weight = 1.0; // reset the particles weight
    particles[i].associations.reserve(observations.size());
    
    for(int obs=0; obs < observations.size(); obs++) {
      x_obs = observations[obs].x;
      y_obs = observations[obs].y;
      
      // X*cos(theta) - Y*sin(theta) + Xshift
      x_obj_pos = x_obs * cos(p_theta) - y_obs * sin(p_theta) + p_x;
      // X*sin(theta) + Y*cos(theta) + Yshift
      y_obj_pos = x_obs * sin(p_theta) + y_obs * cos(p_theta) + p_y;
      
      // find the closest landmark for the particles observation
      int best_landmark = 0; // tracks closest observation landmark
      double dist, best_dist = 100;
      
      for(int mark=0; mark < map_landmarks.landmark_list.size(); mark++) {
        double x_diff = x_obj_pos - map_landmarks.landmark_list[mark].x_f;
        double y_diff = y_obj_pos - map_landmarks.landmark_list[mark].y_f;
        
        dist = sqrt(x_diff * x_diff + y_diff * y_diff);
        if(dist < best_dist) {
          // store the current landmark as the closest landmark to the observation
          best_landmark = mark;
          best_dist = dist;
        } 
      }
      //cout << "best_landmark " << best_landmark << endl;
      particles[i].associations[obs] = best_landmark;
      
      // calculate the sum of the particles weight using Multivariate-Gaussian probability
      // n * exp(-(error_X + error_Y)
      // n = 1 / (2 * PI * std_x * std_y)
      // error_X = (observed_X - landmark_X) ^ 2 / (2 * std_x ^ 2)
      // error_Y = (observed_Y - landmark_Y) ^ 2 / (2 * std_y ^ 2)
      double x_diff = x_obj_pos - map_landmarks.landmark_list[best_landmark].x_f;
      double y_diff = y_obj_pos - map_landmarks.landmark_list[best_landmark].y_f;
      
      double x_error = (x_diff * x_diff) / (2 * std_x * std_x);
      double y_error = (y_diff * y_diff) / (2 * std_y * std_y);
      
      double normalise = 1 / (2 *  M_PI * std_x * std_y);
      
      particles[i].weight *= normalise * exp(-1.0 * (x_error + y_error));
    }
    weights[i] = particles[i].weight;
    
    //cout << "x " << particles[i].weight << "     t " << weights[i] << endl;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  cout << "ParticleFilter::resample" << endl;
  
  discrete_distribution<int> dist_weights(weights.begin(), weights.end());
  default_random_engine gen;
  
  vector<Particle> new_particles;
  //new_particles.reserve(num_particles);

  for(int i=0; i < num_particles; i++) {
    new_particles.push_back(particles[dist_weights(gen)]);
  }
  particles = new_particles;
  
  //cout << particles[0].weight << endl;
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
