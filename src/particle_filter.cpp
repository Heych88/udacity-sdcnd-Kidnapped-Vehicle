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
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  
  num_particles = 100; 
  default_random_engine gen;
  particles.reserve(num_particles); // set the size of the particle vector
  
  // create each random particle
  for (int i = 0; i < num_particles; ++i) {
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
	particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
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
  double std_vel = *std_pos; 
  double std_yaw_rate = *(std_pos+2);
  
  normal_distribution<double> dist_vel(velocity, std_vel);
  normal_distribution<double> dist_yaw_rate(yaw_rate, std_yaw_rate);
  default_random_engine gen;
  
  double vel, yaw_dot, yaw_dt, x, y, theta;
  // predict each particle after the motion
  for (int i = 0; i < num_particles; ++i) {
    if(yaw_rate == 0){
      vel = (velocity + dist_vel(gen)) * delta_t;
      particles[i].x += vel * cos(particles[i].theta);
      particles[i].y += vel * sin(particles[i].theta);
      //particles[i].theta = particles[i].theta;   
    } else {
      yaw_dot = yaw_rate + dist_yaw_rate(gen);
      yaw_dt = yaw_dot * delta_t;
      vel = (velocity + dist_vel(gen)) / yaw_dot;
      x = particles[i].x;
      y = particles[i].y;
      theta = particles[i].theta;
      
      particles[i].x += vel * (sin(theta + yaw_dt) - sin(theta));
      particles[i].y += vel * (cos(theta) - cos(theta + yaw_dt));
      particles[i].theta += yaw_dt;
    }
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
  cout << "ParticleFilter::updateWeights" << endl;
  
  
  
  /*def Gaussian(self, mu, sigma, x):
  # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
  return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2)) 
   * 
   * def measurement_prob(self, measurement):
  # calculates how likely a measurement should be
  prob = 1.0;
  for i in range(len(landmarks)):
  dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
  prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
  return prob*/
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  cout << "ParticleFilter::resample" << endl;
  
  discrete_distribution<int> dist_weights(weights.begin(), weights.end());
  default_random_engine gen;
  
  vector<Particle> reshaped_particles;
  reshaped_particles.reserve(num_particles);

  for(int i=0; i < num_particles; ++i) {
    reshaped_particles[dist_weights(gen)];
  }
  particles = reshaped_particles;
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
