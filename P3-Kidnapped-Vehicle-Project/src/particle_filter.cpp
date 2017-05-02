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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;
	normal_distribution<double> N_x_init(x, std[0]);
	normal_distribution<double> N_y_init(y, std[1]);
	normal_distribution<double> N_theta_init(theta, std[2]);

	num_particles = 1000;
	for (int i=0; i<num_particles; i++) {
		Particle p = Particle();
		p.id = i;
		p.x = N_x_init(gen);
		p.y = N_y_init(gen);
		p.theta = N_theta_init(gen);
		p.weight = 1;
		weights.push_back(1);
		particles.push_back(p);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;

	double n_x, n_y, n_theta, velocity_by_yaw_rate, next_theta;

	for (int i=0; i<particles.size(); i++) {

		velocity_by_yaw_rate = velocity/yaw_rate;
		next_theta = particles[i].theta + (yaw_rate * delta_t);
		if (fabs(yaw_rate) > 0.01) {
			particles[i].x += velocity_by_yaw_rate * ( sin(next_theta) - sin(particles[i].theta) );
			particles[i].y += velocity_by_yaw_rate * ( cos(particles[i].theta) - cos(next_theta) );
		} else {
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		particles[i].theta = next_theta;

		// Add random Gaussian noise
		normal_distribution<double> N_x(particles[i].x, std_pos[0]);
		normal_distribution<double> N_y(particles[i].y, std_pos[1]);
		normal_distribution<double> N_theta(particles[i].theta, std_pos[2]);
		n_x = N_x(gen);
		n_y = N_y(gen);
		n_theta = N_theta(gen);

		particles[i].x = particles[i].x + n_x;
		particles[i].y = particles[i].y + n_y;
		particles[i].theta = particles[i].theta + n_theta;
	}
}

vector<LandmarkObs> ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	// cout << predicted.size() << "<-Pred.size, Obs.size->" << observations.size() << endl;
	vector<LandmarkObs> associated_lm;
	for (int i=0; i<observations.size(); i++) {
		LandmarkObs obs = observations[i];
		double dt;
		double lowest_dt = INFINITY;
		int lowest_dt_idx = -1;

		for (int j=0; j<predicted.size(); j++) {
			LandmarkObs pred_lm = predicted[j];
			dt = dist(obs.x, obs.y, pred_lm.x, pred_lm.y);
			if (dt < lowest_dt) {
				lowest_dt = dt;
				lowest_dt_idx = j;
			}
		}
		if (predicted.size() > 0)
			associated_lm.push_back(predicted[lowest_dt_idx]);
	}
	return associated_lm;
}

inline vector<LandmarkObs> local_to_map_coordinates(vector<LandmarkObs>& observations, Particle& p) {
	vector<LandmarkObs> observations_map_coordinates;
	for (int i=0; i<observations.size(); i++) {
		LandmarkObs lo_g;
		lo_g.x = (observations[i].x * cos(p.theta)) + (observations[i].y * sin(p.theta)) + p.x;
		lo_g.y = (observations[i].x * sin(p.theta)) + (observations[i].y * cos(p.theta)) + p.y;
		lo_g.id = observations[i].id;
		observations_map_coordinates.push_back(lo_g);
	}
	return observations_map_coordinates;
}


inline double mvg(vector<LandmarkObs> obs, vector<LandmarkObs> lms, double std_landmark[]) {
	// cout << obs.size() << "<-obs size, lms.size -> " << lms.size() << endl;
	double p1, p2x, p2y, p2, p3, p4; 
	double tp = 1;
	if (lms.size() < obs.size()) {
		tp=0;
		return tp;
	}
	for (int a=0; a<obs.size(); a++) {
		p1 = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
		p2x = ((obs[a].x - lms[a].x)*(obs[a].x - lms[a].x)) / (2*std_landmark[0]*std_landmark[0]);
		p2y = ((obs[a].y - lms[a].y)*(obs[a].y - lms[a].y)) / (2*std_landmark[1]*std_landmark[1]);
		p2 = -1*(p2x+p2y);
		p3 = exp(p2);
		p4 = p1*p3;
		tp *= p4;
	}
	return tp;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

	for (int i=0; i<particles.size(); i++) {

		// Step-1 From the given map_landmarks, predict landmarks falling within sensor range
		vector<LandmarkObs> pred_map_lms;
		vector<Map::single_landmark_s> map_lms = map_landmarks.landmark_list;
		double in_range;
		for (int k=0; k<map_lms.size(); k++) {
			in_range = dist(particles[i].x, particles[i].y, map_lms[k].x_f, map_lms[k].y_f);
			if(sqrt(in_range) <= sensor_range) {
				LandmarkObs map_lm;
				map_lm.x = map_lms[k].x_f;
				map_lm.y = map_lms[k].y_f;
				map_lm.id = map_lms[k].id_i;
				pred_map_lms.push_back(map_lm);
			}
		}

		// Step-2: Transform Landmark observation from vehicle coordinate system to map coordinate system
		vector<LandmarkObs> observations_mc = local_to_map_coordinates(observations, particles[i]);

		// Step-3: Associate observations_mc with predicted map landmarks (in step-1)
		vector<LandmarkObs> associated_lms = dataAssociation(pred_map_lms, observations_mc);

		// Step-4: Calculate weight using multi variate gaussian probability density function
		double tp = mvg(observations_mc, associated_lms, std_landmark);

		particles[i].weight =  tp;
		weights[i] = particles[i].weight;
	} // End of particles for loop
} // End of updateWeights function

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	random_device rd;
	mt19937 gen(rd());
	//static default_random_engine gen;
	discrete_distribution<int> d(weights.begin(), weights.end());

	vector<Particle> new_particles;
	for (int i=0; i<num_particles; i++) {
		new_particles.push_back(particles[d(gen)]);
	}
	particles = new_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
