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
#include <assert.h>

#include "particle_filter.h"
#include "helper_functions.h"

#include "math.h"

using namespace std;

//constants: might be good to include them in particle_filter.h so that they could be tuned outside the class
//(in main.cpp for ex)
const int N = 1000;
const double EPSILON = 1E-5;

//helper functions
double coef(double x, double ux, double std_x) {
    return pow(x-ux,2)/(2*pow(std_x,2));
}

double gauss(double x, double y, double ux, double uy, double std_x, double std_y) {
    double c = 1/(2*M_PI*std_x*std_y);
    
    double val = coef(x, ux, std_x) + coef(y, uy, std_y);
    
    return c * exp(-val);
}
//end helper functions


void ParticleFilter::init(double x, double y, double theta, double std[]) {
    default_random_engine gen;

    //Create normal distributions for each dimension
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_psi(theta, std[2]);

    num_particles = N;

    for (int i = 0; i < num_particles; i++) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_psi(gen);

        p.weight = 1;

        particles.push_back(p);
        weights.push_back(1);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    default_random_engine gen;

    for (int i = 0; i < num_particles; i++) {
        Particle part = particles[i];
        double xf, yf, thetaf;

        //prediction
        if (fabs(yaw_rate) < EPSILON) { //yawrate ~ 0
            xf = part.x + velocity*delta_t*cos(part.theta);
            yf = part.y + velocity*delta_t*sin(part.theta);
            thetaf = part.theta;
        } else {
            xf = part.x + (velocity/yaw_rate) * (sin(part.theta + yaw_rate*delta_t) - sin(part.theta));
            yf = part.y + (velocity/yaw_rate) * (cos(part.theta) - cos(part.theta + yaw_rate*delta_t));
            thetaf = part.theta + yaw_rate*delta_t;
        }

        //Create normal distributions for each dimension
        normal_distribution<double> dist_x(xf, std_pos[0]);
        normal_distribution<double> dist_y(yf, std_pos[1]);
        normal_distribution<double> dist_psi(thetaf, std_pos[2]);

        //add noise
        xf = dist_x(gen);
        yf = dist_y(gen);
        thetaf = dist_psi(gen);

        particles[i].x = xf;
        particles[i].y = yf;
        particles[i].theta = thetaf;

    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

    for (int i = 0; i < observations.size(); i++) {
        int min_idx = -1;
        double min_dist = 0;

        LandmarkObs obs = observations[i];
        for (int j = 0; j < predicted.size(); j++) {
            LandmarkObs pred = predicted[j];

            double d = dist(obs.x, obs.y, pred.x, pred.y);
            if (min_idx < 0 || d < min_dist) {
                min_dist = d;
                min_idx = j;
            }
        }

        assert(min_idx > -1);

        observations[i].id = predicted[min_idx].id;
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
    double sum_weights = 0.0;
    std::vector<double> new_weights;

    for (int i = 0; i < num_particles; i++) {
        // Transform observations from vehicle's coordinate system into the map's one
        Particle part = particles[i];
        std::vector<LandmarkObs> observations_map;

        for (int j = 0; j < observations.size(); j++) {
            double obs_x = observations[j].x;
            double obs_y = observations[j].y;

            double ang = -part.theta;

            double x = obs_x*cos(ang) + obs_y*sin(ang) + part.x;
            double y = -obs_x*sin(ang) + obs_y*cos(ang) + part.y;

            LandmarkObs landObs;
            landObs.x = x;
            landObs.y = y;

            observations_map.push_back(landObs);
        }

        //filter landmarks within sensor range from the particle
        std::vector<LandmarkObs> predicted;
        for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
            Map::single_landmark_s landmark = map_landmarks.landmark_list[k];

            double d = dist(part.x, part.y, landmark.x_f, landmark.y_f);

            if (d <= sensor_range) {
                LandmarkObs obs;
                obs.x = landmark.x_f;
                obs.y = landmark.y_f;
                obs.id = landmark.id_i;
                predicted.push_back(obs);
            }
        }

        //find which observation corresponds to which landmark
        dataAssociation(predicted, observations_map);

        double p = 1.0; //asumes at least one observation

        assert(observations_map.size() > 0); //if the particle contains no landamarks nearby, what to do?
        //it might mean the initialization was way off the map, or that the vehicle is eventually lost (run away from the
        // marked territory)

        for (int l = 0; l < observations_map.size(); l++) {
            LandmarkObs lobs = observations_map[l];
            Map::single_landmark_s associated_landmark = map_landmarks.landmark_list[lobs.id-1];
            assert (associated_landmark.id_i == lobs.id);

            double val = gauss(lobs.x, lobs.y, associated_landmark.x_f, associated_landmark.y_f, std_landmark[0], std_landmark[1]);

            p *= val;
        }

        new_weights.push_back(p);
        sum_weights += p;
    }

    for (int i = 0; i < num_particles; i++) {
        new_weights[i] /= sum_weights;
    }

    weights = new_weights;

}

void ParticleFilter::resample() {
    default_random_engine gen;
    discrete_distribution<> dd(weights.begin(), weights.end());

    std::vector<Particle> new_particles;
    for (int i = 0; i < num_particles; i++) {
        Particle p;
        int idx = dd(gen);
        p.x = particles[idx].x;
        p.y = particles[idx].y;
        p.theta = particles[idx].theta;
        p.id = i;

        new_particles.push_back(p);
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
