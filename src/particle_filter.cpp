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

        num_particles = 10;
        default_random_engine gen;

        // This line creates a normal (Gaussian) distribution for x.
            normal_distribution<double> dist_x(x, std[0]);

        // TODO: Create normal distributions for y and psi.
            normal_distribution<double> dist_y(y, std[1]);
            normal_distribution<double> dist_psi(theta, std[2]);
            for (int i = 0; i < num_particles; ++i) {
                    double sample_x, sample_y, sample_psi;

                    // TODO: Sample  and from these normal distrubtions like this:
                    // sample_x = dist_x(gen);
                    // where "gen" is the random engine
                    sample_x = dist_x(gen);
                    sample_y = dist_y(gen);
                    sample_psi = dist_psi(gen);


                    // Print your samples to the terminal.
                    //cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_psi << endl;
                    Particle par_tmp;
                    par_tmp.id = i;
                    par_tmp.x = sample_x;
                    par_tmp.y = sample_y;
                    par_tmp.theta = sample_psi;
                    par_tmp.weight = 1.0/float(num_particles);
                    weights.push_back(par_tmp.weight);

                    //cout << "x:" << par_tmp.x << " y:" << par_tmp.y << " weight:" << par_tmp.weight << endl;

                    particles.push_back(par_tmp);
            }

           self:is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    default_random_engine gen;
    
    for(int i=0;i<num_particles;i++){
        double x_pre = particles[i].x;
        double y_pre = particles[i].y;
        double theta_pre = particles[i].theta;
        
        if (yaw_rate<0.0001) {
            x_pre = x_pre+velocity*delta_t*cos(theta_pre);
            y_pre = y_pre+velocity*delta_t*sin(theta_pre);
        }else{
            x_pre=x_pre+velocity*(sin(theta_pre+yaw_rate*delta_t)-sin(theta_pre))/yaw_rate;
            y_pre=y_pre+velocity*(cos(theta_pre)-cos(theta_pre+yaw_rate*delta_t))/yaw_rate;
            theta_pre=theta_pre+yaw_rate*delta_t;
            
        }
        // This line creates a normal (Gaussian) distribution for x.
        normal_distribution<double> dist_x(x_pre, std_pos[0]);
        
        // TODO: Create normal distributions for y and psi.
        normal_distribution<double> dist_y(y_pre, std_pos[1]);
        normal_distribution<double> dist_psi(theta_pre, std_pos[2]);
        
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_psi(gen);
        
        
    }
    
    
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

        /* ////////////////// Parameters data explore
        for(int i=0;i<observations.size();i++){
            cout << "Test for see observations in dataAssociation function:" << endl;
            cout << "order:" << i << " id:" << observations[i].id << " x:" << observations[i].x << " y:" << observations[i].y << endl;

        }

        for(int j=0;j<predicted.size();j++){
            cout << "Test for see predicted vector in dataAssociation function:" << endl;
            cout << "order:" << j << " id:" << predicted[j].id << " x:" << predicted[j].x << " y:" << predicted[j].y << endl;
        }
        */ // finished exploration


        for(int i=0;i<observations.size();i++){
            double previous_dis = 100000.00;
            for(int j=0;j<predicted.size();j++){

                double distance = sqrt((observations[i].x-predicted[j].x)*(observations[i].x-predicted[j].x)+(observations[i].y-predicted[j].y)*(observations[i].y-predicted[j].y));
                if(distance<previous_dis){
                    observations[i].id = predicted[j].id;
                }

                previous_dis = distance;
            }



        }


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
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

        /* /////// Print out for first explore the data information
        cout << "sensor range:" << sensor_range <<endl;
        cout << "size of std lanmark 1:" << sizeof(std_landmark[0]) << endl;
        for(int j=0;j<sizeof(std_landmark);j++){
            cout << "std landmark " << j <<":" << std_landmark[j] << endl;
        }

        cout << "observation size:" << observations.size() << endl;
        for(int i=0;i<observations.size();i++){
            cout << "observations id:" << observations[i].id << endl;
            cout << "observations x:" << observations[i].x << endl;
            cout << "observations y:" << observations[i].y << endl;
        }

        cout << "map landmark list size:" << map_landmarks.landmark_list.size() << endl;
        */ // end explore the data /////////////////////

        // Start to do the transformation
        for(int i=0;i<num_particles;i++){
            double x_pra = particles[i].x;
            double y_pra = particles[i].y;
            double theta_pra = particles[i].theta;

            vector<LandmarkObs> trans_observation;
            LandmarkObs obs;
            for(int i=0;i<observations.size();i++){
                    LandmarkObs trans_obs;
                    obs = observations[i];

                    trans_obs.x = x_pra+(obs.x*cos(theta_pra)-obs.y*sin(theta_pra));
                    trans_obs.y = y_pra+(obs.x*sin(theta_pra)+obs.y*cos(theta_pra));
                    trans_observation.push_back(trans_obs);

                }

            // prepare for using dataAssociation functinon

            vector<LandmarkObs> predicted;
            for(int i=0;i<map_landmarks.landmark_list.size();i++){
                double x_l = map_landmarks.landmark_list[i].x_f;
                double y_l = map_landmarks.landmark_list[i].y_f;
                double distance = sqrt((x_pra-x_l)*(x_pra-x_l)+(y_pra-y_l)*(y_pra-y_l));
                if(distance<sensor_range){
                    LandmarkObs tmp;
                    tmp.id = map_landmarks.landmark_list[i].id_i;
                    tmp.x = x_l;
                    tmp.y = y_l;

                    predicted.push_back(tmp);
                }


            }

            dataAssociation(predicted,trans_observation);

            /* // for check data corect or not
            for(int k=0;k<trans_observation.size();k++){

                cout << "Test for see trans_ovservation :" << endl;
                cout << "order:" << k << " id:" << trans_observation[k].id << " x:" << trans_observation[k].x << " y:" << trans_observation[k].y << endl;

            }
            */ // Finished checking data

            // Updating the weight

            double sig_x = std_landmark[0];
            double sig_y = std_landmark[1];
            vector<double> weights;
            double final_weight=1.0;
            double x_obs;
            double y_obs;
            double mu_x;
            double mu_y;


            for(int k=0;k<trans_observation.size();k++){

                x_obs = trans_observation[k].x;
                y_obs = trans_observation[k].y;

                for(int i =0;i<predicted.size();i++){
                    if(predicted[i].id==trans_observation[k].id){
                        mu_x = predicted[i].x;
                        mu_y = predicted[i].y;
                    }



                }


            //cout << "sig x:" << sig_x << " sig_y:" << sig_y <<endl;
            //cout << "x_obs:" << x_obs << " y_obs:" << y_obs << endl;
            //cout << "mu_x:" << mu_x << " mu_y:" << mu_y << endl;

            // calculate normalization term
            double gauss_norm = (1/(2 * M_PI * sig_x * sig_y));

            // calculate exponent
            double exponent= (((x_obs - mu_x)*(x_obs - mu_x))/(2 * sig_x*sig_x)) + (((y_obs - mu_y)*(y_obs - mu_y))/(2 * sig_y*sig_y));

            // calculate weight using normalization terms and exponent
            double weight= gauss_norm * exp(-exponent);
            //cout << "----weight----" << weight << endl;
            if(weight > 0.0000001){
              weights.push_back(weight);
            }



            }

            for(int j=0;j<weights.size();j++){
                final_weight = final_weight*weights[j];
            }

            cout << "weight size" << weights.size() << "final weight:" << final_weight << endl;
            particles[i].weight=final_weight;


        }



}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // Set of current particles
    default_random_engine gen;
    std::vector<Particle> particles_resample;

    //collect all weights
    vector<double> weights;
    for(int i=0;i<num_particles;i++){
        weights.push_back(particles[i].weight);
    }

    random_device rd;
    discrete_distribution<> d(weights.begin(),weights.end());
    //map<int, int> m;
    for(int n=0;n<num_particles;++n){
        Particle particle_res = particles[d(gen)];
        particles_resample.push_back(particle_res);
    }

    particles = particles_resample;



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
