#include "robot.h"
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <random>

#define M_PI 3.1415926

ROBOT::ROBOT() {}
ROBOT::~ROBOT() {}

double random(double world_size) {
	double v1 = rand() % 10000;
	v1 = v1 / 10000;
	v1 = v1 * world_size;
	return v1;
}

void ROBOT::Init(double world_size) {
	this->x = random(world_size);
	this->y = random(world_size);
	this->orientaion = random(2.0*M_PI);
	this->forward_noise = 0.0001;
	this->turn_noise = 0.0001;
	this->sense_noise = 0.0001;
}

void ROBOT::SetRobot(double new_x, double new_y, double new_ori, double world_size) {
	if (new_x < 0 || new_x >= world_size) throw "X coordinate out of bound";
	if (new_y < 0 || new_y >= world_size) throw "Y coordinate out of bound";
	if (new_ori < 0 || new_ori >= world_size) throw "Orientation must be in [0,2pi]";
	this->x = new_x;
	this->y = new_y;
	this->orientaion = new_ori;
}

void ROBOT::SetNoise(double f_noise, double t_noise, double s_noise){
	this->forward_noise = float(f_noise);
	this->turn_noise = float(t_noise);
	this->sense_noise = float(s_noise);
}

vector<double> ROBOT::sense(vector<vector<double>> landmark) {
	vector<double> Z;
	for (int i = 0; i < landmark.size(); i++) {
		double dist = sqrt(pow(x - landmark[i][0], 2) + pow(y - landmark[i][1], 2));
		default_random_engine generator;
		normal_distribution<double> distance_distribution(0.0, this->sense_noise);
		double d = distance_distribution(generator);
		dist += d;
		Z.push_back(dist);
	}
	return Z;
}

ROBOT ROBOT::move(double turn, double forward, double world_size) {
	if (forward < 0) throw "ValueError, 'Robot cant move backwards'";
	default_random_engine generator;
	normal_distribution<double> theta_distribution(0.0, this->turn_noise);
	double theta = theta_distribution(generator);
	double ori = this->orientaion + turn + theta;
	while (ori > 2*M_PI) ori -= 2*M_PI;
	while (ori < 0) ori += 2 * M_PI;

	normal_distribution<double> distance_distribution(0.0, this->forward_noise);
	double d = distance_distribution(generator);
	double dist = forward + d;
	double x = this->x + cos(ori)*dist;
	double y = this->y + sin(ori)*dist;
	while (x > world_size) x -= world_size;
	while (x < 0) x += world_size;
	while (y > world_size) y -= world_size;
	while (y < 0) y += world_size;

	ROBOT res = ROBOT();
	res.SetRobot(x, y, ori, world_size);
	res.SetNoise(this->forward_noise, this->turn_noise, this->sense_noise);
	return res;
}

double ROBOT::Gaussian(double mu, double sigma, double x) {
	return exp(-((mu-x)*(mu-x)) /(sigma*sigma) /2.0 /sqrt(2.0*M_PI*(sigma*sigma)));
}

double ROBOT::measurement_prob(vector<double> measurement,
	vector<vector<double>> landmark) {
	double prob = 1.0;
	for (int i = 0; i < landmark.size(); i++) {
		double dist = sqrt(pow(x - landmark[i][0], 2) + pow(y - landmark[i][1], 2));
		prob *= Gaussian(dist, this->sense_noise, measurement[i]);
	}
	return prob;
}

