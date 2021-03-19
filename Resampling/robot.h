#ifndef ROBOT_H_
#define ROBOT_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

using namespace std;

class ROBOT {
public:
	ROBOT();
	virtual ~ROBOT();
	void Init(double world_size);
	void SetRobot(double new_x, double new_y, double new_ori, double world_size);
	void SetNoise(double f_noise, double t_noise, double s_noise);
	vector<double> sense(vector<vector<double>> landmark);
	ROBOT move(double turn, double forward, double world_size);
	double Gaussian(double mu, double sigma, double x);
	double measurement_prob(vector<double> measurement, vector<vector<double>> landmark);

	double x;
	double y;
	double orientaion;
	double forward_noise;
	double turn_noise;
	double sense_noise;
};

#endif