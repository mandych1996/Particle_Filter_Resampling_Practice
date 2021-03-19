#include <iostream>
#include <vector>
#include <algorithm>
#include "robot.h"


double world_size = 100.0;

vector<vector<double>> landmark = {
	{20.0, 20.0},
	{80.0, 80.0},
	{20.0, 80.0},
	{80.0, 20.0}
};


double random_(double world_size) {
	double v1 = rand() % 10000;
	v1 = v1 / 10000;
	v1 = v1 * world_size;
	return v1;
}

int main() {
	ROBOT myrobot = ROBOT();
	myrobot.Init(world_size);
	cout << myrobot.x << endl;

	//myrobot.move(0.1, 5,0);
	//cout << myrobot.x << endl;

	vector <double> Z = myrobot.sense(landmark);
	cout << Z[0] << ", " << Z[1] << ", " << Z[2] << endl;

	int N = 1000;
	vector<ROBOT> p;
	vector<double> w;

	for (int i = 0; i < N; i++) {
		ROBOT r = ROBOT();
		r.Init(world_size);
		r.SetNoise(0.05, 0.05, 5.0);
		//r.move(0.1, 5.0, world_size);
		//cout << r.x << r.y << r.orientaion << endl;
		p.push_back(r);
		w.push_back(r.measurement_prob(Z, landmark));
	}

	/*
	vector<ROBOT> p3;
	int index = rand() % N;
	double beta = 0.0;
	double mw = *max_element(w.begin(), w.end());
	for (int i = 0; i < N; i++) {
		beta += 2.0* random_(mw);
		while (beta > w[index]) {
			beta -= w[index];
			index = (index + 1) % N;
		}
		p3.push_back(p[index]);
	}
	p = p3;
	for (int i = 0; i < p.size(); i++) {
		cout << p[i].x << p[i].y << p[i].orientaion << endl;
	}
	*/

	system("pause");
	return 0;
}
