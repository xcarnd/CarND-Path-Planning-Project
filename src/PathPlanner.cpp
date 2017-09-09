//
// Created by x on 17-9-10.
//
#include "helpers.h"
#include "PathPlanner.h"

using namespace std;

void
PathPlanner::get_path(double car_x, double car_y, double theta, std::vector<double> &next_x_vals,
                      std::vector<double> &next_y_vals) {

	double dist_inc = 0.5;

	for (int i = 0; i < 50; ++i) {
		next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(theta)));
		next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(theta)));
	}

}
