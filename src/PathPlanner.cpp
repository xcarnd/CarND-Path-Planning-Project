//
// Created by x on 17-9-10.
//
#include "helpers.h"
#include "PathPlanner.h"

#include <iostream>

using namespace std;

PathPlanner::PathPlanner()
: max_velocity (getMeterPerSecond(50)),
  planning_t(5),
  time_step(0.5),
  scales(4),
  num_samples(9)
{}

void
PathPlanner::get_path(double car_x, double car_y, double theta, double car_s, double car_d, double car_speed,
                      const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
                      std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals) {

	// predict the target goal after planning_t seconds
	

}
