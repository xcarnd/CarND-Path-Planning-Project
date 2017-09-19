//
// Created by x on 17-9-10.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include <vector>
#include <random>

#include "helpers.h"

enum BehaviorState {
	/**
	 * Keep Lane
	 */
	KL,

	/**
	 * Lane Change Left
	 */
	LCL,

	/**
	 * Lane Change Right
	 */
	LCR
};

class PathPlanner {

private:

	/**
	 * map data for x
	 */
	std::vector<double> map_x;

	/**
	 * map data for y
	 */
	std::vector<double> map_y;

	/**
	 * map data for s
	 */
	std::vector<double> map_s;

	/**
	 * map data for d_x
	 */
	std::vector<double> map_d_x;

	/**
	 * map data for d_y
	 */
	std::vector<double> map_d_y;

	/**
	 * velocity limit in m/s
	 */
	const double max_velocity;

	/**
	 * time interval for path generation
	 */
	double pg_interval;

	/**
	 * reference speed
	 */
	double ref_v;

	/**
	 * current lane
	 */
	int current_lane;

	/**
	 * current state
	 */
	BehaviorState current_state;

	/**
	 * acceleration when keeping lane and current velocity is not at the maximum
	 */
	double a_keep_lane;

public:
	PathPlanner(const std::vector<double>& map_x, const std::vector<double>& map_y, const std::vector<double>& map_s,
	            const std::vector<double>& map_d_x, const std::vector<double>& map_d_y);

	void get_path(double car_x, double car_y, double theta,
	              double car_s, double car_d, double end_s, double end_d,
	              double car_speed,
	              const std::vector<std::vector<double>>& sensor_fusion,
	              const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
	              std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals);

};


#endif //PATH_PLANNING_PATHPLANNER_H
