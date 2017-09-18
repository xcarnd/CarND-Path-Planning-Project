//
// Created by x on 17-9-10.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include <vector>
#include <random>

#include "helpers.h"

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
	double max_velocity;

	/**
	 * in what time span the planner will plan the trajectory, in seconds
	 */
	double planning_t;

	/**
	 * resolution for time step.
	 */
	double time_step;

	/**
	 * scales for t component when doing goals sampling.
	 *
	 * t component for goals will lie in [planning_t - scales * time_step, planning_t + scales * time_step]
	 */
	int scales;

	/**
	 * how many samples for one t component when doing goals sampling
	 */
	int num_samples;

	/**
	 * sigma for s component when doing random sampling.
	 */
	double sample_sigma_s;

	/**
	 * sigma for d component when doing random sampling.
	 */
	double sample_sigma_d;

	/**
	 * time interval for path generation
	 */
	double pg_interval;

	/**
	 * reference speed
	 */
	double ref_v;

	/**
	 * acceleration when keeping lane and current velocity is not at the maximum
	 */
	double a_keep_lane;

	/**
	 * random number engine
	 */
	std::default_random_engine e;

private:
	Polynomial jmt(const std::vector<double>& start, const std::vector<double>& end, double t);

	void generate_xy_trajectory(const Trajectory& trajectory,
	                            std::vector<double>& out_x, std::vector<double>& out_y, std::size_t maximum_points = 50);

public:
	PathPlanner(const std::vector<double>& map_x, const std::vector<double>& map_y, const std::vector<double>& map_s,
	            const std::vector<double>& map_d_x, const std::vector<double>& map_d_y);

	void get_path(double car_x, double car_y, double theta,
	              double car_s, double car_d, double end_s, double end_d,
	              double car_speed,
	              const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
	              std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals);
};


#endif //PATH_PLANNING_PATHPLANNER_H
