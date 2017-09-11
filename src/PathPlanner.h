//
// Created by x on 17-9-10.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include <vector>

class PathPlanner {

private:
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

public:
	PathPlanner();
	void get_path(double car_x, double car_y, double theta,
	              double car_s, double car_d,
	              double car_speed,
	              const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
	              std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals);
};


#endif //PATH_PLANNING_PATHPLANNER_H
