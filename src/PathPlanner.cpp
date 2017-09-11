//
// Created by x on 17-9-10.
//
#include "helpers.h"
#include "PathPlanner.h"

#include "Eigen-3.3/Eigen/Dense"

#include <iostream>

using namespace std;

PathPlanner::PathPlanner(const std::vector<double>& map_x, const std::vector<double>& map_y)
	: map_x(map_x), map_y(map_y),
	max_velocity(getMeterPerSecond(50)),
	planning_t(5),
	time_step(0.5),
	scales(4),
	num_samples(9),
	sample_sigma_s(10),
	sample_sigma_d(1),
	pg_interval(0.2),
	e(std::default_random_engine())
{}

void
PathPlanner::get_path(double car_x, double car_y, double theta, double car_s, double car_d, double car_speed,
	const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
	std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals) {

	// predict the target goal after planning_t seconds
	vector<double> start_s;
	vector<double> start_d;
	double v_s;
	double a_s;
	double v_d;
	double a_d;
	if (path_x_vals.size() > 2) {
		vector<double> path1_sd = getFrenet(path_x_vals[0], path_y_vals[0], theta, map_x, map_y);
		double p1_s = path1_sd[0];
		double p1_d = path1_sd[1];
		double p1_theta = atan2((path_y_vals[0] - car_y), (path_x_vals[0] - car_x));

		vector<double> path2_sd = getFrenet(path_x_vals[1], path_y_vals[1], p1_theta, map_x, map_y);
		double p2_s = path2_sd[0];
		double p2_d = path2_sd[1];

		double v1_s = (p1_s - car_s) / pg_interval;
		double v1_d = (p1_d - car_d) / pg_interval;

		double v2_s = (p2_s - p1_s) / pg_interval;
		double v2_d = (p2_d - p1_d) / pg_interval;

		a_s = (v2_s - v1_s) / pg_interval;
		a_d = (v2_d - v1_d) / pg_interval;

		v_s = v2_s;
		v_d = v2_d;

		start_s = { p2_s, v_s, a_s };
		start_d = { p2_d, v_d, a_d };
	}
	else {
		// we are at the very beginning. how about giving the vehicle an 1m/s^2 s acceleration and 0m/s^2 d acceleration?
		start_s = { 0.0, 0.0, 0.0 };
		start_d = { 0.0, 0.0, 0.0 };
		v_s = 0;
		a_s = 1;
		v_d = 0;
		a_d = 0;
	}

	// suppose the acceleration is the same in goal state
	// so: v_t = v_0 + a * t, s_t = v_0 * t + 1/2 * a * t^2 (the same for the d component)
	vector<vector<double>> trajectories_s;
	vector<vector<double>> trajectories_d;
	vector<double> costs;
	for (int i = -scales; i <= scales; ++i) {
		double t = planning_t + i * time_step;
		double v_s_end = v_s + a_s * t;
		double s_end = v_s * t + 0.5 * a_s * t * t;
		double v_d_end = v_d + a_d * t;
		double d_end = v_d * t + 0.5 * a_d * t * t;
		vector<double> goal_s = { s_end, v_s_end, a_s };
		vector<double> goal_d = { d_end, v_d_end, a_d };

		vector<double> traj_s;
		vector<double> traj_d;
		const vector<double>& coeffs_s = jmt(start_s, goal_s, t);
		const vector<double>& coeffs_d = jmt(start_d, goal_d, t);
		trajectories_s.push_back(generate_trajectory(coeffs_s, t));
		trajectories_d.push_back(generate_trajectory(coeffs_d, t));

		std::normal_distribution<double> distribution_s(s_end, sample_sigma_s);
		std::normal_distribution<double> distribution_d(d_end, sample_sigma_d);
		for (int j = 0; j < num_samples; ++j) {
			vector<double> g_s = { distribution_s(e), v_s_end, a_s};
			vector<double> g_d = { distribution_d(e), v_d_end, a_d};
			vector<double> traj_s;
			vector<double> traj_d;
			const vector<double>& coeffs_s = jmt(start_s, goal_s, t);
			const vector<double>& coeffs_d = jmt(start_d, goal_d, t);
			trajectories_s.push_back(generate_trajectory(coeffs_s, t));
			trajectories_d.push_back(generate_trajectory(coeffs_d, t));
		}
	}

}

// calculating jerk minimizing trajectory
std::vector<double> PathPlanner::jmt(const std::vector<double>& start, const std::vector<double>& end, double t) {
	vector<double> coeffs(6, 0.0);
	coeffs[0] = start[0];
	coeffs[1] = start[1];
	coeffs[2] = 1 / 2 * start[2];

	double t2 = t * t;
	double t3 = t2 * t;
	double t4 = t2 * t2;
	double t5 = t3 * t2;
	Eigen::MatrixXd M(3, 3);
	M <<    t3,      t4,      t5,
		3 * t2,  4 * t3,  5 * t4,
		6 * t , 12 * t2, 20 * t3;
	Eigen::VectorXd u(3);
	u << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
		 end[1] - (start[1] + start[2] * t),
		 end[2] -  start[2];
	Eigen::VectorXd v = M.inverse() * u;
	coeffs[3] = v[0];
	coeffs[4] = v[1];
	coeffs[5] = v[2];

	return coeffs;
}

std::vector<double> PathPlanner::generate_trajectory(const std::vector<double>& coeffs, double t)
{
	std::vector<double> result(int(ceil(t / pg_interval)));
	Polynomial poly(coeffs);
	double t_v = 0.0;
	while (t_v < t) {
		result.push_back(poly(t));
		t_v += pg_interval;
	}
	return result;
}
