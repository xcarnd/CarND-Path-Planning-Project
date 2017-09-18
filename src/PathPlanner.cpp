//
// Created by x on 17-9-10.
//
#include "consts.h"
#include "helpers.h"
#include "PathPlanner.h"

#include "Eigen-3.3/Eigen/Dense"

#include <iostream>
#include <algorithm>
#include <numeric>

#include "cost_functions.h"

#include "spline.h"

using namespace std;
using namespace AlgebraX;

namespace {

	std::vector<WeightedCostFunction> ALL_COST_FUNCTIONS = {
		{ "S", s_diff_cost, 1.0 },
		{ "D", d_diff_cost, 20.0 },
		{ "MAX_V", max_velocity_cost, 50.0 },
		{ "MAX_ACCEL", max_acceleration_cost, 50.0 },
		{ "MAX_JERK", max_jerk_cost, 50.0 }
	};

	class CostCalculator {
	private:
		const std::vector<double>& goal_s;
		const std::vector<double>& goal_t;
	public:
		inline CostCalculator(const std::vector<double>& goal_s, const std::vector<double>& goal_t)
			: goal_s(goal_s), goal_t(goal_t) {}

		inline double operator() (const Trajectory& trajectory, bool verbose = false) const {
			double total = 0.0;
			auto iter = ALL_COST_FUNCTIONS.begin();
			while (iter != ALL_COST_FUNCTIONS.end()) {
				double cost = iter->weight * iter->f(trajectory, goal_s, goal_t);
				if (verbose) {
					std::cout<<"Cost function "<< iter->name << ": " << cost << endl;
				}
				total += cost;
				++iter;
			}
			return total;
		}
	};

	inline int get_lane_no(double d) {
		return static_cast<int>(std::floor(d / LANE_WIDTH));
	}

	inline double get_lane_center(int lane) {
		return lane * LANE_WIDTH + LANE_WIDTH / 2.0;
	}

}

PathPlanner::PathPlanner(
		const std::vector<double>& map_x, const std::vector<double>& map_y, const std::vector<double>& map_s,
		const std::vector<double>& map_d_x, const std::vector<double>& map_d_y)
	: map_x(map_x), map_y(map_y), map_s(map_s), map_d_x(map_d_x), map_d_y(map_d_y),
	  max_velocity(getMeterPerSecond(MAX_VELOCITY)),
	  planning_t(5),
	  time_step(0.5),
	  scales(5),
	  num_samples(9),
	  sample_sigma_s(SAMPLE_S_SIGMA), sample_sigma_d(SAMPLE_D_SIGMA),
	  pg_interval(0.02),
	  latency(0.4),
	  e(std::default_random_engine())
{}

void
PathPlanner::get_path(double car_x, double car_y, double theta, double car_s, double car_d, double car_speed,
	const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
	std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals) {

	int target_lane = 1;
	double target_d = get_lane_center(target_lane);
	double ref_speed = getMeterPerSecond(MAX_VELOCITY);

	vector<double> spline_x;
	vector<double> spline_y;
	double last_x;
	double last_y;
	double last_heading;

	if (path_x_vals.size() >= 2) {
		double x_n1 = path_x_vals[path_x_vals.size() - 1];
		double x_n2 = path_x_vals[path_x_vals.size() - 2];
		double y_n1 = path_y_vals[path_y_vals.size() - 1];
		double y_n2 = path_y_vals[path_y_vals.size() - 2];

		double heading = atan2(y_n1 - y_n2, x_n1 - x_n2);

		spline_x.push_back(x_n2);
		spline_x.push_back(x_n1);

		spline_y.push_back(y_n2);
		spline_y.push_back(y_n1);

		last_x = x_n1;
		last_y = y_n1;
		last_heading = heading;
	} else {
		double prev_x = car_x - cos(theta);
		double prev_y = car_y - sin(theta);
		spline_x.push_back(prev_x);
		spline_y.push_back(prev_y);

		spline_x.push_back(car_x);
		spline_y.push_back(car_y);

		last_x = car_x;
		last_y = car_y;
		last_heading = theta;
	}

	auto wp_30 = getXY(car_s + 30, target_d, map_s, map_x, map_y);
	auto wp_60 = getXY(car_s + 60, target_d, map_s, map_x, map_y);
	auto wp_90 = getXY(car_s + 90, target_d, map_s, map_x, map_y);

	spline_x.push_back(wp_30[0]);
	spline_x.push_back(wp_60[0]);
	spline_x.push_back(wp_90[0]);

	spline_y.push_back(wp_30[1]);
	spline_y.push_back(wp_60[1]);
	spline_y.push_back(wp_90[1]);

	for (auto i = 0; i < spline_x.size(); ++i) {
		double x = spline_x[i] - last_x;
		double y = spline_y[i] - last_y;
		spline_x[i] = x * cos(-last_heading) - y * sin(-last_heading);
		spline_y[i] = x * sin(-last_heading) + y * cos(-last_heading);
	}

	copy(path_x_vals.begin(), path_x_vals.end(), back_inserter(new_path_x_vals));
	copy(path_y_vals.begin(), path_y_vals.end(), back_inserter(new_path_y_vals));

	tk::spline s;
	s.set_points(spline_x, spline_y);

	double horizon_x = 30;
	double horizon_y = s(horizon_x);
	double dist = sqrt(pow(horizon_x, 2) + pow(horizon_y, 2));
	double n = dist / (pg_interval * ref_speed);

	for (auto i = 1; i <= 50 - path_x_vals.size(); ++i) {
		double sample_x = i * (horizon_x / n);
		double sample_y = s(sample_x);

		double global_x = sample_x * cos(last_heading) - sample_y * sin(last_heading);
		double global_y = sample_x * sin(last_heading) + sample_y * cos(last_heading);
		global_x += last_x;
		global_y += last_y;

		new_path_x_vals.push_back(global_x);
		new_path_y_vals.push_back(global_y);
	}



}

// calculating jerk minimizing trajectory
Polynomial PathPlanner::jmt(const std::vector<double>& start, const std::vector<double>& end, double t) {
	vector<double> coeffs(6, 0.0);
	coeffs[0] = start[0];
	coeffs[1] = start[1];
	coeffs[2] = 0.5 * start[2];

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

	return Polynomial(coeffs);
}

void PathPlanner::generate_xy_trajectory(const Trajectory& trajectory,
	std::vector<double>& out_x, std::vector<double>& out_y, std::size_t maximum_points) {
	double t = trajectory.t;
	double ts = 0.0 + pg_interval;
	size_t i = out_x.size();
	while (ts < t && i < maximum_points) {
		double s = trajectory.s_poly(ts);
		double d = trajectory.d_poly(ts);
		const std::vector<double>& xy = getXY(s, d, map_s, map_x, map_y);
		out_x.push_back(xy[0]);
		out_y.push_back(xy[1]);
		ts += pg_interval;
		++i;
	}
}

