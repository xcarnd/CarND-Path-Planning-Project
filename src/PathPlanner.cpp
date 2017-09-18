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
	  pg_interval(0.02),
	  ref_v(0.0),
	  a_keep_lane(3)
{}

void
PathPlanner::get_path(double car_x, double car_y, double theta,
                      double car_s, double car_d, double end_s, double end_d,
                      double car_speed,
                      const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
                      std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals) {
	int target_lane = 1;
	double target_d = get_lane_center(target_lane);

	ref_v += a_keep_lane * pg_interval;
	if (ref_v > max_velocity) {
		ref_v = max_velocity;
	}

	vector<double> spline_x;
	vector<double> spline_y;
	double last_s;
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

		last_s = end_s;
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

		last_s = car_s;
		last_x = car_x;
		last_y = car_y;
		last_heading = theta;
	}

	auto wp_30 = getXY(last_s + 30, target_d, map_s, map_x, map_y);
	auto wp_60 = getXY(last_s + 60, target_d, map_s, map_x, map_y);
	auto wp_90 = getXY(last_s + 90, target_d, map_s, map_x, map_y);

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
	double n = dist / (pg_interval * ref_v);

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


