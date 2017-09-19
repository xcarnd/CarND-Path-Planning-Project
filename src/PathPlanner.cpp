//
// Created by x on 17-9-10.
//
#include "consts.h"
#include "helpers.h"
#include "PathPlanner.h"

#include "Eigen-3.3/Eigen/Dense"

#include <iostream>

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

	constexpr size_t SENSOR_FUSION_ID = 0;
	constexpr size_t SENSOR_FUSION_X  = 1;
	constexpr size_t SENSOR_FUSION_Y  = 2;
	constexpr size_t SENSOR_FUSION_VX = 3;
	constexpr size_t SENSOR_FUSION_VY = 4;
	constexpr size_t SENSOR_FUSION_S  = 5;
	constexpr size_t SENSOR_FUSION_D  = 6;

	inline int get_lane_no(double d) {
		return static_cast<int>(std::floor(d / LANE_WIDTH));
	}

	inline double get_lane_center(int lane) {
		return lane * LANE_WIDTH + LANE_WIDTH / 2.0;
	}

	std::vector<BehaviorState> NextPossibleStates(int currentLane, BehaviorState currentState) {
		switch (currentState) {
			case KL: {
				std::vector<BehaviorState> result;
				result.push_back(KL);
				if (currentLane < 3) {
					result.push_back(LCR);
				}
				if (currentLane > 1) {
					result.push_back(LCL);
				}
				break;
			}
			case LCL: {
				break;
			}
			case LCR: {
				break;
			}
		}
	}

}

PathPlanner::PathPlanner(
		const std::vector<double>& map_x, const std::vector<double>& map_y, const std::vector<double>& map_s,
		const std::vector<double>& map_d_x, const std::vector<double>& map_d_y)
	: map_x(map_x), map_y(map_y), map_s(map_s), map_d_x(map_d_x), map_d_y(map_d_y),
	  max_velocity(getMeterPerSecond(MAX_VELOCITY)),
	  pg_interval(0.02),
	  ref_v(0.0),
	  a_keep_lane(3),
	  current_lane(1),
	  current_state(KL)
{}

void
PathPlanner::get_path(double car_x, double car_y, double theta,
                      double car_s, double car_d, double end_s, double end_d,
                      double car_speed,
                      const std::vector<std::vector<double>>& sensor_fusion,
                      const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
                      std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals) {

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

		current_lane = get_lane_no(car_d);
	}

	auto next_states = NextPossibleStates(current_lane, current_state);
	// nextStateAndCost is an vector of vector with the following format:
	// [
	//   [cost1, nextState1, nextReferenceSpeed1, nextTargetLane1, BehaviorSpecificData1...],
	//   [cost2, nextState2, nextReferenceSpeed2, nextTargetLane2, BehaviorSpecificData2...],
	//   ...,
	// ]
	//
	// Sensor fusion data structure:
	// [
	//   [ id1, x1, y1, vx1, vy1, s1, d1],
	//   [ id2, x2, y2, vx2, vy2, s2, d2],
	//   ...,
	// ]
	vector<vector<double>> nextStateAndCost;
	for (BehaviorState bs : next_states) {
		if (bs == KL) {
			auto v_ego = getSDVelocity(car_x, car_y, car_speed * cos(theta), car_speed * sin(theta), map_x, map_y);

			// Keep Lane shall check against the vehicle in front of the ego vehicle
			// and see if there's any possible collision. If yes, reference speed shall
			// be reduced.
			double new_ref_speed = ref_v;
			double nearest_vehicle_s = numeric_limits<double>::max();
			int nearest_target_id = 0;
			bool no_collision_risk = true;
			for (auto vehicle_state : sensor_fusion) {
				double s_target = vehicle_state[SENSOR_FUSION_S];
				double d_target = vehicle_state[SENSOR_FUSION_D];

				int at_lane = get_lane_no(d_target);
				if (at_lane != current_lane) {
					// not in the same lane as the ego vehicle. safe to ignore
					continue;
				}
				if (s_target < car_s) {
					// ignore vehicles behind the ego vehicle
					continue;
				}
				if (s_target > nearest_vehicle_s) {
					// ignore vehicles not close to the ego vehicle
					continue;
				}
				double id_target = vehicle_state[SENSOR_FUSION_ID];
				double x_target = vehicle_state[SENSOR_FUSION_X];
				double y_target = vehicle_state[SENSOR_FUSION_Y];
				double vx_target = vehicle_state[SENSOR_FUSION_VX];
				double vy_target = vehicle_state[SENSOR_FUSION_VY];
				auto v_target_frenet = getSDVelocity(x_target, y_target, vx_target, vy_target, map_x, map_y);

				// so we're behind the target vehicle. check the distance between that target and us. If too close
				// (or even we'll be in front of it), collision happens.
				// say, 30m in s?
				double check_dist = 30;
				double too_close = 15;

				double t = check_dist / v_ego[0];
				double new_target_s = s_target + v_target_frenet[0] * t;
				if (new_target_s - (car_s + check_dist) < too_close) {
					// collision. We shall reduce the reference speed to be the same as the target vehicle.
					no_collision_risk = false;
					if (s_target < nearest_vehicle_s) {
						nearest_vehicle_s = s_target;
						nearest_target_id = id_target;
						new_ref_speed = sqrt(pow(vx_target, 2) + pow(vy_target, 2));
					}
				}
			}
			if (no_collision_risk) {
				double speed = ref_v + a_keep_lane * pg_interval;
				if (speed > max_velocity) {
					speed = max_velocity;
				}
				nextStateAndCost.emplace_back({0, KL, speed, current_lane});
			} else {
				nextStateAndCost.emplace_back({0, KL, new_ref_speed, current_lane});
			}
		}
		// ignore other states for now.
	}

//	double target_d = get_lane_center(current_lane);
//	double ref_speed = ref_v + a_keep_lane * pg_interval;
//	if (ref_speed > max_velocity) {
//		ref_speed = max_velocity;
//	}
	double min_cost = numeric_limits<double>::max();
	size_t min_idx = 0;
	for (size_t i = 0; i < nextStateAndCost.size(); ++i) {
		auto nsac = nextStateAndCost[i];
		if (nsac[0] < min_cost) {
			min_idx = i;
			min_cost = nsac[0];
		}
	}
	int target_lane = static_cast<int>(nextStateAndCost[min_idx][3]);
	double ref_speed = nextStateAndCost[min_idx][2];

	double target_d = get_lane_center(target_lane);

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

	ref_v = ref_speed;

}


