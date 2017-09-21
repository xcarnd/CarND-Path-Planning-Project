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

namespace {

	/**
	 * Weighted cost functions
	 */
	std::vector<WeightedCostFunction> ALL_COST_FUNCTIONS = {
		{ max_allowed_velocity_cost,  20.0   },
		{ lane_change_cost,           5.0    },
//		{ high_traffic_cost,          2.0    },
		{ safe_distance_cost,         1000.0 }
	};

	double get_total_cost(double start_s, double start_speed, int start_lane, int end_lane,
	                      const std::vector<std::vector<double>> &sensor_fusion) {
		double total = 0.0;
		for (WeightedCostFunction wcf : ALL_COST_FUNCTIONS) {
			total += wcf.weight * wcf.cost_function(start_s, start_speed, start_lane,
			                                        end_lane,
			                                        sensor_fusion);
		}
		return total;
	}

	std::vector<BehaviorState> NextPossibleStates(int currentLane, BehaviorState currentState) {
		std::vector<BehaviorState> result;
		result.push_back(KL);
		if (currentLane < 2) {
			result.push_back(LCR);
		}
		if (currentLane > 0) {
			result.push_back(LCL);
		}
		return result;
	}

}

PathPlanner::PathPlanner(
		const std::vector<double>& map_x, const std::vector<double>& map_y, const std::vector<double>& map_s,
		const std::vector<double>& map_d_x, const std::vector<double>& map_d_y)
	: map_x(map_x), map_y(map_y), map_s(map_s), map_d_x(map_d_x), map_d_y(map_d_y),
	  max_velocity(getMeterPerSecond(MAX_VELOCITY)),
	  pg_interval(0.02),
	  ref_v(0.0),
	  accel(3),
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
	double ref_s;
	double ref_x;
	double ref_y;
	double ref_heading;
	int current_lane = get_lane_no(car_d);

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

		ref_s = end_s;
		ref_x = x_n1;
		ref_y = y_n1;
		ref_heading = heading;
	} else {
		double prev_x = car_x - cos(theta);
		double prev_y = car_y - sin(theta);
		spline_x.push_back(prev_x);
		spline_y.push_back(prev_y);

		spline_x.push_back(car_x);
		spline_y.push_back(car_y);

		ref_s = car_s;
		ref_x = car_x;
		ref_y = car_y;
		ref_heading = theta;
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
			// although velocity of s in frenet framework is not the same as sqrt(vx^2 + vy^2) in cartesian framework,
			// such approximation work well.
			double future_t = 5;
			double new_s = (car_s + car_speed * future_t);

			// Keep Lane shall check against the vehicle in front of the ego vehicle
			// and see if there's any possible collision. If yes, reference speed shall
			// be reduced.
			double new_ref_speed = ref_v;
			double dist_to_nearest = numeric_limits<double>::max();
			bool no_collision_risk = true;
			int idx = GetNearestLeadingVehicleInLane(sensor_fusion, car_s, current_lane)[0];
			if (idx > -1) {
				const vector<double>& vehicle_state = sensor_fusion[idx];
				double s_target = vehicle_state[SENSOR_FUSION_S];
				double d_target = vehicle_state[SENSOR_FUSION_D];
				double id_target = vehicle_state[SENSOR_FUSION_ID];
				double x_target = vehicle_state[SENSOR_FUSION_X];
				double y_target = vehicle_state[SENSOR_FUSION_Y];
				double vx_target = vehicle_state[SENSOR_FUSION_VX];
				double vy_target = vehicle_state[SENSOR_FUSION_VY];
				auto v_s_target = sqrt(pow(vx_target, 2) + pow(vy_target, 2));

				// so we're behind the target vehicle. check the distance between that target and us. If too close
				// (or even we'll be in front of it), collision happens.
				// say, 30m in s?
				double new_target_s = s_target + v_s_target * future_t;
				dist_to_nearest = new_target_s - new_s;
				if (dist_to_nearest < SAFE_DIST) {
//					cout<<"Potential collision caution. Will getting too close with vehicle "<<id_target<<". (Distance at "<<future_t<<" second: "<<dist_at_future<<")"<<endl;
//					cout<<"Target status: "
//					    <<x_target<<", "<<y_target<<", "
//					    <<vx_target<<", "<<vy_target<<", "
//					    <<s_target<<", "<<d_target<<endl;
					// collision. We shall reduce the reference speed to be the same as the target vehicle.
					no_collision_risk = false;
					new_ref_speed = sqrt(pow(vx_target, 2) + pow(vy_target, 2));
				}
			}
			if (no_collision_risk) {
				double speed = ref_v + accel * pg_interval;
				if (speed > max_velocity) {
					speed = max_velocity;
				}
				nextStateAndCost.emplace_back(vector<double>(
						{get_total_cost(car_s, car_speed, current_lane, current_lane, sensor_fusion), (double)KL, speed, (double)current_lane}
				));
			} else {
				double speed = ref_v;
				// reduce to target speed, or if reducing to target speed is not enough (still getting to close), continue
				// reducing even more.

				if (dist_to_nearest < SAFE_DIST / 2) {
					speed = speed - 2 * accel * pg_interval;
				}

				nextStateAndCost.emplace_back(vector<double>(
						{get_total_cost(car_s, car_speed, current_lane, current_lane, sensor_fusion), (double)KL, speed, (double)current_lane}));
			}
		} else if (bs == LCL || bs == LCR){
			// LCL / LCR are quite the same except for the new lane
			int new_lane;
			if (bs == LCL) {
				new_lane = current_lane - 1;
			} else {
				new_lane = current_lane + 1;
			}

			// lane change is supposed to be able to driving at higher speed
			// (otherwise it is of no good to perform lane shifting.
			// the highest possible speed depends on the traffic of the target lane:
			// if in the target lane, no vehicles is within, say, 30m of the ego vehicle

			double speed = ref_v + accel * pg_interval;
			if (speed > max_velocity) {
				speed = max_velocity;
			}
			double future_t = 5;
			double new_s = (car_s + car_speed * future_t);

			nextStateAndCost.emplace_back(vector<double>(
					{get_total_cost(car_s, car_speed, current_lane, new_lane, sensor_fusion), (double)bs, speed, (double)new_lane}));
//			// if we're performing lane change with some v_d, we will then know how long it will take to
			// perform the lane shifting.
			// improvement: may be we can use jmt to pick one optimal lane change trajectory?
			// fix it to be 1m/s

		}
	}

//	double target_d = get_lane_center(current_lane);
//	double ref_speed = ref_v + accel * pg_interval;
//	if (ref_speed > max_velocity) {
//		ref_speed = max_velocity;
//	}
	double min_cost = numeric_limits<double>::max();
	size_t min_idx = 0;
	BehaviorState min_state = KL;
	for (size_t i = 0; i < nextStateAndCost.size(); ++i) {
		auto nsac = nextStateAndCost[i];
		auto state = static_cast<BehaviorState>(static_cast<int>(nsac[1]));
		double cost = nsac[0];
		if (cost < min_cost) {
			min_idx = i;
			min_cost = cost;
			min_state = state;
		}
		cout<<state<<": "<<cost<<", ";
	}
	cout<<"Next behavior: "<<min_state<<", cost: "<<min_cost<<endl;
	auto target_lane = static_cast<int>(nextStateAndCost[min_idx][3]);
	double ref_speed = nextStateAndCost[min_idx][2];

	double target_d = get_lane_center(target_lane);

	auto wp_30 = getXY(ref_s + 30, target_d, map_s, map_x, map_y);
	auto wp_60 = getXY(ref_s + 60, target_d, map_s, map_x, map_y);
	auto wp_90 = getXY(ref_s + 90, target_d, map_s, map_x, map_y);

	spline_x.push_back(wp_30[0]);
	spline_x.push_back(wp_60[0]);
	spline_x.push_back(wp_90[0]);

	spline_y.push_back(wp_30[1]);
	spline_y.push_back(wp_60[1]);
	spline_y.push_back(wp_90[1]);

	for (auto i = 0; i < spline_x.size(); ++i) {
		double x = spline_x[i] - ref_x;
		double y = spline_y[i] - ref_y;
		spline_x[i] = x * cos(-ref_heading) - y * sin(-ref_heading);
		spline_y[i] = x * sin(-ref_heading) + y * cos(-ref_heading);
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

		double global_x = sample_x * cos(ref_heading) - sample_y * sin(ref_heading);
		double global_y = sample_x * sin(ref_heading) + sample_y * cos(ref_heading);
		global_x += ref_x;
		global_y += ref_y;

		new_path_x_vals.push_back(global_x);
		new_path_y_vals.push_back(global_y);
	}

	this->ref_v = ref_speed;
	current_state = static_cast<BehaviorState>(static_cast<int>(nextStateAndCost[min_idx][1]));
}


