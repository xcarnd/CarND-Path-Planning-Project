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

	constexpr size_t COST_AND_STATE_COST        = 0;
	constexpr size_t COST_AND_STATE_NEXT_STATE  = 1;
	constexpr size_t COST_AND_STATE_REF_SPEED   = 2;
	constexpr size_t COST_AND_STATE_TARGET_LANE = 3;
	//   [cost1, nextState1, nextReferenceSpeed1, nextTargetLane1, BehaviorSpecificDataIfAny1...],
	//   [cost2, nextState2, nextReferenceSpeed2, nextTargetLane2, BehaviorSpecificDataIfAny2...],


	/**
	 * Weighted cost functions
	 */
	std::vector<WeightedCostFunction> ALL_COST_FUNCTIONS = {
		{ max_allowed_velocity_cost,  25.0   },
		{ lane_change_cost,           5.0    },
		{ buffer_cost,                1.0    },
		{ safe_distance_cost,         1000.0 }
	};

	/**
	 * Get total cost for behavior
	 */
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

	/**
	 * Get the next possible states
	 */
	std::vector<BehaviorState> NextPossibleStates(int currentLane, BehaviorState currentState, bool changingLane) {
		std::vector<BehaviorState> result;
		if (changingLane) {
			result.push_back(currentState);
		} else {
			result.push_back(KL);
			if (currentLane < 2) {
				result.push_back(LCR);
			}
			if (currentLane > 0) {
				result.push_back(LCL);
			}
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
	  current_state(KL),
	  changing_lane(false),
	  target_lane(-1)
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

	// takeaway from the walk-through:
	// if there're enough points in the previously planned path, use them as the starting.
	// otherwise planning a new trajectory using the current car states.
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

	if (changing_lane) {
		if (target_lane == current_lane) {
			changing_lane = false;
		}
	}

	//
	// Sensor fusion data structure:
	// [
	//   [ id1, x1, y1, vx1, vy1, s1, d1],
	//   [ id2, x2, y2, vx2, vy2, s2, d2],
	//   ...,
	// ]
	//

	// nextStateAndCost is an vector of vector with the following format:
	// [
	//   [cost1, nextState1, nextReferenceSpeed1, nextTargetLane1, BehaviorSpecificData1...],
	//   [cost2, nextState2, nextReferenceSpeed2, nextTargetLane2, BehaviorSpecificData2...],
	//   ...,
	// ]
	auto next_states = NextPossibleStates(current_lane, current_state, changing_lane);
	vector<vector<double>> nextStateAndCost;
	// loops all the possible next behavior states and pick up the one with minimum cost
	for (BehaviorState bs : next_states) {
		if (bs == KL) {
			// although velocity of s in frenet framework is not the same as sqrt(vx^2 + vy^2) in cartesian framework,
			// such approximation works well.

			// how far shall we predict (in seconds)
			double future_t = 5;
			// and this is where the ego vehicle will be after that future seconds
			double new_s = (ref_s + ref_v * (future_t - path_x_vals.size() * pg_interval));

			// Keep Lane shall check against the vehicle in front of the ego vehicle
			// and see if there's any possible collision. If yes, reference speed will
			// be reduced.
			double new_ref_speed = ref_v;
			double dist_to_nearest = numeric_limits<double>::max();
			bool no_collision_risk = true;
			int idx = GetNearestLeadingVehicleInLane(sensor_fusion, car_s, current_lane)[0];
			if (idx > -1) {
				// sensor_funtion[idx] is the vehicle closest to ego vehicle and driving ahead of the ego vehicle
				// check if the ego vehicle will collide with it.
				const vector<double>& vehicle_state = sensor_fusion[idx];
				double s_target = vehicle_state[SENSOR_FUSION_S];
				double d_target = vehicle_state[SENSOR_FUSION_D];
				double id_target = vehicle_state[SENSOR_FUSION_ID];
				double x_target = vehicle_state[SENSOR_FUSION_X];
				double y_target = vehicle_state[SENSOR_FUSION_Y];
				double vx_target = vehicle_state[SENSOR_FUSION_VX];
				double vy_target = vehicle_state[SENSOR_FUSION_VY];
				auto v_s_target = sqrt(pow(vx_target, 2) + pow(vy_target, 2));

				// check where the target will be after that future seconds, and check if in the future the ego vehicle
				// will be too close with the target. if so, reduce the reference speed.
				double new_target_s = s_target + v_s_target * future_t;

				dist_to_nearest = new_target_s - new_s;
				if (dist_to_nearest < SAFE_DIST) {
					// potential collision risk
					no_collision_risk = false;
					// shall try to reduce to the vehicle of the target
					new_ref_speed = sqrt(pow(vx_target, 2) + pow(vy_target, 2));
				}
			}
			if (no_collision_risk) {
				// when no collision risk, we can safely accelerating to the maximum allowed speed.
				double speed = ref_v + accel * pg_interval;
				if (speed > max_velocity) {
					speed = max_velocity;
				}
				nextStateAndCost.emplace_back(vector<double>(
						{get_total_cost(car_s, car_speed, current_lane, current_lane, sensor_fusion), (double)KL, speed, (double)current_lane}
				));
			} else {
				double speed = ref_v;
				// otherwise we'll have to decelerating.
				if (dist_to_nearest < SAFE_DIST) {
					speed = speed - accel * pg_interval;
				}
				// to avoid decelerating too much, once we have reach 90% of the target speed, we can stop deceleration.
				if (speed < 0.9 * new_ref_speed) {
					speed = 0.9 * new_ref_speed;
				}

				nextStateAndCost.emplace_back(vector<double>(
						{get_total_cost(car_s, car_speed, current_lane, current_lane, sensor_fusion), (double)KL, speed, (double)current_lane}));
			}
		} else if (bs == LCL || bs == LCR){
			// LCL /uite the same except for the new lane
			int new_lane;
			if (bs == LCL) {
				new_lane = current_lane - 1;
			} else {
				new_lane = current_lane + 1;
			}

			// a simple assumption (not always be true): lane change is supposed to be able to driving at higher speed,
			// otherwise we'll prefer staying in the current lane.
//			double speed = ref_v + accel * pg_interval;
//			if (speed > max_velocity) {
//				speed = max_velocity;
//			}
			nextStateAndCost.emplace_back(vector<double>(
					{get_total_cost(car_s, car_speed, current_lane, new_lane, sensor_fusion), (double)bs, ref_v, (double)new_lane}));
		}
	}

	// Pick up the next behavior state with minimum total costs.
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

	double ref_speed = nextStateAndCost[min_idx][COST_AND_STATE_REF_SPEED];
	auto target_lane = static_cast<int>(nextStateAndCost[min_idx][COST_AND_STATE_TARGET_LANE]);

	double target_d = get_lane_center(target_lane);

	// another takeaway from the walkthrough: generating a trajectory.
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

	if ((min_state == LCL || min_state == LCR) && current_state != min_state) {
		changing_lane = true;
		this->target_lane = target_lane;
	}

	current_state = min_state;

}


