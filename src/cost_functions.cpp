#include "consts.h"
#include "helpers.h"
#include "cost_functions.h"

using namespace std;

/**
 * Mainly used when breaking tie in lane change. Awards behavior which will get
 * more buffer distance in the target lane.
 */
double buffer_cost(double start_s, double start_speed, int start_lane,
                   int end_lane,
                   const std::vector<std::vector<double>> &sensor_fusion) {
	auto nearest = GetNearestLeadingVehicleInLane(sensor_fusion, start_s, end_lane);
	int leading = nearest[0];
	if (leading == -1) {
		return -1;
	}
	const vector<double> &vehicle_state = sensor_fusion[leading];
	double s_target = vehicle_state[SENSOR_FUSION_S];
	// the further the ego vehicle is away from the leading vehicle, the more awards it get.
	double diff = s_target - start_s;
	if (diff < SAFE_DIST) {
		return 0.0;
	}
	return -logistic(diff / SAFE_DIST);
}

/**
 * Penalizes low allowed speed.
 *
 * If no collision risk, the allowed speed will be the maximum allowed speed,
 * otherwise the same speed as the leading vehicle in that lane.
 */
double max_allowed_velocity_cost(double start_s, double start_speed, int start_lane, int end_lane,
                                 const std::vector<std::vector<double>> &sensor_fusion) {
	double max_v = getMeterPerSecond(MAX_VELOCITY);
	double max_allowed = max_v;
	int idx = GetNearestLeadingVehicleInLane(sensor_fusion, start_s, end_lane)[0];
	if (idx != -1) {
		const vector<double> &vehicle_state = sensor_fusion[idx];
		double s_target = vehicle_state[SENSOR_FUSION_S];
		double vx_target = vehicle_state[SENSOR_FUSION_VX];
		double vy_target = vehicle_state[SENSOR_FUSION_VY];
		auto v_s_target = sqrt(pow(vx_target, 2) + pow(vy_target, 2));
		double delta_s = s_target - start_s;
		if (delta_s < SAFE_DIST) {
			max_allowed = v_s_target;
		} else {
			double safe_dist = delta_s - SAFE_DIST;
			double future_t = 5.0;
			double delta_v = safe_dist / future_t;
			max_allowed = v_s_target + delta_v;
			if (max_allowed > max_v) {
				max_allowed = max_v;
			}
		}
	}
	// the closer the allowed speed with the maximum allowed speed, the lower the cost.
	return (1 - (max_allowed / max_v));
}

/**
 * Penalizes lane change
 */
double lane_change_cost(double start_s, double start_speed, int start_lane, int end_lane,
                        const std::vector<std::vector<double>> &sensor_fusion) {
	return start_lane != end_lane ? 1.0 : 0.0;
}

/**
 * Penalizes unsafe distance (or even collision)
 */
double safe_distance_cost(double start_s, double start_speed, int start_lane, int end_lane,
                          const std::vector<std::vector<double>> &sensor_fusion) {
	if (start_lane == end_lane) {
		// keeping lane is always try to avoid collision. (if not, that's the bug of keeping lane)
		return 0.0;
	}
	// have to check vehicles leading and following the ego vehicle. If either is too close,
	// it is considered as not safe.
	vector<int> nearest = GetNearestLeadingVehicleInLane(sensor_fusion, start_s, end_lane);

	// the strategy of checking:
	// compute the distance between the target and the ego vehicle (noted as ds) and the difference of
	// velocity (noted as dv).
	// 1. if ds is too small, considered as not safe.
	// 2. If (ds/dv) < 5.0, that means in less than 5s, the location of the
	// the vehicles will coincide, which means collision. That's considered as not safe.
	int idx_back = nearest[1];
	if (idx_back > -1) {
		const vector<double>& vehicle_state = sensor_fusion[idx_back];
		double s_target = vehicle_state[SENSOR_FUSION_S];
		double vx_target = vehicle_state[SENSOR_FUSION_VX];
		double vy_target = vehicle_state[SENSOR_FUSION_VY];
		double v_s_target = sqrt(pow(vx_target, 2) + pow(vy_target, 2));
		double diff_v = abs(start_speed - v_s_target);
		double diff_s = start_s - s_target;
		if (diff_s < DIST_TOO_CLOSE) {
			return 1;
		}
		if ((v_s_target > start_speed) && ((diff_s - DIST_TOO_CLOSE) / diff_v) < 5.0)  {
			return 1;
		}
	}
	int idx_front = nearest[0];
	if (idx_front > -1) {
		const vector<double>& vehicle_state = sensor_fusion[idx_front];
		double s_target = vehicle_state[SENSOR_FUSION_S];
		double vx_target = vehicle_state[SENSOR_FUSION_VX];
		double vy_target = vehicle_state[SENSOR_FUSION_VY];
		double v_s_target = sqrt(pow(vx_target, 2) + pow(vy_target, 2));
		double diff_v = abs(start_speed - v_s_target);
		double diff_s = s_target - start_s;
		if (diff_s < DIST_TOO_CLOSE) {
			return 1;
		}
		if ((start_speed > v_s_target) && ((diff_s - DIST_TOO_CLOSE) / diff_v) < 5.0) {
			return 1.0;
		}
	}
	return 0.0;
}
