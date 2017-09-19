#include "consts.h"
#include "helpers.h"
#include "cost_functions.h"

using namespace std;

/**
 * Penalizes low speeds
 */
double velocity_cost(double target_speed, int lane_delta,
                     const std::vector<std::vector<double>>& sensor_fusion,
                     BehaviorState state) {
	return logistic(abs(target_speed - getMeterPerSecond(MAX_VELOCITY)));
}

/**
 * Penalizes lane change
 */
double lane_change_cost(double target_speed, int lane_delta,
                        const std::vector<std::vector<double>>& sensor_fusion,
                        BehaviorState state) {
	return lane_delta == 0 ? 0 : 1;
}

/**
 * Penalizes collisions
 *
 * Note: if the passing in BehaviorState is KL, it is supposed that
 * no collisions will happen since the trajectory for KL has already
 * considered avoiding collision at its best efforts.
 */
double collision_cost(double target_speed, int lane_delta,
                      const std::vector<std::vector<double>>& sensor_fusion,
                      BehaviorState state) {
	if (state == KL) {
		return 0.0;
	}
	return 0.0;
}
