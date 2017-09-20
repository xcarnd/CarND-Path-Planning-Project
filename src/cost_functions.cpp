#include "consts.h"
#include "helpers.h"
#include "cost_functions.h"

using namespace std;

/**
 * Penalizes low speeds
 */
double velocity_cost(double start_s, double start_speed, int start_lane,
                     double end_s, double end_speed, int end_lane,
                     const std::vector<std::vector<double>> &sensor_fusion) {
	return logistic(abs(end_speed - getMeterPerSecond(MAX_VELOCITY)));
}

/**
 * Penalizes lane change
 */
double lane_change_cost(double start_s, double start_speed, int start_lane,
                        double end_s, double end_speed, int end_lane,
                        const std::vector<std::vector<double>> &sensor_fusion) {
	return start_lane != end_lane ? 1.0 : 0.0;
}

/**
 * Penalizes unsafe distance (or even collision)
 */
double safe_distance_cost(double start_s, double start_speed, int start_lane,
                          double end_s, double end_speed, int end_lane,
                          const std::vector<std::vector<double>> &sensor_fusion) {
	return 0.0;
}
