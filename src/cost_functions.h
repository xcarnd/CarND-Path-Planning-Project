#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <vector>
#include "helpers.h"
#include "PathPlanner.h"

typedef double (*CostFunction)(double start_s, double start_speed, int start_lane,
                               int end_lane,
                               const std::vector<std::vector<double>> &sensor_fusion);

struct WeightedCostFunction {
	CostFunction cost_function;
	double weight;
};

double buffer_cost(double start_s, double start_speed, int start_lane,
                         int end_lane,
                         const std::vector<std::vector<double>> &sensor_fusion);

double max_allowed_velocity_cost(double start_s, double start_speed, int start_lane, int end_lane,
                                 const std::vector<std::vector<double>> &sensor_fusion);

double lane_change_cost(double start_s, double start_speed, int start_lane, int end_lane,
                        const std::vector<std::vector<double>> &sensor_fusion);

double safe_distance_cost(double start_s, double start_speed, int start_lane, int end_lane,
                          const std::vector<std::vector<double>> &sensor_fusion);

#endif // !COST_FUNCTIONS_H
