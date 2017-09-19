#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <vector>
#include "helpers.h"
#include "PathPlanner.h"

typedef double (*CostFunction)(double target_speed, int lane_delta,
                               const std::vector<std::vector<double>>& sensor_fusion,
                               BehaviorState state);

struct WeightedCostFunction {
	CostFunction cost_function;
	double weight;
};

double velocity_cost(double target_speed, int lane_delta,
                     const std::vector<std::vector<double>>& sensor_fusion,
                     BehaviorState state);

double lane_change_cost(double target_speed, int lane_delta,
                        const std::vector<std::vector<double>>& sensor_fusion,
                        BehaviorState state);

double collision_cost(double target_speed, int lane_delta,
                      const std::vector<std::vector<double>>& sensor_fusion,
                      BehaviorState state);

#endif // !COST_FUNCTIONS_H
