#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <vector>
#include "helpers.h"

typedef double (*CostFunction)(const Trajectory& trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d);

struct WeightedCostFunction {
	std::string name;
	CostFunction f;
	double weight;
};

double s_diff_cost(const Trajectory& trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d);

double d_diff_cost(const Trajectory& trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d);

double max_velocity_cost(const Trajectory& trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d);

double max_jerk_cost(const Trajectory& trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d);

#endif // !COST_FUNCTIONS_H
