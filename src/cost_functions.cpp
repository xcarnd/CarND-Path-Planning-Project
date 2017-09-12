#include "consts.h"
#include "helpers.h"
#include "cost_functions.h"
#include <cmath>

using namespace std;

double s_diff_cost(const Trajectory & trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d) {

	const Polynomial& poly_s = trajectory.s_poly;
	const Polynomial& poly_s_d1 = poly_s.differentiate();
	const Polynomial& poly_s_d2 = poly_s.differentiate();

	double t = trajectory.t;

	double cost = 0.0;

	vector<double> actual = { poly_s(t), poly_s_d1(t), poly_s_d2(t) };
	
	for (std::size_t i = 0; i < goal_s.size(); ++i) {
		double diff = std::abs(actual[i] - goal_s[i]);
		cost += logistic(diff / SAMPLE_S_SIGMA);
	}

	return cost;
}

double d_diff_cost(const Trajectory & trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d) {

	const Polynomial& poly_d = trajectory.d_poly;
	const Polynomial& poly_d_d1 = poly_d.differentiate();
	const Polynomial& poly_d_d2 = poly_d.differentiate();

	double t = trajectory.t;

	double cost = 0.0;

	vector<double> actual = { poly_d(t), poly_d_d1(t), poly_d_d2(t) };

	for (std::size_t i = 0; i < goal_d.size(); ++i) {
		double diff = std::abs(actual[i] - goal_d[i]);
		cost += logistic(diff / SAMPLE_D_SIGMA);
	}

	return cost;
}
