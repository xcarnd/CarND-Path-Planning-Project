#include "consts.h"
#include "helpers.h"
#include "cost_functions.h"

using namespace std;

using namespace AlgebraX;

double s_diff_cost(const Trajectory & trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d) {

	const Polynomial& poly_s = trajectory.s_poly;
	const Polynomial& poly_s_d1 = d(poly_s);
	const Polynomial& poly_s_d2 = d(poly_s_d1);

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
	const Polynomial& poly_d_d1 = d(poly_d);
	const Polynomial& poly_d_d2 = d(poly_d_d1);

	double t = trajectory.t;

	double cost = 0.0;

	vector<double> actual = { poly_d(t), poly_d_d1(t), poly_d_d2(t) };

	for (std::size_t i = 0; i < goal_d.size(); ++i) {
		double diff = std::abs(actual[i] - goal_d[i]);
		cost += logistic(diff / SAMPLE_D_SIGMA);
	}

	return cost;
}

double max_velocity_cost(const Trajectory& trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d) {
	const Polynomial& poly_s = trajectory.s_poly;
	const Polynomial& poly_d = trajectory.d_poly;
	const Polynomial& poly_s_d1 = d(poly_s);
	const Polynomial& poly_d_d1 = d(poly_d);
	const double v_s_end = poly_s_d1(trajectory.t);
	const double v_d_end = poly_d_d1(trajectory.t);
	return std::sqrt(std::pow(v_s_end, 2) + std::pow(v_d_end, 2)) > getMeterPerSecond(MAX_VELOCITY) ? 1 : 0;
}

double max_jerk_cost(const Trajectory& trajectory, const std::vector<double>& goal_s, const std::vector<double>& goal_d) {
	const Polynomial& poly_jerk_s = d<2>(trajectory.s_poly);
	double ts = trajectory.t / 100;
	double t = 0;
	for (int i = 0; i < 100; ++i) {
		double jerk = abs(poly_jerk_s(t));
		if (jerk > MAX_JERK) {
			return 1;
		}
		t += ts;
	}
	return 0;
}