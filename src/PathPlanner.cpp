//
// Created by x on 17-9-10.
//
#include "consts.h"
#include "helpers.h"
#include "PathPlanner.h"

#include "Eigen-3.3/Eigen/Dense"

#include <iostream>
#include <algorithm>
#include <numeric>

#include "cost_functions.h"

using namespace std;
using namespace AlgebraX;

namespace {

	std::vector<WeightedCostFunction> ALL_COST_FUNCTIONS = {
		{ "S", s_diff_cost, 1.0 },
		{ "D", d_diff_cost, 1.0 },
		{ "MAX_V", max_velocity_cost, 100.0 },
		{ "MAX_JERK", max_jerk_cost, 100.0 }
	};

	class CostCalculator {
	private:
		const std::vector<double>& goal_s;
		const std::vector<double>& goal_t;
	public:
		inline CostCalculator(const std::vector<double>& goal_s, const std::vector<double>& goal_t)
			: goal_s(goal_s), goal_t(goal_t) {}

		inline double operator() (const Trajectory& trajectory, bool verbose = false) const {
			double total = 0.0;
			auto iter = ALL_COST_FUNCTIONS.begin();
			while (iter != ALL_COST_FUNCTIONS.end()) {
				double cost = iter->weight * iter->f(trajectory, goal_s, goal_t);
				if (verbose) {
					std::cout<<"Cost function "<< iter->name << ": " << cost << endl;
				}
				total += cost;
				++iter;
			}
			return total;
		}
	};

	inline int get_lane_no(double d) {
		return static_cast<int>(std::floor(d / LANE_WIDTH));
	}

	inline double get_lane_center(int lane) {
		return lane * LANE_WIDTH + LANE_WIDTH / 2.0;
	}

}

PathPlanner::PathPlanner(const std::vector<double>& map_x, const std::vector<double>& map_y, const std::vector<double>& map_s)
	: map_x(map_x), map_y(map_y), map_s(map_s),
	max_velocity(getMeterPerSecond(MAX_VELOCITY)),
	planning_t(5),
	time_step(0.5),
	scales(5),
	num_samples(9),
	sample_sigma_s(SAMPLE_S_SIGMA),
	sample_sigma_d(SAMPLE_D_SIGMA),
	pg_interval(0.02),
	e(std::default_random_engine())
{}

void
PathPlanner::get_path(double car_x, double car_y, double theta, double car_s, double car_d, double car_speed,
	const std::vector<double> &path_x_vals, const std::vector<double> &path_y_vals,
	std::vector<double> &new_path_x_vals, std::vector<double> &new_path_y_vals) {

	// predict the target goal after planning_t seconds
	vector<double> start_s;
	vector<double> start_d;
	double v_s;
	double a_s;
	double v_d;
	double a_d;
	if (path_x_vals.size() > 3) {
		// theta passing in getFrenet is used for a reference for getting next waypoint,
		// so the exact value of theta will not matter as long as it is pointing into the right direction.
		vector<double> path0_sd = getFrenet(path_x_vals[0], path_y_vals[1], theta, map_x, map_y);
		vector<double> path1_sd = getFrenet(path_x_vals[1], path_y_vals[1], theta, map_x, map_y);

		double ds = path1_sd[0] - path0_sd[0];
		double dd = path1_sd[1] - path0_sd[1];
		double theta_in_frenet = std::atan2(ds, dd);
		v_s = car_speed * std::sin(theta_in_frenet);
		v_d = car_speed * std::cos(theta_in_frenet);

		// ds = v_s * t + 0.5 * a_s * t ** 2 => a_s = (ds - v_s * t) * 2 / (t ** 2)
		// dd = v_d * t + 0.5 * a_d * t ** 2 => a_d = (dd - v_d * t) * 2 / (t ** 2)

		a_s = (ds - v_s * pg_interval) * 2 / std::pow(pg_interval, 2);
		a_d = (dd - v_d * pg_interval) * 2 / std::pow(pg_interval, 2);

		start_s = { car_s, v_s, a_s };
		start_d = { car_d, v_d, a_d };
	}
	else {
		// we are at the very beginning. how about giving the vehicle an 0.1m/s^2 s acceleration and 0m/s^2 d acceleration?
		start_s = { car_s, 0.0, 0.0 };
		start_d = { car_d, 0.0, 0.0 };
		v_s = 0;
		a_s = 1;
		v_d = 0;
		a_d = 0;
	}

	cout << "start s: "<< start_s[0] << " "<< start_s[1] << " "<< start_s[2] <<endl;
	cout << "start d: "<< start_d[0] << " "<< start_d[1] << " "<< start_d[2] <<endl;

	// suppose the acceleration is the same in goal state
	// so: v_t = v_0 + a * t, s_t = v_0 * t + 1/2 * a * t^2 (the same for the d component)
	vector<Trajectory> trajectories;
	vector<double> costs;

	double a = 1;
	for (int i = -scales; i <= scales; ++i) {
		double t = planning_t + i * time_step;
		double v_s_end = v_s + a * t;
		double s_end = car_s + v_s * t + 0.5 * a * t * t;
		if (v_s_end > max_velocity) {
			v_s_end = max_velocity;
			a_s = 0;
		} else {
			a_s = a;
		}

		double v_d_end = 0;
		double d_end = car_d + v_d * t + 0.5 * a_d * t * t;

		d_end = get_lane_center(get_lane_no(car_d));

		vector<double> goal_s = { s_end, v_s_end, a_s };
		vector<double> goal_d = { d_end, 0, 0 };

		//cout << "goal s: "<< goal_s[0] << " "<< goal_s[1] << " "<< goal_s[2] <<endl;
		//cout << "goal d: "<< goal_d[0] << " "<< goal_d[1] << " "<< goal_d[2] <<endl;

		Polynomial coeffs_s = jmt(start_s, goal_s, t);
		Polynomial coeffs_d = jmt(start_d, goal_d, t);

		CostCalculator cc(goal_s, goal_d);

		Trajectory traj = { coeffs_s, coeffs_d, t };
		//cout<<traj.s_poly<<endl<<traj.d_poly<<endl<<t<<endl<<endl;
		trajectories.push_back(traj);
		double cost = cc(traj);
		costs.push_back(cost);
		//cout<<"-->"<<traj.d_poly(t)<<endl;
		//cout<<cost<<endl<<endl;

		std::normal_distribution<double> distribution_s(s_end, sample_sigma_s);
		std::normal_distribution<double> distribution_d(d_end, sample_sigma_d);
		num_samples = 0;
		for (int j = 0; j < num_samples; ++j) {
			vector<double> g_s = { distribution_s(e), v_s_end, a_s};
			vector<double> g_d = { distribution_d(e), v_d_end, a_d};

			Polynomial coeffs_s = jmt(start_s, g_s, t);
			Polynomial coeffs_d = jmt(start_d, g_d, t);

			Trajectory traj = { coeffs_s, coeffs_d, t };
			trajectories.push_back(traj);
			double cost = cc(traj);
			costs.push_back(cost);
		}
	}

	auto min_iter = min_element(costs.begin(), costs.end());
	int min_idx = static_cast<int>(distance(costs.begin(), min_iter));

	const Trajectory& min_traj = trajectories[min_idx];
	cout << "s: " << min_traj.s_poly << endl;
	cout << "s': " << d(min_traj.s_poly) << endl;
	cout << "s'': " << d<2>(min_traj.s_poly) << endl;
	cout << "d: " << min_traj.d_poly << endl;
	cout << "d': " << d(min_traj.d_poly) << endl;
	cout << "d': " << d<2>(min_traj.d_poly) << endl;
	cout << "t: " << min_traj.t << endl;
	cout << "cost: " << (*min_iter) << endl;
	cout << "goal s: " << min_traj.s_poly(min_traj.t) << " " << d(min_traj.s_poly)(min_traj.t) << " " << d<2>(min_traj.s_poly)(min_traj.t) << endl;
	cout << "goal d: " << min_traj.d_poly(min_traj.t) << " " << d(min_traj.d_poly)(min_traj.t) << " " << d<2>(min_traj.d_poly)(min_traj.t) << endl;

	// convert back to cartersian coordinate
	generate_xy_trajectory(min_traj, new_path_x_vals, new_path_y_vals);
}

// calculating jerk minimizing trajectory
Polynomial PathPlanner::jmt(const std::vector<double>& start, const std::vector<double>& end, double t) {
	vector<double> coeffs(6, 0.0);
	coeffs[0] = start[0];
	coeffs[1] = start[1];
	coeffs[2] = 0.5 * start[2];

	double t2 = t * t;
	double t3 = t2 * t;
	double t4 = t2 * t2;
	double t5 = t3 * t2;
	Eigen::MatrixXd M(3, 3);
	M <<    t3,      t4,      t5,
		3 * t2,  4 * t3,  5 * t4,
		6 * t , 12 * t2, 20 * t3;
	Eigen::VectorXd u(3);
	u << end[0] - (start[0] + start[1] * t + 0.5 * start[2] * t2),
		 end[1] - (start[1] + start[2] * t),
		 end[2] -  start[2];
	Eigen::VectorXd v = M.inverse() * u;
	coeffs[3] = v[0];
	coeffs[4] = v[1];
	coeffs[5] = v[2];

	return Polynomial(coeffs);
}

void PathPlanner::generate_xy_trajectory(const Trajectory& trajectory,
	std::vector<double>& out_x, std::vector<double>& out_y, std::size_t maximum_points) {
	double t = trajectory.t;
	double ts = 0.0 + pg_interval;
	size_t i = 0;
	while (ts < t && i < maximum_points) {
		double s = trajectory.s_poly(ts);
		double d = trajectory.d_poly(ts);
		const std::vector<double>& xy = getXY(s, d, map_s, map_x, map_y);
		out_x.push_back(xy[0]);
		out_y.push_back(xy[1]);
		ts += pg_interval;
		++i;
	}
}

