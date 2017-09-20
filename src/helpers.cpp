//
// Created by x on 17-9-10.
//

#include "helpers.h"
#include <cmath>
#include <numeric>
#include <limits>

using namespace std;

int ClosestWaypoint(double x, double y, const std::vector<double>& maps_x, const std::vector<double>& maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x, y, map_x, map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const std::vector<double>& maps_x, const std::vector<double>& maps_y) {

	int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = abs(theta - heading);

	if (angle > pi() / 4) {
		closestWaypoint++;
	}

	return closestWaypoint;

}

std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double>& maps_x, const std::vector<double>& maps_y) {
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
		prev_wp = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp];
	double n_y = maps_y[next_wp] - maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

	double frenet_d = distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x, center_y, x_x, x_y);
	double centerToRef = distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
	}

	frenet_s += distance(0, 0, proj_x, proj_y);

	return {frenet_s, frenet_d};

}

std::vector<double>
getXY(double s, double d, const std::vector<double>& maps_s, const std::vector<double>& maps_x, const std::vector<double>& maps_y) {
	int prev_wp = -1;

	while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi() / 2;

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x, y};

}

std::vector<double> getSDVelocity(double x, double y, double vx, double vy, const std::vector<double>& maps_x, const std::vector<double>& maps_y) {
	double heading = atan2(vy, vx);
	auto sd = getFrenet(x, y, heading, maps_x, maps_y);
	double nx = x + vx * 0.02;
	double ny = y + vy * 0.02;
	auto sd2 = getFrenet(nx, ny, heading, maps_x, maps_y);
	return { (sd2[0] - sd[0]) / 0.02, (sd2[1] - sd[1]) / 0.02 };
}


std::vector<int> GetNearestLeadingVehicleInLane(const vector<vector<double>>& sensor_fusion, double ego_s, int lane) {
	double min_dist_front = std::numeric_limits<double>::max();
	double min_dist_back = std::numeric_limits<double>::max();
	int min_idx_front = -1;
	int min_idx_back = -1;
	for (size_t i = 0; i < sensor_fusion.size(); ++i) {
		auto vehicle = sensor_fusion[i];
		int v_lane = get_lane_no(vehicle[SENSOR_FUSION_D]);
		if (v_lane != lane) {
			continue;
		}

		double vehicle_s = vehicle[SENSOR_FUSION_S];
		if (vehicle_s < ego_s) {
			double diff = ego_s - vehicle_s;
			if (diff < min_dist_back) {
				min_dist_back = diff;
				min_idx_back = static_cast<int>(i);
			}
		} else {
			double diff = vehicle_s - ego_s;
			if (diff < min_dist_front) {
				min_dist_front = diff;
				min_idx_front = static_cast<int>(i);
			}
		}
	}
	return {min_idx_front, min_idx_back};
}
