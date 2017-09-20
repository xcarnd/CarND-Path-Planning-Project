//
// Created by x on 17-9-10.
//

#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include <cmath>
#include <utility>
#include <vector>
#include <ostream>
#include "consts.h"

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }

inline double rad2deg(double x) { return x * 180 / pi(); }

inline double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const std::vector<double>& maps_x, const std::vector<double>& maps_y);

int NextWaypoint(double x, double y, double theta, const std::vector<double>& maps_x, const std::vector<double>& maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double>& maps_x, const std::vector<double>& maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double>& maps_s, const std::vector<double>& maps_x, const std::vector<double>& maps_y);

std::vector<double> getSDVelocity(double x, double y, double vx, double vy, const std::vector<double>& maps_x, const std::vector<double>& maps_y);

inline double getMeterPerSecond(double mph) {
	return 0.44704 * mph;
}

inline double getMilePerHour(double mps) {
	return 2.236936 * mps;
}

inline double logistic(double v) {
	return 2.0 / (1 + std::exp(-v)) - 1.0;
}

inline int get_lane_no(double d) {
	return static_cast<int>(std::floor(d / LANE_WIDTH));
}

inline double get_lane_center(int lane) {
	return lane * LANE_WIDTH + LANE_WIDTH / 2.0;
}

std::vector<int> GetNearestLeadingVehicleInLane(const std::vector<std::vector<double>>& sensor_fusion, double ego_s, int lane);

#endif //PATH_PLANNING_HELPERS_H
