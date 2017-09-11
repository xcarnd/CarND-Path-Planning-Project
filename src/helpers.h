//
// Created by x on 17-9-10.
//

#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include <cmath>
#include <vector>

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

inline double getMeterPerSecond(double mph) {
	return 0.44704 * mph;
}

inline double getMilePerHour(double mps) {
	return 2.236936 * mps;
}

class Polynomial {
private:
	const std::vector<double>& coeffs;
public:
	inline Polynomial(const std::vector<double>& coeffs)
		: coeffs(coeffs) {}

	inline double operator()(double x) const {
		double value = 0;
		int order = coeffs.size() - 1;
		auto iter_r = coeffs.rbegin();
		while (iter_r != coeffs.rend()) {
			value += (*iter_r) * std::pow(x, order);
			--order;
			++iter_r;
		}
		return value;
	}
};


#endif //PATH_PLANNING_HELPERS_H
