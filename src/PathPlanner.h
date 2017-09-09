//
// Created by x on 17-9-10.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H


#include <vector>

class PathPlanner {

public:
	void get_path(double car_x, double car_y, double theta, std::vector<double> &next_x_vals,
		              std::vector<double> &next_y_vals);
};


#endif //PATH_PLANNING_PATHPLANNER_H
