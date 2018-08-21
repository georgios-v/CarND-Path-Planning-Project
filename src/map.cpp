/* 
 * File:   map.cpp
 * Author: Georgios Varisteas <georgios.varisteas@uni.lu>
 * 
 * Created on August 20, 2018, 2:54 PM
 */

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "map.h"

Map::Map(const char *map_file) {
	std::ifstream in_map_(map_file, std::ifstream::in);

	std::string line;
	while (std::getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x_.push_back(x);
		map_waypoints_y_.push_back(y);
		map_waypoints_s_.push_back(s);
		map_waypoints_dx_.push_back(d_x);
		map_waypoints_dy_.push_back(d_y);
	}
}

Map::Map(const Map& orig) {
}

Map::~Map() {
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates

std::vector<double> Map::getFrenet(double x, double y, double theta) {
	int next_wp = NextWaypoint(x, y, theta);

	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
		prev_wp = map_waypoints_x_.size() - 1;
	}

	double n_x = map_waypoints_x_[next_wp] - map_waypoints_x_[prev_wp];
	double n_y = map_waypoints_y_[next_wp] - map_waypoints_y_[prev_wp];
	double x_x = x - map_waypoints_x_[prev_wp];
	double x_y = y - map_waypoints_y_[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = Map::distance(x_x, x_y, proj_x, proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - map_waypoints_x_[prev_wp];
	double center_y = 2000 - map_waypoints_y_[prev_wp];
	double centerToPos = Map::distance(center_x, center_y, x_x, x_y);
	double centerToRef = Map::distance(center_x, center_y, proj_x, proj_y);

	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (int i = 0; i < prev_wp; i++) {
		frenet_s += Map::distance(map_waypoints_x_[i], map_waypoints_y_[i], map_waypoints_x_[i + 1], map_waypoints_y_[i + 1]);
	}

	frenet_s += Map::distance(0, 0, proj_x, proj_y);

	return
	{
		frenet_s, frenet_d
	};
}

// Transform from Frenet s,d coordinates to Cartesian x,y

std::vector<double> Map::getXY(double s, double d) {
	int prev_wp = -1;

	while (s > map_waypoints_s_[prev_wp + 1] && (prev_wp < (int) (map_waypoints_s_.size() - 1))) {
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % map_waypoints_x_.size();

	double heading = atan2((map_waypoints_y_[wp2] - map_waypoints_y_[prev_wp]), (map_waypoints_x_[wp2] - map_waypoints_x_[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - map_waypoints_s_[prev_wp]);

	double seg_x = map_waypoints_x_[prev_wp] + seg_s * std::cos(heading);
	double seg_y = map_waypoints_y_[prev_wp] + seg_s * std::sin(heading);

	double perp_heading = heading - Map::pi() / 2;

	double x = seg_x + d * std::cos(perp_heading);
	double y = seg_y + d * std::sin(perp_heading);

	return
	{
		x, y
	};
}

int Map::ClosestWaypoint(double x, double y) {
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < map_waypoints_x_.size(); i++) {
		double map_x = map_waypoints_x_[i];
		double map_y = map_waypoints_y_[i];
		double dist = Map::distance(x, y, map_x, map_y);
		if (dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}
	return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta) {
	int closestWaypoint = ClosestWaypoint(x, y);

	double map_x = map_waypoints_x_[closestWaypoint];
	double map_y = map_waypoints_y_[closestWaypoint];

	double heading = atan2((map_y - y), (map_x - x));

	double angle = fabs(theta - heading);
	angle = std::min(2 * pi() - angle, angle);

	if (angle > pi() / 4) {
		closestWaypoint++;
		if (closestWaypoint == map_waypoints_x_.size()) {
			closestWaypoint = 0;
		}
	}
	return closestWaypoint;
}

double Map::distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));
}

// For converting back and forth between radians and degrees.

constexpr double Map::pi() {
	return M_PI;
}

double Map::deg2rad(double x) {
	return x * Map::pi() / 180;
}

double Map::rad2deg(double x) {
	return x * 180 / Map::pi();
}
