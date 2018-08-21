/* 
 * File:   map.h
 * Author: Georgios Varisteas <georgios.varisteas@uni.lu>
 *
 * Created on August 20, 2018, 2:54 PM
 */

#ifndef MAP_H
#define MAP_H

#include <vector>

class Map {
public:
	Map(const char *map_file);
	Map(const Map& orig);
	virtual ~Map();

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	std::vector<double> getFrenet(double x, double y, double theta);

	// Transform from Frenet s,d coordinates to Cartesian x,y
	std::vector<double> getXY(double s, double d);

	static double distance(double x1, double y1, double x2, double y2);
	static constexpr double pi();
	static double deg2rad(double x);
	static double rad2deg(double x);

	static constexpr int WAYPOINT_LENGTH = 50;
	static constexpr int LANES = 3;

private:
	int ClosestWaypoint(double x, double y);
	int NextWaypoint(double x, double y, double theta);

	std::vector<double> map_waypoints_x_;
	std::vector<double> map_waypoints_y_;
	std::vector<double> map_waypoints_s_;
	std::vector<double> map_waypoints_dx_;
	std::vector<double> map_waypoints_dy_;
	// The max s value before wrapping around the track back to 0
	static constexpr double max_s_ = 6945.554;
};

#endif /* MAP_H */

