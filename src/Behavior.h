/* 
 * File:   Behavior.h
 * Author: Georgios Varisteas <georgios.varisteas@uni.lu>
 *
 * Created on August 20, 2018, 6:28 PM
 */

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <vector>
#include "State.h"

class Behavior {
public:
	static const double SPEED_LIMIT;
	static const double SPEED_STEP;
	static const int SAFE_DIST;
	static const int CHANGE_LANE_DIST;
	Behavior(State& S);
	Behavior(const Behavior& orig);
	virtual ~Behavior();

	bool check_proximity();
	void explore_change_lane();
private:
	static double mps2mph(double ms);
	static double mph2mps(double mph);

	double prev_dist_ = SAFE_DIST;
	State *s_ = nullptr;
};

#endif /* BEHAVIOR_H */
