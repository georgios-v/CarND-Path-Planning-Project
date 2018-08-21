/* 
 * File:   Trajectory.h
 * Author: Georgios Varisteas <georgios.varisteas@uni.lu>
 *
 * Created on August 20, 2018, 5:56 PM
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>

#include "State.h"

class Trajectory {
public:
	Trajectory(State& S);
	Trajectory(const Trajectory& orig);
	virtual ~Trajectory();
	
	void generate(Map& map);
	std::vector<std::vector<double>> getSpline();
	
private:
	std::vector<double> ptsx_;
	std::vector<double> ptsy_;
	
	State *s_ = nullptr;
};

#endif /* TRAJECTORY_H */

