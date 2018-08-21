/* 
 * File:   Behavior.cpp
 * Author: Georgios Varisteas <georgios.varisteas@uni.lu>
 * 
 * Created on August 20, 2018, 6:28 PM
 */
#include <cmath>

#include "Behavior.h"
#include "map.h"
#include "State.h"

const double Behavior::SPEED_LIMIT = 49.5;
const double Behavior::SPEED_STEP = 0.224;
const int Behavior::SAFE_DIST = 30;
const int Behavior::CHANGE_LANE_DIST = 20;

Behavior::Behavior(State& S) {
	s_ = &S;
}

Behavior::Behavior(const Behavior& orig) {
}

Behavior::~Behavior() {
}

bool Behavior::check_proximity() {
	bool too_close = false;
	double dist = SAFE_DIST + 1.0;
	double obstacle_speed = SPEED_LIMIT + 1.0;
	for (int i = 0; i < s_->sensor_fusion.size(); ++i) {
		float d = s_->sensor_fusion[i][State::SENSOR_FUSION_D];
		if (d > (2 + 4 * s_->lane + 2) || d < (2 + 4 * s_->lane - 2))
			continue; // car on a different lane
		double vx = s_->sensor_fusion[i][State::SENSOR_FUSION_VX];
		double vy = s_->sensor_fusion[i][State::SENSOR_FUSION_VY];
		double check_speed = sqrt(vx * vx + vy * vy);
		double check_car_s = s_->sensor_fusion[i][State::SENSOR_FUSION_S];
		// we work in the past so account for that
		check_car_s += ((double) s_->prv_path_x.size() * 0.02 * check_speed);
		if (check_car_s < s_->s || // car is behind us
				check_car_s - s_->s > dist) // there is another car closer
			continue;
		dist = check_car_s - s_->s;
		if (dist > SAFE_DIST)
			continue;
		obstacle_speed = mps2mph(check_speed);
		std::cout << "TOO CLOSE: SPEAD:" << obstacle_speed << " dist:" << dist << " pdist:" << prev_dist_ << std::endl;
		// if distance has increased accelerate, this will smoothly match the speed of the car in front
		too_close = (dist <= prev_dist_);
		prev_dist_ = dist;
	}

	if (too_close) {
		double rel_speed = mph2mps(s_->vel - obstacle_speed);
		double decel = std::max(SPEED_STEP, mps2mph((rel_speed * rel_speed) / (dist * dist)));
		std::cout << "SPEED DEC: " << s_->vel << " DECEL:" << decel << std::endl;
		s_->vel -= decel;
	} else if (s_->vel < SPEED_LIMIT) {
		std::cout << "SPEED INC: " << s_->vel << std::endl;
		s_->vel += SPEED_STEP;
		prev_dist_ = SAFE_DIST; // reset, no longer close to a car
	}
	// return true if need to explore lane changing
	return (too_close && s_->vel < SPEED_LIMIT);
}

void Behavior::explore_change_lane() {
	// abort if a lane change is in progress
	if (s_->d < 1 + 4*s_->lane || s_->d > 3 + 4*s_->lane)
		return;
	bool change_left = true;
	bool change_right = true;
	for (int i = 0; i < s_->sensor_fusion.size(); ++i) {
		float d = s_->sensor_fusion[i][State::SENSOR_FUSION_D];
		int lane = d / 4; // lane width is 4, integer division of d by 4 will give lane index
		double vx = s_->sensor_fusion[i][State::SENSOR_FUSION_VX];
		double vy = s_->sensor_fusion[i][State::SENSOR_FUSION_VY];
		double check_speed = sqrt(vx * vx + vy * vy);
		double check_car_s = s_->sensor_fusion[i][State::SENSOR_FUSION_S];
		// we work in the past so account for that
		check_car_s += ((double) s_->prv_path_x.size() * 0.02 * check_speed);
		double dist = std::abs(check_car_s - s_->s);
		// make sure the car is far to the front or 70% as far behind (since we'll accelerate 0.7 is fine)
		bool car_is_far = (check_car_s > s_->s + CHANGE_LANE_DIST) || (check_car_s < s_->s - 0.7 * CHANGE_LANE_DIST);
		/** change if 
		 * 1) a previous car hasn't invalidaded the direction
		 * 2) a car is not in the destination lane OR it is far away
		 * 3) we are not at the destination-most lane already
		 **/
		change_left = (change_left && (lane != s_->lane - 1 || car_is_far) && s_->lane > 0);
		change_right = (change_right && (lane != s_->lane + 1 || car_is_far) && s_->lane < Map::LANES - 1);
	}
	std::cout << "LANE CHANGE FROM " << s_->lane << " TO ";
	// prioritize left over right
	if (change_left) {
		std::cout << "left ";
		s_->lane -= 1;
	} else if (change_right) {
		std::cout << "right ";
		s_->lane += 1;
	}
	std::cout << s_->lane << std::endl;
}

double Behavior::mps2mph(double ms) {
	return ms * 2.23694;
}

double Behavior::mph2mps(double mph) {
	return mph * 0.44704;
}
