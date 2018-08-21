/* 
 * File:   Trajectory.cpp
 * Author: Georgios Varisteas <georgios.varisteas@uni.lu>
 * 
 * Created on August 20, 2018, 5:56 PM
 */

#include "Behavior.h"
#include "map.h"
#include "spline.h"
#include "Trajectory.h"


Trajectory::Trajectory(State& S) {
	s_ = &S;
}

Trajectory::Trajectory(const Trajectory& orig) {
}

Trajectory::~Trajectory() {
}

void Trajectory::generate(Map& map) {
	int previous_path_size = s_->prv_path_x.size();
	if (previous_path_size < 2) {
		double car_yaw = Map::rad2deg(s_->yaw);
		double prev_car_x = std::cos(car_yaw);
		double prev_car_y = std::sin(car_yaw);

		ptsx_.push_back(prev_car_x);
		ptsx_.push_back(s_->x);

		ptsy_.push_back(prev_car_y);
		ptsy_.push_back(s_->y);
	} else {
		s_->x = s_->prv_path_x[previous_path_size - 1];
		s_->y = s_->prv_path_y[previous_path_size - 1];

		double ref_x_prev = s_->prv_path_x[previous_path_size - 2];
		double ref_y_prev = s_->prv_path_y[previous_path_size - 2];
		s_->yaw = std::atan2(s_->y - ref_y_prev, s_->x - ref_x_prev);

		ptsx_.push_back(ref_x_prev);
		ptsx_.push_back(s_->x);

		ptsy_.push_back(ref_y_prev);
		ptsy_.push_back(s_->y);
	}

	std::vector<double> next_wp0 = map.getXY(s_->s + Behavior::SAFE_DIST, (2 + 4 * s_->lane));
	std::vector<double> next_wp1 = map.getXY(s_->s + 2 * Behavior::SAFE_DIST, (2 + 4 * s_->lane));
	std::vector<double> next_wp2 = map.getXY(s_->s + 3 * Behavior::SAFE_DIST, (2 + 4 * s_->lane));

	ptsx_.push_back(next_wp0[0]);
	ptsx_.push_back(next_wp1[0]);
	ptsx_.push_back(next_wp2[0]);

	ptsy_.push_back(next_wp0[1]);
	ptsy_.push_back(next_wp1[1]);
	ptsy_.push_back(next_wp2[1]);

	for (int i = 0; i < ptsx_.size(); ++i) {
		double shift_x = ptsx_[i] - s_->x;
		double shift_y = ptsy_[i] - s_->y;
		ptsx_[i] = (shift_x * std::cos(0 - s_->yaw) - shift_y * std::sin(0 - s_->yaw));
		ptsy_[i] = (shift_x * std::sin(0 - s_->yaw) + shift_y * std::cos(0 - s_->yaw));
	}
}

std::vector<std::vector<double>> Trajectory::getSpline() {
	tk::spline s;

	std::vector<double> next_x_vals;
	std::vector<double> next_y_vals;

	s.set_points(ptsx_, ptsy_);

	int previous_path_size = s_->prv_path_x.size();
	for (int i = 0; i < previous_path_size; ++i) {
		next_x_vals.push_back(s_->prv_path_x[i]);
		next_y_vals.push_back(s_->prv_path_y[i]);
	}

	double target_x = (double) Behavior::SAFE_DIST;
	double target_y = s(target_x);
	double target_dist = std::sqrt((target_x * target_x) + (target_y * target_y));

	double x_add_on = 0.0;

	for (int i = 1; i <= Map::WAYPOINT_LENGTH - previous_path_size; ++i) {
		double N = (target_dist / (0.02 * s_->vel / 2.24));
		double x_point = x_add_on + target_x / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = (x_ref * std::cos(s_->yaw) - y_ref * std::sin(s_->yaw));
		y_point = (x_ref * std::sin(s_->yaw) + y_ref * std::cos(s_->yaw));

		x_point += s_->x;
		y_point += s_->y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}

	return
	{
		next_x_vals, next_y_vals
	};
}
