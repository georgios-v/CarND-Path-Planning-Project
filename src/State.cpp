/* 
 * File:   State.cpp
 * Author: Georgios Varisteas <georgios.varisteas@uni.lu>
 * 
 * Created on August 20, 2018, 6:39 PM
 */

#include "State.h"
#include "map.h"

State::State(){
}

void State::fromJson(nlohmann::json& j) {
	// j[1] is the data JSON object
	x = j[1]["x"];
	y = j[1]["y"];
	s = j[1]["s"];
	d = j[1]["d"];
	yaw = Map::deg2rad(j[1]["yaw"]);
	speed = j[1]["speed"];

	// Previous path data given to the Planner
	prv_path_x = j[1]["previous_path_x"].get<std::vector<double>>();
	prv_path_y = j[1]["previous_path_y"].get<std::vector<double>>();

	// Previous path's end s and d values 
	end_path_s = j[1]["end_path_s"];
	end_path_d = j[1]["end_path_d"];
	
	// Sensor Fusion Data, a list of all other cars on the same side of the road.
	sensor_fusion = j[1]["sensor_fusion"].get<std::vector<std::vector<double>>>();
}

State::State(const State& orig) {
}

State::~State() {
}

