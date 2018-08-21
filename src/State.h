/* 
 * File:   State.h
 * Author: Georgios Varisteas <georgios.varisteas@uni.lu>
 *
 * Created on August 20, 2018, 6:39 PM
 */

#ifndef STATE_H
#define STATE_H

#include <vector>
#include "json.hpp"

class State {
public:

	typedef enum SENSOR_FUSION {
		SENSOR_FUSION_ID = 0,
		SENSOR_FUSION_X = 1,
		SENSOR_FUSION_Y = 2,
		SENSOR_FUSION_VX = 3,
		SENSOR_FUSION_VY = 4,
		SENSOR_FUSION_S = 5,
		SENSOR_FUSION_D = 6
	} SENSOR_FUSION;

	State();
	State(const State& orig);
	virtual ~State();

	void fromJson(nlohmann::json& j);

	double vel;
	double x;
	double y;
	double s;
	double d;
	double yaw;
	double speed;
	int lane;

	std::vector<double> prv_path_x;
	std::vector<double> prv_path_y;

	double end_path_s;
	double end_path_d;

	std::vector<std::vector<double>> sensor_fusion;
private:

};

#endif /* STATE_H */

