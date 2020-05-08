#include "JintProvider.h"
#include <sensor_msgs/JointState.h>
#include <vector>

using namespace std;

JintProvider::JintProvider() {
	_server = _nh.advertiseService("jint_srv", &JintProvider::callback, this);
	_pub = _nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	_position = {0, 0, 0};
	_limits = vector<double>(3);
	_limits[0] = M_PI * _nh.param("/joint1_lim", 1.0);
	_limits[1] = M_PI * _nh.param("/joint2_lim", 1.0);
	_limits[2] = _nh.param("/d3", 0.5) * _nh.param("/joint3_lim", 1.0);
	_publish(_position);
	ROS_INFO("Provider_init");
}

bool JintProvider::callback(lab_4::jint_control_srv::Request& req, lab_4::jint_control_srv::Response& resp) {
	ROS_INFO("Callback");
	vector<double> final_joint_states = req.joint_states;
	ros::Duration mov_duration = req.mov_duration;
	string int_type = req.int_type;
	if(int_type == "lin")
		iType = Interpolation::LIN;
	else if(int_type == "cube")
		iType = Interpolation::CUBE;
	else {
		ROS_ERROR("[Jint Service]: Interpolation \"%s\" not supported", int_type.c_str());
		return false;
	}
	
	if(mov_duration.toSec() <= 0) {
		ROS_ERROR("[Jint Service]: Time must be greater than 0");
		return false;
	}
	
	for(int i = 0; i < 3; ++i)
		if(_limits[i] < fabs(final_joint_states[i])) {
			ROS_ERROR("[Jint Service]: Goal (%.2f) exceeds joint limint no. %d (%.2f)", final_joint_states[i], i + 1, _limits[i]);
			return false;
		}
			
			
	
	ros::Time start = ros::Time::now();
	vector<vector<double>> interpol_params;
	for(int i = 0; i < 3; ++i) {
		vector<double> params;
		switch(iType) {
		case Interpolation::LIN: {
			double v = (final_joint_states[i] - _position[i])/mov_duration.toSec();
			params.push_back(_position[i]);
			params.push_back(v);
		}
			break;
		case Interpolation::CUBE:
			calc_cube_params(final_joint_states[i], _position[i], mov_duration.toSec(), params);
			break;
		}
		interpol_params.push_back(params);
	}
	
	while(ros::Time::now() <= start + mov_duration) {
		vector<double> new_pos;
		double t = ros::Time::now().toSec() - start.toSec();
		for(int i = 0; i < 3; ++i)
			new_pos.push_back(interpolate(interpol_params[i], t));
		_position = new_pos;
		_publish(_position);
	}
	resp.status = true;
	resp.msg="Interpolation successful";
	return true;
}

void JintProvider::_publish(const vector<double>& position) {
	sensor_msgs::JointState msg;
	msg.position = position;
	msg.name = {"joint1", "joint2", "joint3"};
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "base";
	_pub.publish(msg);
}

void JintProvider::calc_cube_params(double x1, double x0, double dt, vector<double>& vect) {
	double dx = x1 - x0;
	double a0 = x0;
	double a1 = 0;
	double a2 = 3 * dx / (dt * dt);
	double a3 = -2 * dx / (dt * dt * dt);
	vect.push_back(a0);
	vect.push_back(a1);
	vect.push_back(a2);
	vect.push_back(a3);
}

double JintProvider::interpolate(const vector<double>& params, double t) {
	double sum = 0;
	for(int i = 0; i < params.size(); ++i)
		sum += params[i] * pow(t, i);
	return sum;
}