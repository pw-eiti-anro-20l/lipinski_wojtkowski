#include "JintProvider.h"
#include <sensor_msgs/JointState.h>
#include <vector>

using namespace std;

JintProvider::JintProvider() {
	_server = _nh.advertiseService("jint_srv", &JintProvider::callback, this);
	_pub = _nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	_position = {0, 0, 0};
	_publish(_position);
	_publish(_position);
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
		resp.status = false;
		resp.msg = "[ERROR]: Interpolation \"" + int_type + "\" not supported"; 
		return false;
	}
	
	if(mov_duration.toSec() <= 0) {
		resp.status = false;
		resp.msg="[ERROR]: Time must be greater than 0.";
		return false;
	}
	
	ros::Time start = ros::Time::now();
	vector<vector<double>> interpol_params;
	for(int i = 0; i < 3; ++i) {
		vector<double> params;
		switch(iType) {
		case Interpolation::LIN: {
			double v = (final_joint_states[i] - _position[i])/mov_duration.toSec();
			params.push_back(_position[i] - v * start.toSec());
			params.push_back(v);
		}
			break;
		case Interpolation::CUBE:
			calc_cube_params(final_joint_states[i], _position[i], mov_duration.toSec(), start.toSec(), params);
			break;
		}
		interpol_params.push_back(params);
	}
	
	while(ros::Time::now() <= start + mov_duration) {
		vector<double> new_pos;
		double t = ros::Time::now().toSec();
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

void JintProvider::calc_cube_params(double x1, double x0, double dt, double t0, vector<double>& vect) {
	double t1 = t0 + dt;
	double a0 = (x0*t1-t0*x1)/dt-t1*t0*(t1+t0)*(x1-x0)/pow(dt, 3);
	double a1 = (x1-x0)/dt + (x1-x0)/pow(dt, 3)*(t1*t1+4*t1*t0+t0*t0);
	double a2 = 3*(x1-x0)*(t1+t0)/pow(dt, 3);
	double a3 = 2*(x1-x0)/pow(dt, 3);
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