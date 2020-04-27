#ifndef JINTPROVIDER_H
#define JINTPROVIDER_H

#include <ros/ros.h>
#include <lab_4/jint_control_srv.h>
#include <vector>

using namespace std;

class JintProvider {
public:
	JintProvider();
	bool callback(lab_4::jint_control_srv::Request& req, lab4::jint_control_srv::Response& resp);
private:
	ros::NodeHandle _nh;
	ros::ServiceServer _server;
	ros::Publisher _pub;
	vector<double> _position;
	enum Interpolation {LIN, CUBE} iType;
	
	double interpolate(const vector<double>& params, double t);
	void calc_cube_params(double x1, double x0, double dt, double t0, vector<double>& vect);
	void _publish(const vector<double>& position);
};

#endif