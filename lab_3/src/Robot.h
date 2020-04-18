#ifndef ROBOT_H
#define ROBOT_H

#include <KDL/frames.hpp>

class Robot {
public:
	double a1;
	double a2;
	double alpha2;
	double d3;
	double theta1;
	double theta2;
	
	Robot(double[] params) : a1(params[0]), a2(params[1]), alpha2(params[2]), d3(params[3]), theta1(params[4]), theta2(params[5]) {}
	KDL::Frame transform_joint1(double dtheta1);
	KDL::Frame transform_joint2(double dtheta2);
	KDL::Frame transform_joint3(double dd3);
};

#endif