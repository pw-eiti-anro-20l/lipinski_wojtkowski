#ifndef SUBSCRIBEPUBLISH_H
#define SUBSCRIBEPUBLISH_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl/frames.hpp>

class SubscribePublish {
public:
	SubscribePublish();
	void callback(const sensor_msgs::JointState::ConstPtr& msg);
private:
	ros::NodeHandle _n;
	ros::Subscriber _sub;
	ros::Publisher _pub1;
	ros::Publisher _pub2;
	ros::Publisher _pub3;
	
	class Robot {
	public:
		double a1;
		double a2;
		double alpha2;
		double d3;
		double theta1;
		double theta2;
		
		Robot() {}
		Robot(double params[]) : a1(params[0]), a2(params[1]), alpha2(params[2]), d3(params[3]), theta1(params[4]), theta2(params[5]) {}
		KDL::Frame transform_joint1(double dtheta1);
		KDL::Frame transform_joint2(double dtheta2);
		KDL::Frame transform_joint3(double dd3);
	} _robot;
	
	void _publish(const ros::Publisher& pub, const KDL::Frame& pos);
};

#endif