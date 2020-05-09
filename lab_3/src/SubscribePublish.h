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
	
	struct Robot {
		double a1;
		double a2;
		double alpha2;
		double d3;
		double theta1;
		double theta2;
		double limit1;
		double limit2;
		double limit3;
	} _robot;
	
	void _publish(const ros::Publisher& pub, const KDL::Frame& pos);
};

#endif