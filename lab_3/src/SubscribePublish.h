#ifndef SUBSCRIBEPUBLISH_H
#define SUBSCRIBEPUBLISH_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include "Robot.h"

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
	Robot _robot;
	void _publish(ros::Publisher pub, KDL::Frame pos);
}

#endif