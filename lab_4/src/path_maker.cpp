#include "path_maker.h"

using namespace std;

PathMaker::PathMaker(const string& topic) {
	_sub = _nh.subscribe(topic, 1000, &PathMaker::callback, this);
	_pub = _nh.advertise<nav_msgs::Path>("/path", 1000);
}

void PathMaker::callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	_path.push_back(*msg);
	nav_msgs::Path path_msg;
	path_msg.poses= _path;
	path_msg.header.stamp = ros::Time::now();
	path_msg.header.frame_id = "base";
	_pub.publish(path_msg);
}