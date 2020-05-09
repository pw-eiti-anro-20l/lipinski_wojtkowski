#ifndef PATH_MAKER_H
#define PATH_MAKER_H

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>
#include <vector>


class PathMaker {
public:
	PathMaker(const std::string& topic_name);
	void callback(const geometry_msgs::PoseStamped:: ConstPtr& msg);
	
private:
	ros::NodeHandle _nh;
	ros::Subscriber _sub;
	ros::Publisher _pub;
	std::vector<geometry_msgs::PoseStamped> _path;
};

#endif //PATH_MAKER_H