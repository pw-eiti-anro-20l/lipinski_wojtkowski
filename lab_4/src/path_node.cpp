#include <ros/ros.h>
#include "path_maker.h"

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "Path");
	if(argc < 3) {
		ROS_ERROR("use: source_topic destinton_topic");
		return 1;
	}
	ROS_INFO("[PATH_NODE] Node Init.");
	PathMaker path(argv[1], argv[2]);
	ros::spin();
	return 0;
}