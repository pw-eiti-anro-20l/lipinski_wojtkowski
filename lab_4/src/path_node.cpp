#include <ros/ros.h>
#include "path_maker.h"

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "Path");
	if(argc < 2) {
		ROS_ERROR("Topic name required");
		return 1;
	}
	ROS_INFO("[PATH_NODE] Node Init.");
	PathMaker path(argv[1]);
	ros::spin();
	return 0;
}