#include <ros/ros.h>
#include "path_maker.h"

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "Path");
	if(argc < 2) {
		ROS_ERROR("Topic name requird");
		return 1;
	}
	PathMaker path(argv[1]);
	ros::spin();
	return 0;
}