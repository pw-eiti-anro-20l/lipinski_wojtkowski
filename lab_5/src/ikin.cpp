#include <ros/ros.h>
#include "ikin_publisher.h"

using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "IKIN");
	ROS_INFO("[IKIN] Node init");
	IKINPublisher ikin("/oint");
	ros::spin();
	return 0;
}