#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include "SubscribePublish.h"

using namespace std; 

int main(int argc, char** argv) {
	ros::init(argc, argv, "KDL_DKIN");
	SubscribePublish SPObject;
	ros::spin();
}
