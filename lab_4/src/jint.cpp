#include <ros/ros.h>
#include "JintProvider.h"

using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "JINT");
	JintProvider provider;
	ros::spin();
	return 0;
}