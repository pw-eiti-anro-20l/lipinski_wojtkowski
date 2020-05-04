#include <ros/ros.h>
#include <lab_4/jint_control_srv.h>

using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "JCMD");
	
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<lab_4::jint_control_srv>("jint_srv");
	
	lab_4::jint_control_srv service;
	service.request.joint_states = {1.57, 1.57, -0.25};
	service.request.mov_duration = ros::Duration(5, 0);
	service.request.int_type = "lin";
	ROS_INFO("Request init");
	if(client.call(service))
		ROS_INFO("%s", service.response.msg.c_str());
	else
		ROS_INFO("ERROR");
	service.request.joint_states = {0, 0, 0};
	service.request.mov_duration = ros::Duration(5, 0);
	service.request.int_type = "lin";
	ROS_INFO("Request init");
	if(client.call(service))
		ROS_INFO("%s", service.response.msg.c_str());
	else
		ROS_INFO("ERROR");
	return 0;
}