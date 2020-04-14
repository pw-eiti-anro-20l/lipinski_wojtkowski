#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"

using namespace std; 

class Robot {
public:
	static double a1;
	static double a2;
	static double alpha2;
	static double d3;
	static double theta1;
	static double theta2;
	
	static void set_params(double[] params); //{a1 a2 alpha2 d3 theta1 theta2}
	static KDL::Frame transform_joint1(double dtheta1);
	static KDL::Frame transform_joint2(double dtheta2);
	static KDL::Frame transform_joint3(double dd3);
};

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

int main(int argc, char** argv) {
	ros::init(argc, argv, "KDL_DKIN");
	ros::NodeHandle n;
	double params[6];
	n.getParam("a1", params[0]);
	n.getParam("a2", params[1]);
	n.getParam("alpha1", params[2]);
	n.getParam("d3", params[3]);
	n.getParam("theta1", params[4]);
	n.getParam("theta2", params[5]);
	Robot::set_params(params);
	ros::Subscriber sub = n.subscribe("joint_states", 1000, joint_state_callback);
	ros::spin();
}

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
	vector<double> pos_vec = msg->position;
	KDL::Frame joint1_state = Robot::transform_joint1(pos_vec[0]);
	KDL::Frame joint2_state = joint1_state * Robot::transform_joint2(pos_vec[1]);
	KDL::Frame joint3_state = joint2_state * Robot::transform_joint3(pos_vec[2]);
	/**TODO: Publisher **/
}

void Robot::set_params(double[] params) {
	a1 = params[0];
	a2 = params[1];
	alpha2 = params[2];
	d3 = params[3];
	theta1 = params[4];
	theta2 = params[5];
}

KDL::Frame Robot::transform_joint1(double dtheta1) {
	return KDL::Frame::DH_Craig1998(0, 0, 0, theta1 + dtheta1);
}

KDL::Frame Robot::transform_joint2(double dtheta2) {
	return KDL::Frame::DH_Craig1998(a1, 0, 0, theta2 + dtheta2);
}

KDL::Frame Robot::transform_joint3(double dd3) {
	return KDL::Frame::DH_Craig1998(a2, alpha2, d3 + dd3, 0);
}