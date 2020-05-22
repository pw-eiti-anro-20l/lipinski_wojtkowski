#include "ikin_publisher.h"
#include <sensor_msgs/JointState.h>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>

using namespace std;

IKINPublisher::IKINPublisher(const string& sub_topic) {
	_sub = _nh.subscribe(sub_topic, 1000, &IKINPublisher::callback, this);
	_pub = _nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
	_pub_manip = _nh.advertise<geometry_msgs::PoseStamped>("/manip_pos", 1000);
	_a1 = _nh.param("/a1", 1.0);
	_a2 = _nh.param("/a2", 1.0);
	_alpha2 = _nh.param("/alpha2", 3.1415);
	_d3 = _nh.param("/d3", 1.0);
	_theta1 = _nh.param("/theta1", 0.0);
	_theta2 = _nh.param("/theta2", 0.0);
	joint_pos = {0, 0, 0};
	manip_pos = {0, 0, 0};
	_calc_manip_pos();
	_publish();
}

void IKINPublisher::callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	double x = msg->pose.position.x, y = msg->pose.position.y, z = msg->pose.position.z;
	if(sqrt(x*x+y*y) > _a1 + _a2 || sqrt(x*x+y*y) < _a1 - _a2 || z < -_d3 || z > 0) {
		ROS_ERROR("[IKIN] Position (%.2f, %.2f, %.2f) unreachable", x, y, z);
	} else {
		_calc_thetas(msg->pose.position.x, msg->pose.position.y);
		joint_pos[2] = -_d3/2-msg->pose.position.z;
		//ROS_INFO("[IKIN] New joints: %f, %f, %f", joint_pos[0], joint_pos[1], joint_pos[2]);
		_calc_manip_pos();
		//ROS_INFO("[IKIN] Pose error; %f,%f, %f", msg->pose.position.x - manip_pos[0], msg->pose.position.y - manip_pos[1], msg->pose.position.z - manip_pos[2]);
		_publish();
	}
}

void IKINPublisher::_calc_thetas(double x, double y) {
	double c_t2 = (x*x + y*y - _a1*_a1 - _a2*_a2)/(2*_a1*_a2);
	double t2 = acos(c_t2);
	t2 *= (fabs(t2 - joint_pos[1]) < fabs(t2 + joint_pos[1])) ? 1 : -1;
	double t1 = atan2(-_a2 * sin(t2) * x + (_a1 + _a2*c_t2)*y, (_a1+_a2*c_t2)*x+_a2*sin(t2)*y);
	
	joint_pos[0] = t1 - _theta1;
	joint_pos[1] = t2 - _theta2;
}

void IKINPublisher::_calc_manip_pos() {
	KDL::Frame joint1_state = KDL::Frame::DH_Craig1989(0, 0, 0, _theta1 + joint_pos[0]);
	KDL::Frame joint2_state = joint1_state * KDL::Frame::DH_Craig1989(_a1, 0, 0, _theta2 + joint_pos[1]);
	KDL::Frame joint3_state = joint2_state * KDL::Frame::DH_Craig1989(_a2, _alpha2, joint_pos[2], 0);
	KDL::Frame manip = joint3_state * KDL::Frame::DH_Craig1989(0, 0, _d3/2, 0);
	manip_pos[0] = manip.p.x();
	manip_pos[1] = manip.p.y();
	manip_pos[2] = manip.p.z();
}

void IKINPublisher::_publish() {
	sensor_msgs::JointState msg;
	msg.position = joint_pos;
	msg.name = {"joint1", "joint2", "joint3"};
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "base";
	_pub.publish(msg);
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.pose.position.x = manip_pos[0];
	pose_msg.pose.position.y = manip_pos[1];
	pose_msg.pose.position.z = manip_pos[2];
	pose_msg.header.stamp = ros::Time::now();
	pose_msg.header.frame_id = "base";
	_pub_manip.publish(pose_msg);
}