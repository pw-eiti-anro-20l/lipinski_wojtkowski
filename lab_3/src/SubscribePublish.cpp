#include "SubscribePublish.h"
#include <vector>

SubscribePublish::SubscribePublish() {
	_sub = _n.subscribe("joint_states", 1000, &SubscribePublish::callback, this);
	_pub1 = _n.advertise<geometry_msgs::PoseStamped>("/joint1", 1000);
	_pub2 = _n.advertise<geometry_msgs::PoseStamped>("/joint2", 1000);
	_pub3 = _n.advertise<geometry_msgs::PoseStamped>("/joint3", 1000);
	_n.getParam("/a1", _robot.a1);
	_n.getParam("/a2", _robot.a2);
	_n.getParam("/alpha2", _robot.alpha2);
	_n.getParam("/d3", _robot.d3);
	_n.getParam("/theta1", _robot.theta1);
	_n.getParam("/theta2", _robot.theta2);
}

void SubscribePublish::callback(const sensor_msgs::JointState::ConstPtr& msg) {
	std::vector<double> pos_vec = msg->position;
	KDL::Frame joint1_state = KDL::Frame::DH_Craig1989(0, 0, 0, _robot.theta1 + pos_vec[0]);
	KDL::Frame joint2_state = joint1_state * KDL::Frame::DH_Craig1989(_robot.a1, 0, 0, _robot.theta2 + pos_vec[1]);
	KDL::Frame joint3_state = joint2_state * KDL::Frame::DH_Craig1989(_robot.a2, _robot.alpha2, _robot.d3 + pos_vec[2], 0);
	_publish(_pub1, joint1_state);
	_publish(_pub2, joint2_state);
	_publish(_pub3, joint3_state);
}

void SubscribePublish::_publish(const ros::Publisher& pub, const KDL::Frame& pos) {
	geometry_msgs::PoseStamped msg;
	pos.M.GetQuaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
	msg.pose.position.x = pos.p.x();
	msg.pose.position.y = pos.p.y();
	msg.pose.position.z = pos.p.z();
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "base";
	pub.publish(msg);
}
