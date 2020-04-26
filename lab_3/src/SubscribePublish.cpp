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
}
||||||| merged common ancestors
=======
#include "SubscribePublish.h"
#include <vector>

SubscribePublish::SubscribePublish() {
	_sub = _n.subscribe("joint_states", 1000, &SubscribePublish::callback, this);
	_pub1 = _n.advertise<geometry_msgs::PoseStamped>("/joint1", 1000);
	_pub2 = _n.advertise<geometry_msgs::PoseStamped>("/joint2", 1000);
	_pub3 = _n.advertise<geometry_msgs::PoseStamped>("/joint3", 1000);
	double params[6];
	_n.getParam("/kdl_dkin/a1", params[0]);
	_n.getParam("/kdl_dkin/a2", params[1]);
	_n.getParam("/kdl_dkin/alpha2", params[2]);
	_n.getParam("/kdl_dkin/d3", params[3]);
	_n.getParam("/kdl_dkin/theta1", params[4]);
	_n.getParam("/kdl_dkin/theta2", params[5]);
	_robot = Robot(params);
}

void SubscribePublish::callback(const sensor_msgs::JointState::ConstPtr& msg) {
	std::vector<double> pos_vec = msg->position;
	KDL::Frame joint1_state = _robot.transform_joint1(pos_vec[0]);
	KDL::Frame joint2_state = joint1_state * _robot.transform_joint2(pos_vec[1]);
	KDL::Frame joint3_state = joint2_state * _robot.transform_joint3(pos_vec[2]);
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

KDL::Frame SubscribePublish::Robot::transform_joint1(double dtheta1) {
	return KDL::Frame::DH_Craig1989(0, 0, 0, theta1 + dtheta1);
}

KDL::Frame SubscribePublish::Robot::transform_joint2(double dtheta2) {
	return KDL::Frame::DH_Craig1989(a1, 0, 0, theta2 + dtheta2);
}

KDL::Frame SubscribePublish::Robot::transform_joint3(double dd3) {
	return KDL::Frame::DH_Craig1989(a2, alpha2, d3 + dd3, 0);
}
>>>>>>> origin/MW
