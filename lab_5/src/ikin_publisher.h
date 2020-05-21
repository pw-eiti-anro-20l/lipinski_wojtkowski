#ifndef IKIN_PUBLISHER_H
#define IKIN_PUBLISHER_H

#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

using namespace std;

class IKINPublisher {
public:
	IKINPublisher(const string &sub_topic);
	void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
private:
	ros::NodeHandle _nh;
	ros::Subscriber _sub;
	ros::Publisher _pub;
	ros::Publisher _pub_manip;
	vector<double> joint_pos;
	vector<double> manip_pos;
	double _a1;
	double _a2;
	double _theta1;
	double _theta2;
	double _alpha2;
	double _d3;
	
	void _calc_manip_pos();
	void _calc_thetas(double x, double y);
	void _publish();
};

#endif //IKIN_PUBLISHER_H