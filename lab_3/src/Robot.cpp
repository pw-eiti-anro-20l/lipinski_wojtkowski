#include "Robot.h"

KDL::Frame Robot::transform_joint1(double dtheta1) {
	return KDL::Frame::DH_Craig1989(0, 0, 0, theta1 + dtheta1);
}

KDL::Frame Robot::transform_joint2(double dtheta2) {
	return KDL::Frame::DH_Craig1989(a1, 0, 0, theta2 + dtheta2);
}

KDL::Frame Robot::transform_joint3(double dd3) {
	return KDL::Frame::DH_Craig1989(a2, alpha2, d3 + dd3, 0);
}
