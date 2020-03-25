#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <iostream>
#include <cmath>
#include <fstream>

int main( int argc, char** argv) {
	double a1 = 1, a2 = 1, d3 = 0, alpha1 = M_PI, theta1 = 0, theta2 = 0;
	double dh_params[3][4] = {{0, 0, 0, theta1}, {a1, 0, 0, theta1}, {a2, alpha1, d3, 0}};
	std::ofstream  config;
	config.open("./src/lipinski-wojtkowski/LAB_2/config/config.yaml");
	if(!config.is_open()) {
		std::cerr <<  "Couldn't open config.yaml file\n";
		return -1;
	}
	config <<"---\n";
	for(int i = 0; i < 3; ++i) {
		KDL::Frame frame = KDL::Frame::DH(dh_params[i][0], dh_params[i][1], dh_params[i][2], dh_params[i][3]);
		double roll, pitch, yaw;
		frame.M.GetRPY(roll, pitch, yaw);
		config << "joint" << i+1 << ": {\n";
		config << "\trpy: \"" << roll << " " << pitch << " " << yaw << "\",\n";
		config << "\tpos: \"" << frame.p.x() << " " << frame.p.y() << " " << frame.p.z() << "\",\n}\n";
	}
	config << "...";
	config.close();
}