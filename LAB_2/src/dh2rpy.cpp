#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <map>
#include <utility>
#include <stdexcept>

std::pair<std::string, double> getArg(std::string str);
bool check_arg_map(const std::map<std::string, double> & map);

int main( int argc, char** argv) {
	std::map<std::string, double> arg_map;
	for(int i = 1; i < argc - 2; ++i) {
		arg_map.insert(getArg(argv[i]));
	}
	if(!check_arg_map(arg_map)) {
		std::cerr << "Incorrect or missing arguments!\n";
		return -1;
	}
	double dh_params[3][4] = {{0, 0, 0, arg_map["theta1"]},
	                           {arg_map["a1"], 0, 0, arg_map["theta2"]},
							   {arg_map["a2"], arg_map["alpha2"], arg_map["d3"], 0}};
	
	std::ofstream  config;
	config.open("../../../src/lipinski-wojtkowski/LAB_2/config/config.yaml", std::ofstream::out | std::ofstream::trunc);
	if(!config.is_open()) {
		std::cerr <<  "Couldn't open config.yaml file\n";
		return -1;
	}
	config <<"---\n";
	config << "a1: " << arg_map["a1"] << "\n";
	config << "a2: " << arg_map["a2"] << "\n";
	config << "d3: " << arg_map["d3"] << "\n";
	for(int i = 0; i < 3; ++i) {
		KDL::Frame frame = KDL::Frame::DH_Craig1989(dh_params[i][0], dh_params[i][1], dh_params[i][2], dh_params[i][3]);
		double roll, pitch, yaw;
		frame.M.GetRPY(roll, pitch, yaw);
		config << "joint" << i+1 << ": {\n";
		config << " rpy: \"" << roll << " " << pitch << " " << yaw << "\"," << "\n";
		config << " pos: \"" << frame.p.x() << " " << frame.p.y() << " " << frame.p.z() << "\",\n}\n";
	}
	config << "...";
	config.close();
	return 0;
}

std::pair<std::string, double> getArg(std::string str) {
	size_t split_point = str.find_first_of('=');
	if(str[0] != '-' || split_point == std::string::npos)
		throw std::invalid_argument(str);
	double value = stod(str.substr(split_point + 1));
	std::string name = str.substr(1, split_point - 1);
	return std::make_pair(name, value);
}

bool check_arg_map(const std::map<std::string, double> & arg_map) {
	std::string keys[] = {"a1", "a2", "d3", "alpha2", "theta1", "theta2"};
	for(int i = 0; i < 6; ++i) {
		if(arg_map.find(keys[i]) == arg_map.end())
			return false;
	}
	return true;
}