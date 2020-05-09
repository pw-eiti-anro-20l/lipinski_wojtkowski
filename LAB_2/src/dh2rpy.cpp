#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <iostream>
#include <cmath>
#include <fstream>
#include <string>
#include <map>
#include <utility>
#include <stdexcept>
#include <cctype>

using namespace std;

pair<string, double> getArg(string str);
string get_dir(string str);
pair<string, string> split_arg(string str);
bool check_arg_map(const map<string, double> & map);

int main( int argc, char** argv) {
	map<string, double> arg_map;
	string config_dir;
	for(int i = 1; i < argc - 2; ++i) {
		pair<string, string> arg = split_arg(argv[i]);
		if(isdigit(arg.second[0]))
			arg_map.insert(make_pair(arg.first, stod(arg.second)));
		else if(arg.first == "dir")
			config_dir = "../../../src/lipinski-wojtkowski/" + arg.second + "/config/config.yaml";
		else
			throw invalid_argument("No directory");
	}
	if(!check_arg_map(arg_map)) {
		cerr << "Incorrect or missing arguments!\n";
		return -1;
	}
	double dh_params[3][4] = {{0, 0, 0, arg_map["theta1"]},
	                           {arg_map["a1"], 0, 0, arg_map["theta2"]},
							   {arg_map["a2"], arg_map["alpha2"], arg_map["d3"], 0}};
	
	ofstream  config;
	config.open(config_dir, ofstream::out | ofstream::trunc);
	if(!config.is_open()) {
		cerr <<  "Couldn't open config.yaml file\n";
		return -1;
	}
	config <<"---\n";
	config << "a1: " << arg_map["a1"] << "\n";
	config << "a2: " << arg_map["a2"] << "\n";
	config << "d3: " << arg_map["d3"] << "\n";
	config << "alpha2: " << arg_map["alpha2"] << "\n";
	config << "theta1: " << arg_map["theta1"] << "\n";
	config << "theta2: " << arg_map["theta2"] << "\n";
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

pair<string, string> split_arg(string str) {
	size_t split_point = str.find_first_of('=');
	if(str[0] != '-' || split_point == string::npos)
		throw invalid_argument(str);
	return make_pair(str.substr(1, split_point - 1), str.substr(split_point + 1));
}

bool check_arg_map(const map<string, double> & arg_map) {
	string keys[] = {"a1", "a2", "d3", "alpha2", "theta1", "theta2"};
	for(int i = 0; i < 6; ++i) {
		if(arg_map.find(keys[i]) == arg_map.end())
			return false;
	}
	return true;
}