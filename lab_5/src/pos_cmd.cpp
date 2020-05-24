#include <ros/ros.h>
#include <lab_5/oint_srv.h>

using namespace std;

int main(int argc, char** argv) {
	ros::init(argc, argv, "POS_CMD");
	
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<lab_5::oint_srv>("oint_srv");
	client.waitForExistence();
	lab_5::oint_srv srv;
	
	vector<vector<double>> v_rect = {{1.5, 0.5, -0.25, 0, 0, 0},
									  {1.5, 0.5, 0, 0, 0, 0},
									  {2, 0, 0, 0, 0, 0},
									  {2, 0, -0.25, 0, 0, 0}};
	vector<double> v_rect_times = {50, 25, 50, 25};
	for(int j = 0; j < 2; ++j)
		for(int i = 0; i < 4; ++i){
			srv.request.desired_position = v_rect[i];
			srv.request.mov_duration = v_rect_times[i];
			if(client.call(srv))
				ROS_INFO("Interpolation successful - movement no. %d", i+1);
			else
				ROS_INFO("Service failed");
		}
	
	for(double alpha = 0; alpha <= 4*M_PI; alpha += 0.1) {
		srv.request.desired_position = {1.75 + 0.25*cos(alpha), 0.25*sin(alpha), -0.25, 0, 0, 0};
		srv.request.mov_duration = 1;
		if(!client.call(srv))
			ROS_ERROR("[ICMD] Circle failed");
	}
	
	return 0;
}