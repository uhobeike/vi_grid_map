#include <ros/ros.h>

#include <nav_msgs/GetMap.h>

#include "vi_grid_map/ViGridMap.hpp"

using namespace::std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vi_grid_map");
	ros::NodeHandle nh;

	while (!ros::service::waitForService("/static_map", ros::Duration(5.0))){
		ROS_ERROR("Service not found /static_map");
		return 1;
	}

	ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("/static_map");

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response resp;
	if (not client.call(req, resp)){
		ROS_ERROR("static_map not working");
		return 1;
	}
	ros::service::call("static_map", req, resp);

	vi_grid_map::ViGridMap vi(nh, resp.map);

	ros::spin();
	return 0;
}