#include <ros/ros.h>

#include "vi_grid_map/ViGridMap.hpp"

using namespace::std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vi_grid_map");
    ros::NodeHandle nodeHandle;
    vi_grid_map::ViGridMap vi(nodeHandle);
    
    ros::spin();
    return 0;
}
