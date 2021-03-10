#include <ros/ros.h>

#include "vi_grid_map/ViGridMap.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map_interpolation_demo");
    ros::NodeHandle nodeHandle("~");
    vi_grid_map::ViGridMap vi(nodeHandle);
  
    ros::spin();
    return 0;
}
