#include <ros/ros.h>

#include "vi_grid_map/ViGridMap.hpp"

using namespace::std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vi_grid_map");
    ros::NodeHandle nh;
    vi_grid_map::ViGridMap vi(nh);
    
    ros::spin();
    return 0;
}
