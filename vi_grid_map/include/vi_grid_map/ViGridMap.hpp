#ifndef VI_GRID_MAP_
#define VI_GRID_MAP_

#include <ros/ros.h>

#include "vi_grid_map_msgs/ViGridCells.h"

namespace vi_grid_map {

class ViGridMap
{
public:
    ViGridMap(ros::NodeHandle& nodeHandle);
    virtual ~ViGridMap();

    void vi_grid_map_init();
    void msg_set();

    void grid_valueCb(const vi_grid_map_msgs::ViGridCells& grid_value);
    void publish(vi_grid_map_msgs::ViGridCells& msg);

private:
    ros::NodeHandle& _nh;

    ros::Subscriber _vi_valueSubscriber;
    ros::Publisher _vi_grid_mapPublisher;

    float _width;
    float _length;
    float _resolution;
    float _resolution_Reciprocal;
    int _loop_index;
    int _loop_width;
    int _loop_length;

};

} /* namespace */
#endif