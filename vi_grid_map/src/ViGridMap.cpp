#include <ros/ros.h>

#include "vi_grid_map/ViGridMap.hpp"
#include "vi_grid_map_msgs/ViGridCells.h"
#include "math.h"

using namespace::std;

namespace vi_grid_map {

ViGridMap::ViGridMap(ros::NodeHandle& nodeHandle) :
                    _nh(nodeHandle),
                    _width(19.2), _length(19.2),
                    _resolution(0.06),
                    _resolution_Reciprocal(0.0),
                    _loop_index(0), _loop_width(0), _loop_length(0)
{
    _vi_valueSubscriber   = _nh.subscribe("grid_value", 1, &ViGridMap::grid_valueCb, this);
    _vi_grid_mapPublisher = _nh.advertise<vi_grid_map_msgs::ViGridCells>("grid_cells", 100);

    vi_grid_map_init();
    msg_set();
}

ViGridMap::~ViGridMap() {}

void ViGridMap::grid_valueCb(const vi_grid_map_msgs::ViGridCells& grid_value)
{
    vi_grid_map_msgs::ViGridCells msg;

    msg = grid_value;

    for (int x = 0; x < _loop_width; ++x)
    {
      for (int y = 0; y < _loop_length; ++y)
      {
        geometry_msgs::Point& point = msg.cells[x + ( y * _loop_index)];
        point.x = float(x) * _resolution;
        point.y = float(y) * _resolution;
        point.z = 0;
      }
    }

    publish(msg);
}

void ViGridMap::vi_grid_map_init()
{
    _resolution_Reciprocal = 1/_resolution;
    _loop_index = max(_width, _length) * _resolution_Reciprocal;
    _loop_width = _width * _resolution_Reciprocal;
    _loop_length = _length * _resolution_Reciprocal;
}

void ViGridMap::msg_set()
{
    vi_grid_map_msgs::ViGridCells msg;
    
    msg.cells.resize(int(_width * _length* pow(1/_resolution, 2)));
    msg.header.frame_id = "vi_map";
    msg.header.stamp = ros::Time::now();
    msg.cell_width = _resolution;
    msg.cell_height = _resolution;

    publish(msg);
}

void ViGridMap::publish(vi_grid_map_msgs::ViGridCells& msg)
{
    msg.header.frame_id = "vi_map";
    msg.header.stamp = ros::Time::now();
    _vi_grid_mapPublisher.publish(msg);
}

} /* namespace */