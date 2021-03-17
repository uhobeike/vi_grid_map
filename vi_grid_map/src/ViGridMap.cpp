#include <ros/ros.h>

#include "vi_grid_map/ViGridMap.hpp"
#include "vi_grid_map_msgs/ViGridCells.h"

#include <math.h>

using namespace::std;

namespace vi_grid_map {

ViGridMap::ViGridMap(ros::NodeHandle& nodeHandle, nav_msgs::OccupancyGrid& map) :
                    _nh(nodeHandle),
                    _map(map),
                    _width(0.0), _length(0.0),
                    _resolution(0.0),
                    _resolution_Reciprocal(0.0),
                    _loop_index(0), _loop_width(0), _loop_length(0), _loop_width_length(0),
                    _vi_grid_cells_value_min(0)
{
    _vi_valueSubscriber   = _nh.subscribe("grid_value", 1, &ViGridMap::grid_valueCb, this);
    _vi_grid_mapPublisher = _nh.advertise<vi_grid_map_msgs::ViGridCells>("grid_cells", 100);

    vi_grid_map_init();
}

ViGridMap::~ViGridMap() {}

void ViGridMap::grid_valueCb(const vi_grid_map_msgs::ViGridCells& grid_value)
{
    vi_grid_map_msgs::ViGridCells msg;

    msg = grid_value;
    msg.cells.resize(int(_width * _length* pow(1/_resolution, 2)));
    msg.cell_width = _resolution;
    msg.cell_height = _resolution;

    for (int x = 0; x < _loop_width; ++x){
        for (int y = 0; y < _loop_length; ++y){
            geometry_msgs::Point& point = msg.cells[x + ( y * _loop_index)];
            point.x = float(x) * _resolution;
            point.y = float(y) * _resolution;
            point.z = 0;
        }
    }

    search_vi_grid_cells_value_min(msg);
    publish(msg);
}

void ViGridMap::vi_grid_map_init()
{
    _width = _map.info.width * _map.info.resolution;
    _length = _map.info.height * _map.info.resolution;
    _resolution = _map.info.resolution;
    _resolution_Reciprocal = 1/_resolution;
    _loop_index = max(_width, _length) * _resolution_Reciprocal;
    _loop_width = _width * _resolution_Reciprocal;
    _loop_length = _length * _resolution_Reciprocal;
    _loop_width_length = _loop_width * _loop_length;
}

void ViGridMap::search_vi_grid_cells_value_min(vi_grid_map_msgs::ViGridCells& msg)
{
    bool init_search_value_flag = false;
    for (int i = 0; i < _loop_width_length-1; ++i){
        if(msg.cell_value[i] != 0){
            if(!init_search_value_flag){
                _vi_grid_cells_value_min = msg.cell_value[i];
                init_search_value_flag = true;
            }
            _vi_grid_cells_value_min = min(msg.cell_value[i], _vi_grid_cells_value_min);
        }
    }

    msg.cell_value[1] = _vi_grid_cells_value_min;
}

void ViGridMap::publish(vi_grid_map_msgs::ViGridCells& msg)
{
    msg.header.frame_id = "vi_map";
    msg.header.stamp = ros::Time::now();
    _vi_grid_mapPublisher.publish(msg);
}

} /* namespace */