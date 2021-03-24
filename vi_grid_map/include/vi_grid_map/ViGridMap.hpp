#ifndef VI_GRID_MAP_
#define VI_GRID_MAP_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <nav_msgs/OccupancyGrid.h>

#include <vi_grid_map_msgs/ViGridCells.h>
#include <vi_grid_map_msgs/ViGridMapAction.h>

using namespace::std;

namespace vi_grid_map {

class ViGridMap
{
public:
    ViGridMap(ros::NodeHandle& nodeHandle,std::string name, nav_msgs::OccupancyGrid& map);
    virtual ~ViGridMap();

    void vi_grid_map_init();
    void CellPlacementCalculation(vi_grid_map_msgs::ViGridCells& msg);
    void search_vi_grid_cells_value_min(vi_grid_map_msgs::ViGridCells& msg);

    void grid_valueCb(const vi_grid_map_msgs::ViGridCells& grid_value);
    void publish(vi_grid_map_msgs::ViGridCells& msg);
    void vi_value_stock_up(vi_grid_map_msgs::ViGridCells& msg);
    bool vi_value_stock_up_check();
    void ChangeThetaViMap(const vi_grid_map_msgs::ViGridMapGoalConstPtr &goal);

    void executeCb(const vi_grid_map_msgs::ViGridMapGoalConstPtr &goal);

private:
    ros::NodeHandle& _nh;

    ros::Subscriber _vi_valueSubscriber;
    ros::Publisher _vi_grid_mapPublisher;
    actionlib::SimpleActionServer<vi_grid_map_msgs::ViGridMapAction> _as; 
    std::string _action_name;
    bool _success;

    nav_msgs::OccupancyGrid _map;

    float _width;
    float _length;
    float _resolution;
    float _resolution_Reciprocal;
    int _loop_index;
    int _loop_width;
    int _loop_length;
    int _loop_width_length;
    float _vi_grid_cells_value_min;
    uint8_t _vi_cell_theta_total_num;
    
    vector <vector<vi_grid_map_msgs::ViGridCells>> _vi_value_store;
    vector <vector<vi_grid_map_msgs::ViGridCells>> _vi_action_store;
    uint8_t _vi_value_index;
    uint8_t _vi_action_index;

};

} /* namespace */
#endif