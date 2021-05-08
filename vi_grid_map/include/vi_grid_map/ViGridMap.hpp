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

    void ViGridMap_Init(void);
    void CellPlacementCalculation(vi_grid_map_msgs::ViGridCells& msg);
    void ViGrid_ValueMinSearch(vi_grid_map_msgs::ViGridCells& msg);

    void ViGrid_ValueCb(const vi_grid_map_msgs::ViGridCells& grid_value);
    void Publish(vi_grid_map_msgs::ViGridCells& msg);

    void ExecuteCb(const vi_grid_map_msgs::ViGridMapGoalConstPtr& goal);
    void ViGrid_ValueStockUp(vi_grid_map_msgs::ViGridCells& msg);
    bool ViGrid_ValueStockUpCheck(void);
    void ChangeThetaViGridMap(const vi_grid_map_msgs::ViGridMapGoalConstPtr& goal);

private:
    ros::NodeHandle& nh_;

    ros::Subscriber vi_valueSubscriber_;
    ros::Publisher vi_grid_mapPublisher_;
    actionlib::SimpleActionServer<vi_grid_map_msgs::ViGridMapAction> as_;
    vi_grid_map_msgs::ViGridMapFeedback feedback_;
    vi_grid_map_msgs::ViGridMapResult result_; 
    std::string action_name_;
    bool success_;

    nav_msgs::OccupancyGrid map_;

    float width_;
    float length_;
    float resolution_;
    float resolution_Reciprocal_;
    int loop_index_;
    int loop_width_;
    int loop_length_;
    int loop_width_length_;
    float vi_grid_cells_value_min_;
    uint8_t vi_cell_theta_total_num_;
    
    vector <vector<vi_grid_map_msgs::ViGridCells>> vi_value_store_;
    vector <vector<vi_grid_map_msgs::ViGridCells>> vi_action_store_;
    uint8_t vi_value_index_;
    uint8_t vi_action_index_;

};

} /* namespace */
#endif