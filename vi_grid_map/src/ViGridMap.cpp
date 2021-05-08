#include <ros/ros.h>

#include "vi_grid_map/ViGridMap.hpp"
#include "vi_grid_map_msgs/ViGridCells.h"

#include <math.h>

using namespace::std;

namespace vi_grid_map {

ViGridMap::ViGridMap(ros::NodeHandle& nodeHandle, std::string name, nav_msgs::OccupancyGrid& map) :
                    nh_(nodeHandle),
                    as_(nodeHandle, name, boost::bind(&ViGridMap::ExecuteCb, this, _1), false),
                    action_name_(name), success_(true),
                    map_(map),
                    width_(0.0), length_(0.0),
                    resolution_(0.0),
                    resolution_Reciprocal_(0.0),
                    loop_index_(0), loop_width_(0), loop_length_(0), loop_width_length_(0),
                    vi_grid_cells_value_min_(0), vi_cell_theta_total_num_(0)
{
    vi_valueSubscriber_   = nh_.subscribe("grid_value", 1, &ViGridMap::ViGrid_ValueCb, this);
    vi_grid_mapPublisher_ = nh_.advertise<vi_grid_map_msgs::ViGridCells>("grid_cells", 100);

    ViGridMap_Init();
    as_.start();
}

ViGridMap::~ViGridMap() {}

void ViGridMap::ViGrid_ValueCb(const vi_grid_map_msgs::ViGridCells& grid_value)
{
    vi_grid_map_msgs::ViGridCells msg;
    msg = grid_value;

    vi_cell_theta_total_num_ = msg.cell_theta_total_num;
    
    ViGrid_ValueStockUp(msg);

    CellPlacementCalculation(msg);
    ViGrid_ValueMinSearch(msg);
    
    Publish(msg);
}

void ViGridMap::ViGridMap_Init()
{
    width_ = map_.info.width * map_.info.resolution;
    length_ = map_.info.height * map_.info.resolution;
    resolution_ = map_.info.resolution;
    resolution_Reciprocal_ = 1/resolution_;
    loop_index_ = max(width_, length_) * resolution_Reciprocal_;
    loop_width_ = width_ * resolution_Reciprocal_;
    loop_length_ = length_ * resolution_Reciprocal_;
    loop_width_length_ = loop_width_ * loop_length_;
}

void ViGridMap::CellPlacementCalculation(vi_grid_map_msgs::ViGridCells& msg)
{
    msg.cells.resize(int(width_ * length_ * pow(1/resolution_, 2)));
    msg.cell_width = resolution_;
    msg.cell_height = resolution_;

    for (int x(0); x < loop_width_; ++x){
        for (int y(0); y < loop_length_; ++y){
            geometry_msgs::Point& point = msg.cells[x + ( y * loop_index_)];
            point.x = float(x) * resolution_;
            point.y = float(y) * resolution_;
            point.z = 0;
        }
    }
}

void ViGridMap::ViGrid_ValueMinSearch(vi_grid_map_msgs::ViGridCells& msg)
{
    bool init_search_value_flag = false;
    for (int i(3); i < loop_width_length_-1; ++i){
        if (msg.cell_value[i] != 0){
            if (!init_search_value_flag){
                vi_grid_cells_value_min_ = msg.cell_value[i];
                init_search_value_flag = true;
            }
            vi_grid_cells_value_min_ = min(msg.cell_value[i], vi_grid_cells_value_min_);
        }
    }

    msg.cell_value[1] = vi_grid_cells_value_min_;
}

void ViGridMap::Publish(vi_grid_map_msgs::ViGridCells& msg)
{
    msg.header.frame_id = "vi_map";
    msg.header.stamp = ros::Time::now();
    vi_grid_mapPublisher_.publish(msg);
}

void ViGridMap::ViGrid_ValueStockUp(vi_grid_map_msgs::ViGridCells& msg)
{
    vi_value_store_.resize(msg.cell_theta_total_num);

    for (uint8_t i(0); i <= msg.cell_theta_total_num; ++i){
        if (i == msg.cell_value[0]){
            vi_value_store_[i].clear();
            vi_value_store_[i].push_back(msg);
        }
    }
}

bool ViGridMap::ViGrid_ValueStockUpCheck(void)
{
    int cnt_index(0);

    for (uint8_t i(0); i <= vi_cell_theta_total_num_; ++i){
        if (vi_value_store_[i].size() == 1)
            ++cnt_index;
    }

    if (cnt_index == vi_cell_theta_total_num_)
        return true;
    else 
        return false;
}

void ViGridMap::ChangeThetaViGridMap(const vi_grid_map_msgs::ViGridMapGoalConstPtr &goal)
{
    vi_grid_map_msgs::ViGridCells msg;
    msg = vi_value_store_[goal->vi_value_theta_num_set].front();
    CellPlacementCalculation(msg);
    ViGrid_ValueMinSearch(msg);
    
    Publish(msg);
}

void ViGridMap::ExecuteCb(const vi_grid_map_msgs::ViGridMapGoalConstPtr &goal)
{    
    feedback_.vi_value_theta_num_status.clear();
    feedback_.vi_action_theta_num_status.clear();
    feedback_.vi_value_stock_up_check_status.clear();

    feedback_.vi_value_theta_num_status = "Settinng value_theta_num.....";
    feedback_.vi_action_theta_num_status = "Setting action_theta_num.....";
    
    ROS_INFO("%s: PROCESSING\ngoal:\n       vi_value_theta_num: %i, vi_action_theta_num: %i\nstatus:\n      vi_value_theta_num_status: %s, vi_action_theta_num_status: %s"
            ,action_name_.c_str(), goal->vi_value_theta_num_set, goal->vi_action_theta_num_set,
            feedback_.vi_value_theta_num_status.c_str(), feedback_.vi_action_theta_num_status.c_str());

    ros::Rate loop_rate(1);
    while (feedback_.vi_value_stock_up_check_status != "true"){
        if (as_.isPreemptRequested() || !ros::ok()){
            ROS_WARN("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success_ = false;
            break;
        }
        
        feedback_.vi_value_stock_up_check_status = ViGrid_ValueStockUpCheck() ? "true" : "false";
        feedback_.vi_value_store_max_index = vi_cell_theta_total_num_;
        as_.publishFeedback(feedback_);

        if (feedback_.vi_value_stock_up_check_status != "true")
            ROS_WARN("vi_value_stock_up_check(): false");
        
        loop_rate.sleep();
    }
    
    if (success_){
        ChangeThetaViGridMap(goal);
        feedback_.vi_value_theta_num_status = "Set value_theta_num";
        feedback_.vi_action_theta_num_status = "Set action_theta_num";
        as_.publishFeedback(feedback_);
        
        ROS_INFO("\nstatus:\n      vi_value_theta_num_status: %s, vi_action_theta_num_status: %s"
                ,feedback_.vi_value_theta_num_status.c_str(), feedback_.vi_action_theta_num_status.c_str());

        result_.vi_value_theta_num_current = goal->vi_value_theta_num_set;
        result_.vi_action_theta_num_current = goal->vi_action_theta_num_set;

        ROS_INFO("%s: SUCCEEDED", action_name_.c_str());
        as_.setSucceeded(result_);
    }
    else {
        feedback_.vi_value_theta_num_status = "Can't Set value_theta_num";
        feedback_.vi_action_theta_num_status = "Cant't Set action_theta_num";
        as_.publishFeedback(feedback_);

        ROS_ERROR("%s: FAILURE", action_name_.c_str());
        as_.setAborted(result_);
    }
}

} /* namespace */