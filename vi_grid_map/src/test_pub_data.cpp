#include <ros/ros.h>

#include <vi_grid_map_msgs/ViGridCells.h>

#include <fstream>
#include <vector>

using namespace::std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_data");
    ros::NodeHandle nh;
    ros::Publisher pub_data;

    pub_data = nh.advertise<vi_grid_map_msgs::ViGridCells>("grid_value", 1, true);

    string fname(argv[1]);
    string line;
    vector<float> file_read;

    ifstream f_r(fname.c_str(), std::ios::in);
    if (f_r.fail()){
        ROS_ERROR("csv_file could not open %s.", fname.c_str());
        exit(-1);
    }

    while (getline(f_r, line)){
        istringstream stream(line);
        file_read.push_back(stof(line));
    }
    vi_grid_map_msgs::ViGridCells vi_grid_cells;
    vi_grid_cells.cell_value = file_read;
    
    ros::Rate loop_rate(1);
    while (ros::ok){
        pub_data.publish(vi_grid_cells);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}