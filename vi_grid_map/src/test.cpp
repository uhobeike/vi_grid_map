#include "ros/ros.h"
#include "vi_grid_map_msgs/ViGridCells.h"
#include "math.h"

using namespace::std;

int main(int argc, char** argv){
  ros::init(argc, argv, "send_grid_cells");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<vi_grid_map_msgs::ViGridCells>("grid_cells", 100);
  // ros::Rate loop_rate(0.1);

  float width = 19.2;
  float length = 19.2;
  float resolution = 0.06;
  float resolution_Reciprocal = 1/resolution;
  int loop_index = max(width, length) * resolution_Reciprocal;
  int loop_width = width * resolution_Reciprocal;
  int loop_length = length * resolution_Reciprocal;

  vi_grid_map_msgs::ViGridCells msg;
  msg.cells.resize(int(width * length* pow(1/resolution,2)));
  msg.header.frame_id = "vi_map";
  msg.cell_width = resolution;
  msg.cell_height = resolution;

  for (int x = 0; x < loop_width; x++)
    for (int y = 0; y < loop_length; y++)
      //Processing something....

  ros::ok;  
  ros::Duration duration(2);
  duration.sleep();
  
  
  while (ros::ok())
  {
    for (int x = 0; x < loop_width; ++x)
    {
      for (int y = 0; y < loop_length; ++y)
      {
        geometry_msgs::Point& point = msg.cells[x + ( y*loop_index)];
        point.x = float(x)*resolution;
        point.y = float(y)*resolution;
        point.z = 0;
      }
      msg.header.stamp = ros::Time::now();
      pub.publish(msg);

      ros::Duration duration(0.01);
      duration.sleep();
    }
    // ros::spinOnce();
    // loop_rate.sleep();
  }
}
