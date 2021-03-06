#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include "math.h"

using namespace::std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_grid_cells");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::GridCells>("grid_cells", 100);
  ros::Rate loop_rate(5);

  nav_msgs::GridCells msg;
  int width = 19.2;
  int length = 19.2;
  msg.cells.resize(width * length*400);
  msg.header.frame_id = "map";
  msg.cell_width = 0.05;
  msg.cell_height = 0.05;

  while (ros::ok())
  {
    for (int x = 0; x < width*20; x++)
    {
      // cout << "1" << "\n";
      for (int y = 0; y < length*20; y++)
      {
        // cout << "1" << "\n";
        geometry_msgs::Point& point = msg.cells[x + ((y*20) * width)];
        // cout << "2" << "\n";
        // cout << x << " " << y << "\n";
        point.x = float(x)/20;
        point.y = float(y)/20;
        // point.x = x;
        // point.y = y;
        point.z = 0;
      }
    }

    msg.header.stamp = ros::Time::now();

    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
