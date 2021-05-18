# vi_grid_map: value iterative visualization package for ROS
visualization of global planner for mobile robots based on [value_iteration](https://github.com/ryuichiueda/value_iteration).

## Nodes

### ViGridMap_node
This node executes [value_iteration](https://github.com/ryuichiueda/value_iteration) visualization.

#### Topics
##### [vi_grid_map](https://github.com/uhobeike/vi_grid_map)
* vi_grid_cells ([vi_grid_map_msgs::ViGridCells](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/srv/GetGridMap.html))
    * It is the data to visualize on rviz based on the policy and value obtained from [value_iteration](https://github.com/ryuichiueda/value_iteration).

#### Services Called
##### [map_server](http://wiki.ros.org/map_server)
* static_map ([nav_msgs/GetMap](http://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
    * Initiate the map for localization.
##### [value_iteration](https://github.com/ryuichiueda/value_iteration)
* policy ([grid_map_msgs::GetGridMap](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/srv/GetGridMap.html))
    * Provide calculated policy. The id of the optimal action is written as a float value in each cell.
* value ([grid_map_msgs::GetGridMap](http://docs.ros.org/en/kinetic/api/grid_map_msgs/html/srv/GetGridMap.html))
    * Provide calculated value function.
## acknowledgement
This software is developped on the support of JSPS KAKENHI JP20K04382.
