#an array of cells in a 2D vi_grid
Header header
float32 cell_width
float32 cell_height
geometry_msgs/Point[] cells
uint8 cell_theta_total_num
float32[] cell_value
#About the packet contents of cell_value
#[0]  theta_num
#[1]  cell_value_min
#[2]  cell_value_max
#[3~] cell_value