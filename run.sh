#!/bin/bash
xhost +local:docker
cd ..

docker run -it --rm --gpus all \
    -v $(pwd):/home/catkin_ws/src \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_INDIRECT=1 \
    --privileged \
    --net=host \
ros-ci_vi_grid_map /bin/bash -c \
    "git clone -b ikebe_dev https://github.com/ryuichiueda/value_iteration.git;
    rosdep install -r -y --from-paths --ignore-src .; 
    cd /home/catkin_ws && catkin build && source /home/catkin_ws/devel/setup.bash; 
    roslaunch value_iteration vi_map.launch; 
    /bin/bash"

sudo chown -R $USER value_iteration
cd -
xhost -local:docker