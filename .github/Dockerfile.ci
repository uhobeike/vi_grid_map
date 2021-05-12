FROM osrf/ros:melodic-desktop-full

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get upgrade -y && \ 
    apt-get install -y \
    #ROS
    python-catkin-tools \
    python3-pip
RUN pip3 install rospkg catkin_pkg
    
RUN mkdir -p /home/catkin_ws/src \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /home/catkin_ws/src