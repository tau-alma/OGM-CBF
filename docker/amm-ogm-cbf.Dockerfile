FROM ros:humble-ros-base-jammy

# avoid interactive configuration dialogs
ENV DEBIAN_FRONTEND=noninteractive
ENV CXX=clang++
#ENV NVIDIA_DISABLE_REQUIRE=true 
SHELL ["/bin/bash","-c"]
WORKDIR /home/ogm-cbf
RUN mkdir /data
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN apt-get update && apt-get -y upgrade && rm -rf /var/lib/apt/lists/*

# reqs
RUN apt-get update && apt-get -y install clang udev ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-image-proc ros-humble-depth-image-proc ros-humble-imu-pipeline ros-humble-tf-transformations ros-humble-odom-to-tf-ros2 python3-scipy \
   && rm -rf /var/lib/apt/lists/*
# ros generic usefull tools
RUN apt-get update && apt-get -y install git tree neofetch guvcview libyaml-cpp-dev libboost-all-dev zlib1g-dev libeigen3-dev linux-libc-dev nlohmann-json3-dev ros-humble-rviz2 ros-humble-rqt-tf-tree ros-humble-rqt-image-view ros-humble-rqt-bag ros-humble-rqt-graph vim \
   && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y install libsdl2-dev freeglut3-dev ros-humble-pcl-conversions ros-humble-pcl-ros \
   && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ws/src
RUN echo "source /home/ogm-cbf/ws/install/setup.bash" >> /root/.bashrc

COPY ogm ogm
RUN mkdir -p ogm/gridmap/build
RUN cd ogm/gridmap/build && cmake -DLARGE_DATASET=OFF .. && make install

COPY ros2_ws/src/ogm_cbf_kinematic_sim ogm_cbf_kinematic_sim

RUN ln -s /home/ogm-cbf/ogm/ros_ws/src /home/ogm-cbf/ws/src/ogm
RUN ln -s /home/ogm-cbf/ogm_cbf_kinematic_sim /home/ogm-cbf/ws/src/ogm_cbf_kinematic_sim
RUN source /opt/ros/humble/setup.bash && cd ws \
  && rosdep update && apt-get update \
  && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y \
   && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/humble/setup.bash && cd ws && colcon build

