FROM ros:noetic-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    wmctrl \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
# Upgrade pip first to ensure we get new wheels
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir --upgrade \
    numpy \
    utm \
    matplotlib \
    rospkg

# Setup catkin workspace
WORKDIR /root/catkin_ws/src
# Copy specifically the source code folders
COPY src/ /root/catkin_ws/src/AGC-MOTE-Monte-Carlo/src/

# Force create a valid Top-Level CMakeLists.txt if missing, by symlinking the standard catkin one
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"

# Build the workspace
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Add setup.bash to .bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set the entrypoint
CMD ["/bin/bash", "-c", "source /root/catkin_ws/devel/setup.bash && roslaunch mote_ros run_both_nodes.launch"]
