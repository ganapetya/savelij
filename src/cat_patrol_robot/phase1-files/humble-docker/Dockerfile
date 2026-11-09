FROM ros:humble

# GUI + ROS tools used on host peter-pen (Jazzy host, Humble container)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-slam-toolbox \
    ros-humble-rviz2 \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-graph \
    ros-humble-rqt-topic \
    ros-humble-rqt-console \
    ros-humble-rqt-reconfigure \
    ros-humble-rqt-image-view \
    libx11-xcb1 \
    libxcb1 \
    libxcb-cursor0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    libgl1 \
    libglx-mesa0 \
    libegl1 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Defaults; compose can override. Cyclone config is mounted at /cfg/cyclonedds.xml
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=28 \
    ROS_LOCALHOST_ONLY=0 \
    CYCLONEDDS_URI=file:///cfg/cyclonedds.xml \
    QT_QPA_PLATFORM=xcb \
    QT_X11_NO_MITSHM=1

COPY start.sh /usr/local/bin/start.sh
RUN chmod +x /usr/local/bin/start.sh

# Interactive shell by default; use `start.sh` or compose command for rqt_graph
CMD ["bash"]
