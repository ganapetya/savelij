export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file:///home/bots/cyclonedds.xml
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source /home/bots/all/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 run rviz2 rviz2 -d /home/bots/all/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_rviz/rviz/mapping.rviz
