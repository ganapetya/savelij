export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
export CYCLONEDDS_URI=file:///home/bots/cyclonedds.xml
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/bots/slam_params/mapper_params_host.yaml \
  use_sim_time:=false
