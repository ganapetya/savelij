# Savelij

<p align="left">
  <img src="src/images/savelij.jpg" width="280" alt="Savelij" />
</p>

Savelij is an indoor robot on a Yahboom ROSMASTER X3 mecanum chassis. A
Jetson Orin NX on the robot runs Ubuntu and ROS 2 Humble and does the
sensing, driving, and navigation. It carries a SLAMTEC RPLiDAR A1 and an
Orbbec Astra depth camera. The chassis board runs the motors, encoders, and
IMU. The robot patrols rooms on its own, looks for cats, and can bark at one
when it sees it.

This repository is the robot's ROS 2 workspace,
`~/yahboomcar_ros2_ws/yahboomcar_ws/src`, copied from the Jetson. The robot
is the source of those files. This repo only receives pushes.

## Layout

    savelij/
    └── src/
        ├── images/savelij.jpg
        ├── lk_body/
        ├── cat_patrol_robot/
        ├── odom_drift_checker/
        ├── yahboomcar_bringup/
        ├── yahboomcar_base_node/
        ├── yahboomcar_description/
        ├── yahboomcar_msgs/
        └── ...                    ← the other yahboomcar_* packages,
                                      laserscan_to_point_pulisher,
                                      robot_pose_publisher_ros2, and so on
