# yahboom_ws_peter

Mirror of the ROS 2 packages in the Yahboom robot workspace
`~/yahboomcar_ros2_ws/yahboomcar_ws/src`, including `lk_body`,
`cat_patrol_robot`, and `odom_drift_checker`.

The robot is the source of the files. This repo only receives pushes.

## Layout

    yahboom_ws_peter/
    └── src/
        ├── yahboomcar_bringup/        ← Yahboom vendor (with our
        │                                 trim-parameter modifications;
        │                                 see cat_patrol_robot/yahboom_overlay
        │                                 for the patch record)
        ├── yahboomcar_base_node/      ← Yahboom vendor (C++ odometry integrator)
        ├── yahboomcar_description/    ← Yahboom vendor (URDFs and meshes)
        ├── yahboomcar_msgs/           ← Yahboom vendor (message types)
        └── ...                        ← all other yahboomcar_* packages
                                          plus laserscan_to_point_pulisher,
                                          robot_pose_publisher_ros2, etc.

## Sync workflow

On this machine, from the repo root:

    ./sync_from_robot.sh

That copies `jetson@192.168.0.120:~/yahboomcar_ros2_ws/yahboomcar_ws/src/`
into `src/` here, commits whatever changed, and pushes branch `master` to
`git@github.com:ganapetya/savelij.git`.

It does not fetch or pull. A non-fast-forward push stops without changing
GitHub. Optional message: `./sync_from_robot.sh "why this sync"`.

`build/`, `install/`, and `log/` inside the robot `src` tree are colcon
output and are left behind. Files matched by `.gitignore` (large model
files, `src/lk_body/.env`) are copied onto this machine and are not committed.
