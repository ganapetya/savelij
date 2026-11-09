# Phase 3 — Autonomous Navigation (Nav2) Status & Reference

Companion to [`plan.md`](plan.md), [`phase2-status.md`](phase2-status.md),
[`phase1-status.md`](phase1-status.md), [`hardware.md`](hardware.md), and
[`architecture.md`](architecture.md).

Phase 3 = put the full **Nav2** stack on top of Phase 2's localization. The
robot already knows the room (Phase 1 map) and where it is in the room
(Phase 2 AMCL). Now it must **plan a path to a goal and drive there itself**,
avoiding obstacles, and stop at the goal pose.

> **STATUS: Not yet started.** This document is the pre-session plan.
> Fill in the session log (§11) as you run sessions. The §13 learning guide is
> written to be read *before* the first session — it is the study reference for
> the Nav2 architecture, the action-client C++ goal, and the RViz views.

---

## 1. Purpose and scope

**Goal.** Bring up the Nav2 navigation half — `planner_server`,
`controller_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`,
global + local costmaps, and a second `lifecycle_manager` — on the Jetson,
layered on top of Phase 2's `map_server` + `amcl`. Then:

- **Milestone 1.** Click a "Nav2 Goal" in RViz → the robot plans a path,
  drives there avoiding obstacles, and stops near the goal pose.
- **Milestone 2 (the C++ learning meat).** Write a standalone C++ node,
  `nav_goal_client_node`, that programmatically sends a `NavigateToPose`
  action goal, prints live feedback (distance remaining), and prints the final
  result (`succeeded` / `aborted` / `canceled`).

**Why this phase matters (from plan.md).**
- This is where the **action-client architecture** becomes real.
  `NavigateToPose` is a ROS 2 action with *goals, feedback, and results*. The
  C++ client exercises goal handles, feedback callbacks, and
  `std::shared_future` — the conceptual rhyme with `std::future` /
  `std::packaged_task` from *C++ Concurrency in Action* is exactly the point.
- It is the single most valuable phase for C++ / ROS 2 fluency.
- The Phase 3 action client is reused **directly** in Phase 4 (patrol manager
  drives between waypoints with it) and Phase 6 (goal *cancellation* on a cat
  sighting).

**What changes vs Phase 2.**
- Phase 2's `localization.launch.py` (`map_server` + `amcl` +
  `lifecycle_manager_localization`) is **kept unchanged** and runs alongside.
- Phase 3 adds a **second** launch file, `navigation.launch.py`, with its own
  `lifecycle_manager_navigation` managing the navigation nodes. Two lifecycle
  managers is the standard Nav2 split (localization vs navigation).
- The robot now publishes `/cmd_vel` **itself** (from `controller_server`) —
  the joystick is for safety override only.

**Scope.** One room (`living_room_v1`), single goals via RViz and via the C++
client. No patrol loop yet (Phase 4), no cat reaction (Phase 6). DWB local
planner, NavFn global planner — the same plugins the stock Yahboom config uses,
adapted to this robot.

---

## 2. Acceptance criteria

Phase 3 is done when BOTH milestones pass:

**Milestone 1 — RViz goals**
- [ ] All navigation nodes reach **Active** (lifecycle) with no `WARN`/`ERROR`
      about missing TF, missing costmap, or missing `/scan_filtered`.
- [ ] `/global_costmap/costmap` and `/local_costmap/costmap` both appear in
      RViz with the static map + inflation visible.
- [ ] Clicking **"Nav2 Goal"** in RViz produces a green `/plan` path from the
      robot to the goal that routes *around* walls (not through them).
- [ ] The robot drives the path and **stops within ~25 cm** of the goal pose
      (the `xy_goal_tolerance`), then settles (no oscillation).
- [ ] AMCL stays converged during the whole drive (`/amcl_pose` tracks the
      robot; particle cloud stays tight).
- [ ] A goal set behind a (real) obstacle makes Nav2 plan around it or run a
      recovery behavior (spin / backup / wait) — not crash into it.

**Milestone 2 — C++ action client**
- [ ] `nav_goal_client_node` builds cleanly (`colcon build`).
- [ ] Running it sends a hard-coded `NavigateToPose` goal; the terminal prints:
      goal accepted → periodic feedback (distance remaining) → final result.
- [ ] It prints **"goal succeeded"** when the robot reaches the pose,
      **"goal aborted"** if Nav2 gives up, and **"goal canceled"** if you wire
      a cancel (Ctrl-C handler or timed cancel) — all three paths exercised.
- [ ] The node does **not** block the executor while waiting on the result
      (uses the future / callback pattern, not a busy-wait).

Stretch (nice-to-have):
- Tune DWB so the robot does not creep — a brisk-but-safe approach.
- `NavigateThroughPoses` instead of single `NavigateToPose` (preview of Phase 4
  waypoint chaining).
- Save a `navigation.rviz` config with all Phase 3 displays preloaded.

---

## 3. What we need to create

| File | Purpose |
|---|---|
| `config/nav2_params.yaml` | Planner + controller + costmaps + behavior + bt_navigator + lifecycle_manager_navigation, tuned for this robot |
| `launch/navigation.launch.py` | Brings up the Nav2 navigation half in one command |
| `src/nav_goal_client_node.cpp` | **Milestone 2** — the C++ `NavigateToPose` action client |
| `~/myscripts2/t4_nav.sh` | Convenience script (domain-28 + cyclonedds env, mirrors t1/t2/t2.5/t3) |
| `CMakeLists.txt` + `package.xml` edits | Wire the new executable + add `nav2_msgs`, `rclcpp_action` deps |

Phase 2's `config/amcl_params.yaml` and `launch/localization.launch.py` are
**reused unchanged**. The `config/` and `launch/` install rules already exist in
CMakeLists; only the new executable needs CMakeLists changes.

---

## 4. Files to create (do this before the first session)

### 4a. `config/nav2_params.yaml`

Adapted from the stock Yahboom
[`yahboomcar_nav/params/dwa_nav_params.yaml`](../yahboomcar_nav/params/dwa_nav_params.yaml).
Key differences from the stock file, and **why**, for this robot:

| Param | Stock value | Our value | Reason |
|---|---|---|---|
| costmap `scan` topic | `/scan` | `/scan_filtered` | Only the trusted forward arc is reliable on this robot (Phase 1 finding — rear hits the antenna mast). The robot is intentionally "blind" behind; acceptable for forward-driving patrol. |
| `max_vel_x` | `0.26` | `0.18` | Small home with cats; matches our cautious teleop speed. Slip is worse at speed (Phase 0). |
| `max_vel_y` | `0.0` | `0.0` (kept) | X3 *can* strafe, but mecanum rollers slip badly on the smooth-floor area (Phase 0 #19). Keep strafe **off** so the controller commands only forward+yaw, where the wheels are honest. |
| `inflation_radius` | `0.11` | `0.25` | Stock is `robot_radius + 1 cm` — far too tight; Nav2 will graze walls. Start generous so the patrol robot keeps clear, then tighten if it refuses doorways (plan.md guidance). |
| `cost_scaling_factor` | `5.0` | `3.0` | Softer cost falloff → smoother paths that prefer the centre of free space. |
| `robot_radius` | `0.1` | `0.18` | The X3 footprint is ~36 cm wide; `0.1` under-represents it. Use a circular approximation big enough to cover the chassis. |
| obstacle `max_obstacle_height` | `2.0` | `2.0` (kept) | Lidar is planar; height gating is irrelevant but harmless. |

Create the file:

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/nav2_params.yaml << 'EOF'
# ============================================================================
# Nav2 NAVIGATION half for cat_patrol_robot (living_room_v1).
# Localization half (map_server + amcl) is in amcl_params.yaml / Phase 2.
# Adapted from yahboomcar_nav/params/dwa_nav_params.yaml for this robot:
#   - costmaps consume /scan_filtered (trusted front arc), NOT /scan
#   - strafe disabled (mecanum slips on smooth floor); forward+yaw only
#   - generous inflation + realistic footprint
# ============================================================================

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"
    # plugin_lib_names left at the Nav2 defaults (the full stock list works);
    # see dwa_nav_params.yaml if you need to pin the exact set.

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.18          # cautious cruise; mecanum slip worse at speed
      max_vel_y: 0.0           # strafe OFF — rollers slip on smooth floor
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.18
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 0.5
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -0.5
      vx_samples: 20
      vy_samples: 0
      vtheta_samples: 40
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.18
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan_filtered
          max_obstacle_height: 2.0
          obstacle_min_range: 0.1
          obstacle_max_range: 2.5
          raytrace_min_range: 0.0
          raytrace_max_range: 3.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.25
        cost_scaling_factor: 3.0
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      robot_radius: 0.18
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan_filtered
          obstacle_min_range: 0.1
          obstacle_max_range: 2.5
          raytrace_range: 3.0
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.25
        cost_scaling_factor: 3.0
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

behavior_server:
  ros__parameters:
    use_sim_time: false
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"
    global_frame: odom
    robot_base_frame: base_footprint
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

waypoint_follower:
  ros__parameters:
    use_sim_time: false
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200

lifecycle_manager_navigation:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names:
      - controller_server
      - planner_server
      - behavior_server
      - bt_navigator
      - waypoint_follower
EOF
```

> **Note on `min_y_velocity_threshold: 0.5`.** Carried over from stock; with
> `max_vel_y: 0.0` it is moot, but leaving it high guarantees the controller
> never tries to command tiny strafe velocities.

### 4b. `launch/navigation.launch.py`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/launch/navigation.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/config/nav2_params.yaml',
        description='Full path to the Nav2 navigation params YAML',
    )
    params = LaunchConfiguration('params_file')

    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[params],
    )
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[params],
    )
    behaviors = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen', parameters=[params],
    )
    bt_nav = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', parameters=[params],
    )
    waypoints = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[params],
    )
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', parameters=[params],
    )

    return LaunchDescription([
        params_file_arg,
        controller, planner, behaviors, bt_nav, waypoints, lifecycle,
    ])
EOF
```

### 4c. `src/nav_goal_client_node.cpp` — Milestone 2 (the C++ learning goal)

This is the centerpiece of Phase 3. Read every line of §13d before/while
studying it. It is heavily commented on purpose.

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/cat_patrol_robot/src/nav_goal_client_node.cpp << 'EOF'
// nav_goal_client_node.cpp
// A standalone NavigateToPose action client. Sends one hard-coded goal,
// prints feedback (distance remaining) and the final result.
//
// Learning goals (plan.md Phase 3):
//   - rclcpp_action::Client<NavigateToPose>
//   - goal handle (ClientGoalHandle::SharedPtr)
//   - goal-response / feedback / result callbacks
//   - waiting on a result without blocking the executor
//   - goal cancellation (async_cancel_goal) — needed in Phase 6

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

class NavGoalClient : public rclcpp::Node
{
public:
  NavGoalClient() : rclcpp::Node("nav_goal_client")
  {
    // Hard-coded goal pose in the map frame. Edit x/y/yaw to a real free
    // cell in living_room_v1 before running.
    this->declare_parameter("goal_x", 1.0);
    this->declare_parameter("goal_y", 0.0);
    this->declare_parameter("goal_yaw", 0.0);  // radians

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Defer the send until the executor is spinning, so the callbacks fire.
    timer_ = this->create_wall_timer(500ms, std::bind(&NavGoalClient::send_goal, this));
  }

private:
  void send_goal()
  {
    timer_->cancel();  // one-shot

    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
      rclcpp::shutdown();
      return;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = this->get_parameter("goal_x").as_double();
    goal.pose.pose.position.y = this->get_parameter("goal_y").as_double();
    const double yaw = this->get_parameter("goal_yaw").as_double();
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    RCLCPP_INFO(get_logger(), "Sending goal: x=%.2f y=%.2f yaw=%.2f",
                goal.pose.pose.position.x, goal.pose.pose.position.y, yaw);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
      [this](const GoalHandle::SharedPtr & h) {
        if (!h) {
          RCLCPP_ERROR(get_logger(), "Goal REJECTED by server");
          rclcpp::shutdown();
        } else {
          RCLCPP_INFO(get_logger(), "Goal ACCEPTED, navigating...");
        }
      };
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO(get_logger(), "  distance remaining: %.2f m", fb->distance_remaining);
      };
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "goal succeeded"); break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "goal aborted"); break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "goal canceled"); break;
          default:
            RCLCPP_ERROR(get_logger(), "unknown result code"); break;
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal, opts);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavGoalClient>());
  rclcpp::shutdown();
  return 0;
}
EOF
```

### 4d. CMakeLists.txt + package.xml edits

Add the action deps and the new executable.

**`package.xml`** — add after the existing `<depend>` lines:

```xml
  <depend>rclcpp_action</depend>
  <depend>nav2_msgs</depend>
```

**`CMakeLists.txt`** — add the find_package calls near the others:

```cmake
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
```

…and the executable + install (after the `patrol_node` block):

```cmake
add_executable(nav_goal_client_node src/nav_goal_client_node.cpp)
ament_target_dependencies(nav_goal_client_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs
)
install(TARGETS nav_goal_client_node DESTINATION lib/${PROJECT_NAME})
```

### 4e. `~/myscripts2/t4_nav.sh`

```bash
cat > ~/myscripts2/t4_nav.sh << 'EOF'
#!/usr/bin/env bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch cat_patrol_robot navigation.launch.py
EOF
chmod +x ~/myscripts2/t4_nav.sh
```

### 4f. Build

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select cat_patrol_robot
source install/setup.bash
# Verify the new executable exists:
ros2 pkg executables cat_patrol_robot | grep nav_goal_client_node
```

---

## 5. Architecture

```
Jetson (Humble)                                  Host peter-pen (Humble Docker)
──────────────────────────────────────────      ────────────────────────────────
t1.sh   yahboomcar_bringup                       humble-docker-rviz-1
        ├── base_node (MCU)                          rviz2
        ├── imu_filter                                  Map        /map  [TransientLocal]
        ├── ekf_filter  → /odom, odom→base              GlobalCostmap /global_costmap/costmap
        ├── robot_state_publisher                       LocalCostmap  /local_costmap/costmap
        └── yahboom_joy_X3 (safety override)            Path       /plan
                                                        PoseArray  /particle_cloud
t2.sh   sllidar_ros2  → /scan                           LaserScan  /scan_filtered
t2.5.sh scan_front_filter  /scan → /scan_filtered       Tool: "Nav2 Goal" → /goal_pose

t3_amcl.sh  localization.launch.py  (Phase 2, unchanged)
        ├── map_server   → /map [TransientLocal]
        ├── amcl         → /particle_cloud, /amcl_pose, map→odom TF
        └── lifecycle_manager_localization

t4_nav.sh   navigation.launch.py  (NEW in Phase 3)
        ├── planner_server     → /plan          (global path, NavFn)
        ├── controller_server  → /cmd_vel       (DWB local planner)
        │       global_costmap (map frame): static + obstacle + inflation
        │       local_costmap  (odom frame): rolling obstacle + inflation
        ├── behavior_server    → spin / backup / wait (recoveries)
        ├── bt_navigator       ← /navigate_to_pose action server
        ├── waypoint_follower  ← /follow_waypoints action server
        └── lifecycle_manager_navigation (autostart: configures+activates all 5)

ACTION FLOW (Milestone 2):
  nav_goal_client_node  ──/navigate_to_pose goal──►  bt_navigator
                        ◄──feedback (distance_remaining)──
                        ◄──result (succeeded/aborted/canceled)──

TF chain (identical to Phase 2 — Nav2 consumes it, doesn't change it):
  map → odom            AMCL
  odom → base_footprint EKF
  base_footprint → ...  robot_state_publisher
```

---

## 6. Pre-flight checklist

### 6a. One-time (before first session)

- [ ] Create the files in §4, run the colcon build, confirm
      `nav_goal_client_node` shows in `ros2 pkg executables cat_patrol_robot`.
- [ ] Verify launch reachable:
      `ros2 launch cat_patrol_robot navigation.launch.py --show-args`
- [ ] Confirm Nav2 packages installed:
      `for p in nav2_controller nav2_planner nav2_behaviors nav2_bt_navigator nav2_waypoint_follower nav2_lifecycle_manager; do ros2 pkg prefix $p; done`
- [ ] Ensure host RViz Docker is up (`rviz` container, same as Phase 1/2).

### 6b. Every session

- [ ] Robot at the tape mark (you still set the initial AMCL pose by hand).
- [ ] Battery > 11.5 V: `ros2 topic echo /voltage --once`
- [ ] Reboot Jetson if swap > 512 MiB: `free -h`
- [ ] `sudo systemctl stop ollama` (frees ~100 MB — Nav2 is heavier than AMCL).
- [ ] Verify USB ports (chassis CH340 7523, lidar CP2102 by-id).
- [ ] **Clear a real drivable path** in the room — Nav2 *will* drive the robot.
      Have the joystick in hand as a kill/override.

---

## 7. Session procedure

### Step A — Jetson terminals (in order)

Use the scripts in `~/myscripts2/`:

```bash
bash ~/myscripts2/t1.sh        # T1: bringup        → wait: cmd_vel trim=0.0120
bash ~/myscripts2/t2.sh        # T2: lidar          → wait: SLLidar health OK
bash ~/myscripts2/t2.5.sh      # T2b: scan filter   → wait: Keeping scan angles
bash ~/myscripts2/t3_amcl.sh   # T3: localization   → wait: Managed nodes are active
bash ~/myscripts2/t4_nav.sh    # T4: navigation     → wait: Managed nodes are active
```

Success signs from **T4**:
- `[lifecycle_manager_navigation] Configuring controller_server`
- … Configuring planner_server / behavior_server / bt_navigator / waypoint_follower
- `[lifecycle_manager_navigation] Activating ...` for each
- `[lifecycle_manager_navigation] Managed nodes are active`
- No `exit code -9` (OOM — Nav2 is the heaviest stack so far; see §8).

Check memory after T4 is active:
```bash
free -h
# Nav2 (controller + planner + 2 costmaps + BT) is heavier than AMCL.
# Expect ≥ 1.5–2 GiB available. If lower, stop ollama / reboot before retrying.
```

### Step B — Host RViz

Same RViz container as Phase 1/2. Displays for Phase 3:

| Display | Topic | Notes |
|---|---|---|
| Fixed Frame | `map` | |
| Map | `/map` | Durability **Transient Local** |
| Global Costmap | `/global_costmap/costmap` | colormap; shows inflation around walls |
| Local Costmap | `/local_costmap/costmap` | 3×3 m rolling window around robot |
| Path | `/plan` | The global plan (green line) |
| LaserScan | `/scan_filtered` | live obstacles |
| PoseArray | `/particle_cloud` | AMCL (keep an eye it stays tight while driving) |
| Pose | `/amcl_pose` | best-estimate pose |
| RobotModel | — | optional; confirms footprint vs costmap |

RViz toolbar: **"2D Pose Estimate"** (set initial AMCL pose) and
**"Nav2 Goal"** (send a navigation goal).

### Step C — Localize first (Phase 2 step)

1. Click **"2D Pose Estimate"**, click the robot's real location, drag heading.
2. Confirm particle cloud appears and tightens (drive a little if needed).
   **Do not send a Nav2 goal until AMCL is converged** — a bad pose makes Nav2
   plan from the wrong place and drive into a wall.

### Step D — Milestone 1: RViz goal

1. Click **"Nav2 Goal"** in the toolbar.
2. Click a free spot on the map, drag to set final heading, release.
3. A green `/plan` should appear routing around walls.
4. The robot drives it and stops within ~25 cm of the goal.
5. Watch the local costmap: real obstacles (a chair, a cat) should appear as
   coloured cells and the path should bend around them.

If the robot does not move:
- `ros2 topic echo /cmd_vel` — is the controller publishing?
- `ros2 action list | grep navigate` — is `/navigate_to_pose` there?
- Check T4 log for `Received a goal` and controller errors.

### Step E — Milestone 2: C++ action client

In a new sourced Jetson terminal (env from §10), with the stack running:

```bash
# Edit goal_x / goal_y to a real free cell first, or pass as params:
ros2 run cat_patrol_robot nav_goal_client_node --ros-args \
  -p goal_x:=1.0 -p goal_y:=0.0 -p goal_yaw:=0.0
```

Expect:
```
Sending goal: x=1.00 y=0.00 yaw=0.00
Goal ACCEPTED, navigating...
  distance remaining: 0.95 m
  distance remaining: 0.62 m
  ...
goal succeeded
```

Test the other result paths:
- **aborted:** give an unreachable goal (inside a wall) → `goal aborted`.
- **canceled:** add a cancel (Ctrl-C with a SIGINT handler, or extend the node
  with a timer that calls `async_cancel_goal`) → `goal canceled`. Cancellation
  is the Phase 6 hook — wiring it now is worthwhile.

### Step F — Verify

On Jetson (new sourced terminal):
```bash
ros2 action list                       # expect /navigate_to_pose, /follow_waypoints
ros2 topic hz /cmd_vel                 # nonzero while driving
ros2 topic echo /plan --once | head    # path poses in map frame
ros2 run tf2_ros tf2_echo map base_footprint   # robot pose tracks reality
```

---

## 8. Pitfalls and recovery

| Symptom | Cause | Fix |
|---|---|---|
| `lifecycle_manager_navigation` never reaches "active" | One managed node failed to configure (bad param, missing plugin) | Read T4 log for the first node that errored; check its block in `nav2_params.yaml` |
| Robot does not move after Nav2 Goal | Controller not publishing `/cmd_vel`; or AMCL not localized | `ros2 topic echo /cmd_vel`; re-set 2D Pose Estimate; check controller log |
| `/plan` routes *through* a wall | Inflation too tight, or AMCL pose wrong | Raise `inflation_radius`; re-localize; confirm static layer loaded the map |
| Nav2 refuses to plan through a doorway | Inflation too *generous* for the gap | Lower `inflation_radius` toward `0.18`; iterate (plan.md tradeoff) |
| Robot oscillates / spins near goal | DWB goal critics fighting; tolerance too tight | Raise `xy_goal_tolerance`/`yaw_goal_tolerance`; lower `RotateToGoal.scale` |
| Robot drives into something behind it | Rear is unseen — costmap uses `/scan_filtered` (front arc only) | Expected limitation. Approach/patrol forward-first; don't set goals requiring blind reversing |
| Costmap shows phantom walls that don't clear | `raytrace_max_range` < `obstacle_max_range`, or stale obstacles | Ensure raytrace range ≥ obstacle range; `ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}` |
| Robot curves / undershoots consistently | Phase 0 mechanical bias; trim only tuned for teleop forward | Nav2 closes the loop on `/odom`; small residual is normal. Don't chase with trim |
| `controller_server` exits -9 | OOM — Nav2 is the heaviest stack | `sudo systemctl stop ollama`; reboot if swap high; close RViz TF display |
| AMCL diverges *while* navigating | Robot driving faster than AMCL updates, or dynamic obstacles | Lower `max_vel_x`; ensure `/scan_filtered` ~10 Hz; re-localize |
| `Nav2 Goal` does nothing in RViz | RViz publishing to wrong topic / frame | Confirm tool publishes `/goal_pose` in `map` frame; check `bt_navigator` log |
| C++ client: "action server not available" | `bt_navigator` not active, or wrong action name | `ros2 action list`; confirm `/navigate_to_pose`; wait for T4 active |
| C++ client hangs after "accepted", no result | Executor not spinning, or goal genuinely in progress | Confirm `rclcpp::spin` running; watch `/cmd_vel`; the result fires on arrival |

---

## 9. Concept anchors (action clients — the Phase 3 learning goal)

From plan.md: *"writing the C++ client for `NavigateToPose` exercises goal
handles, feedback callbacks, and `std::shared_future`."*

The ROS 2 **action** = a long-running request with three channels:
1. **Goal** — the request (a target pose). Server may *accept* or *reject*.
2. **Feedback** — periodic progress (here: `distance_remaining`).
3. **Result** — terminal outcome: `SUCCEEDED` / `ABORTED` / `CANCELED`.

This is the asynchronous cousin of a service. Where a service blocks until a
single response, an action streams feedback and lets you **cancel** mid-flight
— which is exactly why Phase 6 (cancel-on-cat-sighting) needs it.

The C++ flow (trace it in `nav_goal_client_node.cpp`):
```
create_client<NavigateToPose>("navigate_to_pose")
  → wait_for_action_server()
  → async_send_goal(goal, opts)
       opts.goal_response_callback  → accepted? (GoalHandle::SharedPtr, null = rejected)
       opts.feedback_callback       → distance_remaining stream
       opts.result_callback         → WrappedResult.code switch
```

**The futures connection (C++ Concurrency in Action).** `async_send_goal`
returns a `std::shared_future<GoalHandle::SharedPtr>`. The callbacks are how
you *react* to that future completing without blocking the executor thread —
the same idea as `std::future::then`-style continuation. The executor keeps
spinning (handling feedback, TF, timers) while the goal is in flight.

**Where to read the code:**
- `rclcpp_action/include/rclcpp_action/client.hpp` — the `Client<>` template,
  `SendGoalOptions`, `async_send_goal`, `async_cancel_goal`.
- `rclcpp_action/include/rclcpp_action/client_goal_handle.hpp` — `WrappedResult`
  and `ResultCode`.
- `nav2_msgs/action/NavigateToPose.action` — the Goal/Result/Feedback fields.
- `nav2_bt_navigator/src/bt_navigator.cpp` — the *server* side: how the action
  is hosted and how the behavior tree drives planner + controller.

This is the natural follow-on to Phase 2's lifecycle-node study: lifecycle was
the *server bring-up* pattern; actions are the *runtime request* pattern.

---

## 10. Commands cheat-sheet

**Every Jetson terminal:**
```bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
```

**Start full stack (Jetson):**
```bash
bash ~/myscripts2/t1.sh        # bringup
bash ~/myscripts2/t2.sh        # lidar
bash ~/myscripts2/t2.5.sh      # scan filter
bash ~/myscripts2/t3_amcl.sh   # localization (Phase 2)
bash ~/myscripts2/t4_nav.sh    # navigation   (Phase 3)
```

**Verify everything is active:**
```bash
for n in controller_server planner_server behavior_server bt_navigator waypoint_follower; do
  echo -n "$n: "; ros2 lifecycle get /$n
done
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server
```

**Actions & navigation topics:**
```bash
ros2 action list                       # /navigate_to_pose, /follow_waypoints
ros2 action info /navigate_to_pose     # show the server
ros2 topic hz /cmd_vel                 # nonzero while driving
ros2 topic echo /plan --once | head
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```

**Run the C++ client (Milestone 2):**
```bash
ros2 run cat_patrol_robot nav_goal_client_node --ros-args \
  -p goal_x:=1.0 -p goal_y:=0.0 -p goal_yaw:=0.0
```

**Send a goal from the CLI (no RViz, sanity check):**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

**Clear costmaps if they accumulate phantoms:**
```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap {}
```

**TF / pose checks:**
```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_tools view_frames
```

---

## 11. Phase 3 session log

### Session 1 — 2026-07-04

- Build SHA / colcon clean: **yes** — all §4 files created, `cat_patrol_robot`
  built clean, `nav_goal_client_node` registered, `navigation.launch.py` reachable.
- All nav nodes reached active: **yes** (after the BT-XML fix below).
- Milestone 1 (RViz goal) — path planned around walls: **yes**
- Milestone 1 — stopped within tolerance: **yes** — "**Milestone 1 reached**".
- AMCL stayed converged while driving: **mostly** — degraded once after a bringup
  restart (see Finding C).
- Milestone 2 (C++ client) — succeeded printed: **partial** — node builds & runs and
  sends goals; the three result paths (succeeded / aborted / canceled) and the real
  `async_cancel_goal` cancel were **not yet fully exercised**. Carry to Session 2.
- max_vel_x that felt right: **0.10** (crawl — traction over speed; see Finding D).
- Anything unusual: **yes — lots.** See "Session 1 findings & fixes" below.

### Session 1 findings & fixes (real-world issues hit)

These are the non-obvious things that cost time. Read before Session 2.

**Finding A — `bt_navigator` needs an ABSOLUTE path to the BT XML.**
A bare filename (`navigate_to_pose_w_replanning_and_recovery.xml`) fails with
`Couldn't open input XML file` and the whole `lifecycle_manager_navigation`
aborts bringup. Fixed in `nav2_params.yaml` by pointing
`default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml` at the full
`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/…xml` paths.

**Finding B — the joystick floods `/cmd_vel` and blocks Nav2 (THE big one).**
With the stock bringup (`use_joystick:=true`), `yahboom_joy_X3` publishes a
zero-`Twist` **3× per joystick callback**, and `joy_node` autorepeats ~19 Hz →
**~57 Hz of zeros on `/cmd_vel`**. Nav2's `controller_server` publishes at only
10 Hz, so the base's last-received command is almost always a joystick zero →
robot barely twitches → `controller_server: Failed to make progress` → recovery
`backup` also fails. The plan's assumption ("joystick = safety override only")
is **wrong for this bringup** — the joystick is wired straight onto `/cmd_vel`
with no arbitration and *fights* Nav2.
- **Fix used:** relaunch T1 with `use_joystick:=false` (the launch arg's own
  description says "set false for autonomous patrol"). Then `/cmd_vel` has exactly
  one publisher (`controller_server`) and the robot drives.
- **Safe because:** `Mcnamu_driver_X3.py` has a **0.6 s cmd_vel watchdog**
  (`cmd_timeout_sec`) — if `/cmd_vel` goes silent it forces `set_car_motion(0,0,0)`.
  So Ctrl-C on T4 is a real e-stop (motors stop ≤0.6 s); no runaway.
- **To regain the joystick as a true override later:** it needs (1) a ~15-line edit
  to `yahboom_joy_X3.py` so it only publishes when a stick/deadman is active (stop
  the idle-zero flood), then (2) a `twist_mux` giving the joystick priority. A plain
  twist_mux alone does NOT work — the joy node's continuous zeros would still win.

**Finding C — "costmap rotated vs walls" == AMCL heading error, not a Nav2 bug.**
After restarting bringup (for Finding B), odom reset and the re-set 2D Pose
Estimate had an off heading → AMCL locked onto a rotated pose → laser lands
rotated onto the static map → obstacle layer looks rotated vs the walls →
distant goals fail. The static walls come straight from `/map` and can't rotate;
only the laser-placed layer can. Verified the laser TF is fine:
`base_link → laser_link` yaw = **180.0°** (the intact `yaw=π` mounting fix, see
[[project_slam_map_collapse]]). **Fix:** re-do 2D Pose Estimate carefully
(heading is what matters), confirm `/scan_filtered` snaps onto the walls, nudge
the robot (keyboard teleop, since joystick is off now) so AMCL tightens.

**Finding D — traction: the robot can't turn in place on the slick floor (UNSOLVED).**
Central ~2×2.3 m **carpet** area = grip is fine, drives and turns OK. **Beyond the
carpet the floor is slippery** and in-place rotation fails: the point-turn
(left wheels forward, right wheels backward) is pure sideways **scrubbing** with
almost no rolling motion, so the wheels break static friction and spin without
rotating the body. This is a **friction/physics limit**, not a tuning bug — no
speed setting creates grip that isn't there.

Mitigations applied (all in `nav2_params.yaml`, no rebuild — T4 restart reloads):
1. **Tier 1 — crawl + gentle (DWB):** `max_vel_x 0.18→0.10`, `max_vel_theta 1.0→0.25`,
   `acc_lim_x 2.5→0.5`, **`acc_lim_theta 0.5→0.10`** (the key lever — gentle
   break-away keeps tires under the static-friction limit), tolerances tightened
   to 0.15 for precision, and the progress checker **relaxed**
   (`required_movement_radius 0.5→0.10`, `movement_time_allowance 10→25`) so crawl
   speed doesn't trigger false `Failed to make progress` aborts.
2. **Tier 2 — switched controller DWB → Regulated Pure Pursuit (RPP)** with
   **`use_rotate_to_heading: false`** so it **never point-turns** — it arcs forward
   through every turn (rolling traction ≫ scrubbing traction). DWB kept as a
   commented fallback block in the same file (flip back = swap the two `FollowPath`
   blocks + restart T4). Because RPP-without-in-place-rotation can't achieve a
   *commanded* final heading, `yaw_goal_tolerance` was set to **3.14 (ignore yaw)**,
   or goals would never complete — **position stays precise (0.15 m), final facing
   is whatever the arc gives.**

**Status: parked, still not ideal.** Turning on the slick floor is improved but
not solved. Remaining options, in order of effort:
- **Mechanical (real cure):** rubber O-rings / grip tape on the wheels, or a runner
  rug along the patrol route. Scrubbing is a friction problem; grip fixes it everywhere.
- **Option B — custom `/cmd_vel` pivot-shaper (~30-line node):** rewrite the
  controller's near-pure-spin commands (`vx≈0, |ωz|>0`) into a **pivot turn**
  `vx = |ωz| · track/2` (track ≈ 0.25 m → pivot radius ≈ 0.12 m) so the inner wheel
  stops and the robot pivots about one side instead of scrubbing. Tighter than RPP's
  arc; needs the controller loop to tolerate the added translation.
- **RPP tuning:** if arcs are too wide for the room, tighten lookahead / min radius.

**Config note:** all Phase 3 params live in `config/nav2_params.yaml`, read directly
from the source tree by `navigation.launch.py` — **edits need only a T4 restart, no
`colcon build`.** Only C++ (`nav_goal_client_node.cpp`) changes need a rebuild.

### Session 2 — 2026-07-05  (controller A/B: MPPI; reverse + in-place-rotation experiments)

**Focus:** evaluate alternative Nav2 controllers (esp. MPPI) and test two traction
ideas on the slick floor — (B1) allow reverse, and (2) faster in-place rotation.

> **⚠️ CONFIG LEFT IN A TEST STATE.** `config/nav2_params.yaml` was NOT reverted at
> end of session. Before trusting normal behaviour next session, revert the params
> tagged `ROTATION TEST` and decide on the `TEST B(1)` reverse params (see below).

**Controller menu (verified installed on the Jetson, `/opt/ros/humble`):**
DWB, RegulatedPurePursuit (RPP), **MPPI** (`nav2_mppi_controller`), and RotationShim
are all present — no apt install needed. Switching = edit `nav2_params.yaml` +
restart T4 (no rebuild). MPPI is now the **active** `FollowPath`; RPP and DWB are
commented fallbacks in the same file.

**Key facts learned about MPPI on this robot:**
- MPPI in Humble is **CPU-vectorized (xtensor), NOT GPU-accelerated** — it never
  touches the Orin GPU. Budget is CPU/RAM; kept `batch_size: 1000`, `time_steps: 30`.
- **`model_dt` must be ≥ controller period** (`1/controller_frequency`). With
  `controller_frequency: 10.0`, `model_dt: 0.05` → FATAL "Controller period more then
  model dt". Fixed: `model_dt: 0.1` (== 0.1 s period). To go finer, raise freq to 20
  and set `model_dt: 0.05` (≈2× CPU).
- Why MPPI here: it's the only controller that does smooth forward-biased arcs
  (good on slick floor, like RPP) AND real obstacle avoidance (like DWB) — resolves
  the RPP-vs-DWB tension from Session 1.

**Bring-up gotcha re-confirmed (not a bug):** after T4 launches, `global_costmap`
spams `Timed out waiting for transform ... "map" ... frame does not exist`. Cause:
**AMCL is active but unlocalized** → it doesn't publish `map→odom` until it gets an
initial pose. Fix = set 2D Pose Estimate (or publish `/initialpose`). The costmap
retries (INFO, not fatal) and activates once the transform appears; no T4 restart
needed. Consider `set_initial_pose: true` in `amcl_params.yaml` for repeat runs.

**Experiment B(1) — allow reverse (config currently ACTIVE):**
Reverse needs THREE knobs, not just `vx_min` — the other two actively suppress it:
- `vx_min: -0.10` (was 0.0) — the actual reverse-enable.
- `PreferForwardCritic.enabled: false` (was true) — at weight 5.0 it suppresses reverse.
- `PathAngleCritic.mode: 1` (NO_DIRECTIONAL_PREFERENCE) — else the 180° heading of
  reverse is penalized.
- ⚠️ **Rear is BLIND** (costmap = `/scan_filtered` front arc only) — reverse cannot
  see obstacles behind. Not fully road-tested; test in open space, joystick in hand.

**Experiment 2 — faster in-place rotation (config currently ACTIVE): FAILED, informative.**
Raised `wz_max: 0.5→1.5`, `wz_std: 0.3→0.5`, `TwirlingCritic: 10→2` (so MPPI would
spin in place instead of arcing).
- **Result: instead of rotating, the robot drove SIDEWAYS ~90°.**
- **Why (mecanum physics):** an in-place mecanum rotation is the sum of four 45°
  roller forces whose *sideways* components cancel ONLY if all four wheels grip
  equally. On the slick floor grip is unequal → the cancellation breaks → the
  leftover diagonal components sum to a **net lateral force → the rotation bleeds
  into a strafe.** Rotation and strafe are adjacent differential modes, so that's
  where it leaks. On the carpet the same command rotates fine → confirms it's
  **traction, not wiring**. "Faster spin" is disproven: more speed = more slip.
- **User's "asymmetric wheel" idea** (e.g. right back x, left forward 4x) works out
  to net `vx=1.5x forward + strong wz` = a **forward ARC/pivot**, NOT in-place. That's
  the *right* instinct (wheels roll instead of scrub) — and Nav2 already produces it
  as a normal `vx>0 + wz` command; no per-wheel control needed (`set_car_motion` only
  takes vx,vy,wz). It equals the arc approach we already had (RPP `use_rotate_to_heading:
  false` / MPPI with high `TwirlingCritic`).

**CONCLUSION — in-place rotation on the bare floor is a dead end on this mecanum base;
force it and it strafes. The robust answer is arc/pivot turning (wheels roll).**
Two-mode plan (refined): NOT "spin faster off-carpet" but — **on carpet:** in-place
rotation allowed; **off carpet:** arcs only (no in-place turn). Implement as a
per-surface switch of `TwirlingCritic` weight / `use_rotate_to_heading`, once the
carpet region is designated on the map. Mechanical grip (tape/rug) remains the real cure.

**Also this session (non-controller):**
- `~/myscripts2/t4_nav.sh` now runs `sudo systemctl stop ollama` before launch
  (guarded, won't hang) — MPPI is the heaviest stack; removes the risk a stray LLM
  query loads a multi-GB model mid-drive and OOM-kills `controller_server` (-9).
  Idle ollama is only ~50 MB (no model resident), so it's insurance, not urgent.
- **ollama is NOT part of cat detection.** Phase 5 = YOLOv8→TensorRT on GPU
  (verified: TensorRT 10.7 + trtexec present). ollama serves LLMs (its 12 models are
  mostly a leftover local *coding* assistant — starcoder2/deepseek-coder/codellama +
  the "ollama + Cursor" note). Keep it stopped during patrol; on 8 GB unified memory a
  7B model (4–5 GB) would blow the budget. Nice symmetry: Nav2/MPPI = CPU, YOLO = GPU.

**TODO next session:**
1. Decide + revert config: `ROTATION TEST` params back to `wz_max: 0.5`, `wz_std: 0.3`,
   `TwirlingCritic: 10`. Keep or drop reverse B1 (`vx_min`, `PreferForwardCritic`,
   `PathAngleCritic.mode`) — all tagged in `nav2_params.yaml`.
2. Finish MPPI Milestone 1 acceptance (drive a goal, arc turns, stop in tolerance).
3. Milestone 2 C++ client result paths (succeeded/aborted/canceled) still not exercised
   (carried from Session 1).
4. Build the carpet-vs-bare-floor mode switch (map region → TwirlingCritic/rotate mode).

### Session 3 — 2026-07-06  (ROOT CAUSE FOUND: linear.y ↔ angular.z swapped on the chassis)

**This session overturns the entire "can't turn on the slick floor / traction" thesis
of Sessions 1–2.** The turning problem was never (mainly) traction — the base has
`linear.y` and `angular.z` **physically exchanged** below the driver.

**How it was found.** User noticed that with the joystick, the LEFT stick "rotated
the car well" on the smooth floor while the RIGHT stick "slid it aside" (the Nav2
failure). Recorded `/joy` + `/cmd_vel` (`scratchpad/joy_cmd_logger.py`): LEFT stick =
`axes[0]` → **pure `linear.y`** (strafe cmd); RIGHT stick = `axes[2]` → **pure
`angular.z`** (yaw cmd). So the "good rotation" was a **strafe command**, and the
"slide" was a **yaw command** — inverted from their meaning.

**Confirmed by controlled Twist tests** (`scratchpad/pub_twist.py`, joystick off so
`/cmd_vel` has one publisher):
- pure `linear.y=0.12` → robot **rotated** (full clean circle on the bare floor)
- pure `angular.z=0.4` → robot **strafed** sideways
- `vx + linear.y` → smooth arc/spin

**Kinematics proof.** Strafe and rotate mecanum wheel-patterns differ ONLY in the two
rear wheels; `vx` (all wheels same sign) is immune. A rear-wheel swap gives EXACTLY
`vx→vx, vy→wz, wz→vy` (no residue) — which is why forward driving always looked fine
and only turns misbehaved. Root cause is below `Mcnamu_driver_X3.py` line 128
(`set_car_motion` args are in the CORRECT order) — i.e. MCU/library mixing or a
physical wheel/lead issue (hypothesis at this point; RESOLUTION below pins it to
**mis-mounted rear wheels**). Also explains [[project_odom_imu_fusion]] "wheel rotation
dead ~5°/90°": `get_motion_data()` returns vy/angular swapped too, so true rotation
was read back as strafe → angular≈0.

**INTERIM software workaround (built + verified, then REVERTED — superseded by the
hardware fix in RESOLUTION below; kept here for the record only):**
- `Mcnamu_driver_X3.py`: params `swap_vy_wz` (bool, default False), `swap_wz_scale`
  (0.12), `swap_vy_scale` (1.0), + live `add_on_set_parameters_callback` (tune via
  `ros2 param set /driver_node swap_wz_scale X`, no restart). When on:
  `set_car_motion(vx, w_corrected*swap_wz_scale, vy_corrected*swap_vy_scale)`.
- `yahboomcar_bringup_X3_launch.py`: matching launch args wired to the driver.
- Enable: `bash t1 ... swap_vy_wz:=true trim_vy_per_vx:=0.0`. Log shows `swap_vy_wz=True`.
- Symlink install → no rebuild, just restart t1. Default OFF preserves old behaviour.

**Angular-scale calibration (IMU `/imu/data_raw` gyro, `scratchpad/calib_wz.py`).**
Feeding a rad/s value into the library's m/s (strafe) slot over-rotates ~**11.5×**.
Calibrated `swap_wz_scale`:
| commanded wz | actual (gyro) @ scale 0.12 | gain |
|---|---|---|
| 0.2 | 0.096 | 0.48× (deadband, weak) |
| 0.4 | 0.420 | **1.05× (tuned point)** |
| 0.8 | 1.038 | 1.30× (hot) |
Nonlinear (strafe deadband at low end); `0.12` nails mid-range, closed-loop AMCL
absorbs the ends. `/imu/data_raw` gyro-z sign is inverted vs command (CCW→−gz);
direction correct by eye, magnitude used for calib.

**What this retires from Sessions 1–2:** the grip-tape/rug plan, RPP `use_rotate_to_
heading:false`, MPPI `TwirlingCritic` spin tuning, the carpet-vs-bare-floor mode
switch, and "in-place rotation is a dead end on mecanum" — all were treating symptoms
of the swap. In-place rotation works fine once `angular.z` actually rotates.

**RESOLUTION (same session) — HARDWARE FIX, root cause was mis-mounted wheels.**
Rather than ship the software swap, we chased the true cause:
- **Per-wheel `set_motor` test** (t1 stopped, standalone script): each channel drives
  its own correct wheel, all spin forward for +cmd (m1=FL, m2=RL, m3=FR, m4=RR). So
  **wiring + motor directions are PERFECT** — not a crossed lead.
- **`set_car_motion` does no mixing in Python** — it ships (vx,vy,vz) to the MCU;
  the mecanum mixing is in firmware. `set_motor` bypasses it.
- **Hand-built textbook STRAFE pattern** (FL-,FR+,RL+,RR-, a pure-roller pattern)
  via `set_motor` produced **ROTATION**, not strafe → purely the physical wheels →
  **two mecanum wheels mounted with the wrong roller orientation** ("sides-match"
  instead of diagonal-X). Firmware innocent.
- **FIX: swapped the two REAR wheels (rear-left <-> rear-right).** Re-tested NATIVE
  (`set_car_motion`, no ROS/software): `(0,0,1.0)`→rotates left, `(0,0.3,0)`→strafes
  left (+y correct), `(0.2,0,0)`→straight forward. All correct, NOT mirrored.
- **Reverted ALL the software hacks** (swap_vy_wz/scale params + param-callback in
  `Mcnamu_driver_X3.py` and launch). `t1.sh` restored to `use_joystick:=true`, no
  swap arg, dropped `trim_vy_per_vx:=0.012` (hardware-fixed → not needed).
- **Odom auto-fixed too:** correct wheels → correct encoder→odom, so the old "wheel
  rotation dead ~5°/90°" is resolved; the planned odom-read software fix is moot.

**TODO Session 4:**
1. Bring up the CLEAN stack (`t1` normal) and verify end-to-end through ROS:
   `/cmd_vel` angular.z rotates, linear.y strafes, `/odom` reports rotation now.
2. `nav2_params.yaml` **cleaned (done this session):** MPPI reverted to sane
   post-fix defaults — `vx_max 0.10→0.18`, `vx_min -0.10→0.0` (forward-only),
   `wz_max 1.5→1.0`, `wz_std 0.5→0.2`, `PreferForwardCritic` on, `PathAngleCritic
   mode 0`, `TwirlingCritic 2→10`, goal `yaw_goal_tolerance 0.5→0.25`, progress
   checker back to defaults. RPP/DWB remain commented fallbacks. **Still TODO:
   re-run Nav2 (t4) and verify a goal drives + turns cleanly.**
3. EKF: may switch heading back to wheel-odom, or keep IMU (both fine now).
4. Milestone 2 C++ client result paths (still carried from Sessions 1–2).

### Final results

- nav2_params that worked: inflation=___, max_vel_x=___, robot_radius=___
- Milestone 1: PASS / blockers ___
- Milestone 2: PASS / blockers ___
- Phase 3 acceptance criteria: all met / blockers ___

---

## 12. Open questions / forward links

- **Phase 4 (patrol manager).** The `nav_goal_client_node` becomes the
  navigation primitive inside the new `patrol_manager` package — driving
  between recorded waypoints. The single-goal client grows into a goal
  *sequencer* with an FSM and callback groups.
- **`NavigateThroughPoses`.** For multi-waypoint patrol, the
  `NavigateThroughPoses` action (or `waypoint_follower`) avoids stop-and-replan
  at each point. Worth prototyping at the end of Phase 3.
- **Goal cancellation (Phase 6).** `async_cancel_goal` is stubbed conceptually
  in §7 step E. Phase 6 cancels the active nav goal the instant a cat is
  detected, then resumes from the interrupted waypoint. Get the cancel path
  working now while the client is simple.
- **Rear blindness.** Costmaps use `/scan_filtered` (front arc only), so the
  robot cannot see obstacles behind it. If patrol paths ever need reversing,
  revisit the scan filter or add the depth camera as a second obstacle source.
- **Controller choice.** We use DWB (carried from stock Yahboom config). The
  Regulated Pure Pursuit controller (RPP) is often smoother for a
  differential-style drive and worth A/B testing in Phase 4 tuning.

---

## 13. Learning Guide — Phase 3 deep dive

Read this before the first session. It mirrors the Phase 2 §13 guide:
how the topics connect, what each component does (input/output), what each
RViz view shows, and how to verify every topic from the command line.

---

### 13a. Full data-flow diagram (localization + navigation together)

```
                          ┌─────────────── /map (OccupancyGrid, TransientLocal) ───────────────┐
                          │                                                                    │
 [map_server] ───────────┤                                                          (static layer)
                          │                                                                    │
                          ▼                                                                    ▼
 /scan ─► [scan_front_filter] ─► /scan_filtered ─┬──► [amcl] ──► /amcl_pose            [global_costmap]
                                                 │             ──► /particle_cloud      static+obstacle+inflation
                                                 │             ──► TF map→odom                  │
                                                 │                                              ▼
                                                 ├──────────────────────────────────► [planner_server]
                                                 │                                       reads costmap + goal
                                                 │                                       ──► /plan (Path)
                                                 │                                              │
                                                 │                                              ▼
                                                 └──► [local_costmap] ◄──────────────── [controller_server]
                                                       obstacle+inflation                reads /plan, /odom,
                                                       (odom frame, rolling)             local_costmap
                                                                                         ──► /cmd_vel (Twist)
                                                                                              │
 [ekf] ─► /odom, TF odom→base_footprint ──────────────────────────────────────────────┐     ▼
                                                                                        │  [base_node] ─► motors
 USER RViz "Nav2 Goal" ─► /goal_pose ─┐                                                 │
 nav_goal_client_node ─► action goal ─┴─► [bt_navigator] ── orchestrates ──► planner + controller
                                          (NavigateToPose action server)    + behavior_server
                                          ◄── feedback (distance_remaining) ──
                                          ◄── result (succeeded/aborted/canceled) ──
                                                          │
                                          on failure ───► [behavior_server] spin / backup / wait
```

---

### 13b. Component reference — what each node does, inputs, outputs

#### `planner_server` (package: `nav2_planner`, plugin: NavFn)

| | |
|---|---|
| **What it does** | Computes a **global** collision-free path from the robot's current pose to the goal, over the global costmap. NavFn = Dijkstra/A* over the grid. |
| **Inputs** | `/global_costmap/costmap`; goal pose (from bt_navigator); TF `map → base_footprint` |
| **Output** | `/plan` (nav_msgs/Path) — a sequence of poses in the `map` frame |
| **Key params** | `tolerance` (how close to the goal cell is acceptable), `use_astar`, `allow_unknown` |
| **Lifecycle** | Managed by `lifecycle_manager_navigation` |

#### `controller_server` (package: `nav2_controller`, plugin: DWB)

| | |
|---|---|
| **What it does** | Follows the global `/plan` locally. DWB samples many short candidate trajectories each tick, scores them with *critics* (obstacle distance, path alignment, goal alignment), and publishes the winning velocity. |
| **Inputs** | `/plan`; `/local_costmap/costmap`; `/odom`; TF |
| **Outputs** | `/cmd_vel` (geometry_msgs/Twist); also a `progress_checker` and `goal_checker` decide "stuck" / "arrived" |
| **Key params** | `max_vel_x`, `max_vel_theta`, `xy_goal_tolerance`, DWB `critics` and their `.scale` weights, `sim_time` |
| **Lifecycle** | Managed |

#### `behavior_server` (package: `nav2_behaviors`)

| | |
|---|---|
| **What it does** | Hosts **recovery behaviors** the BT calls when navigation stalls: `spin` (rotate to clear), `backup` (reverse a bit), `wait` (pause for dynamic obstacle to pass). |
| **Inputs** | `local_costmap/costmap_raw`, robot footprint; triggered by bt_navigator |
| **Outputs** | `/cmd_vel` (during a recovery); action results back to the BT |
| **Key params** | `behavior_plugins` list, per-behavior velocity/accel limits |

#### `bt_navigator` (package: `nav2_bt_navigator`)

| | |
|---|---|
| **What it does** | The **brain**. Hosts the `/navigate_to_pose` action *server*. Runs a Behavior Tree (XML) that calls planner → controller, and on failure calls recoveries, with replanning. This is what your C++ client talks to. |
| **Inputs** | `/navigate_to_pose` goals (from RViz `/goal_pose` via a bridge, or directly from action clients); `/plan` availability; TF |
| **Outputs** | Drives planner + controller; emits action **feedback** (`distance_remaining`, `navigation_time`) and **result** |
| **Key params** | `default_nav_to_pose_bt_xml`, `global_frame`, `robot_base_frame`, `plugin_lib_names` |

#### `waypoint_follower` (package: `nav2_waypoint_follower`)

| | |
|---|---|
| **What it does** | Hosts `/follow_waypoints`: accepts a *list* of poses and navigates them in sequence, optionally pausing at each. The Phase 4 patrol primitive (preview). |
| **Inputs** | `/follow_waypoints` goal (list of PoseStamped) |
| **Outputs** | Drives `bt_navigator` for each waypoint; per-waypoint task executor (wait) |

#### `lifecycle_manager_navigation` (package: `nav2_lifecycle_manager`)

| | |
|---|---|
| **What it does** | Same role as Phase 2's localization manager, but for the navigation nodes. Configures then activates `controller → planner → behavior → bt_navigator → waypoint_follower` in order. Watchdog/bond keeps them alive together. |
| **Inputs / Outputs** | None (issues lifecycle service calls only) |

#### Costmaps (`nav2_costmap_2d`, embedded in planner/controller)

| | |
|---|---|
| **global_costmap** | Map-frame, full-room. Layers: **static** (the saved map) + **obstacle** (live `/scan_filtered`) + **inflation** (safety buffer around obstacles). Used by the planner. |
| **local_costmap** | Odom-frame, 3×3 m rolling window around the robot. Layers: **obstacle** + **inflation**. Used by the controller for immediate avoidance. |
| **Inflation layer** | Grows a cost gradient outward from every obstacle so paths keep clear. `inflation_radius` = how far the buffer extends; `cost_scaling_factor` = how steeply cost falls off. |

---

### 13c. The two-lifecycle-manager split — why

Nav2 deliberately separates **localization** and **navigation** into two
lifecycle managers:

| | `lifecycle_manager_localization` (Phase 2) | `lifecycle_manager_navigation` (Phase 3) |
|---|---|---|
| Manages | `map_server`, `amcl` | `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower` |
| Can restart independently | yes | yes |
| Why split | Localization must be up *before* navigation. You can bounce the nav stack (retune params) without losing your AMCL convergence. |

Startup order matters: localization (T3) must be **active** before navigation
(T4), because the planner needs `map → odom` and the static map to exist.

---

### 13d. The action mechanism (the heart of Milestone 2)

```
   nav_goal_client_node                         bt_navigator (action server)
   ─────────────────────                        ───────────────────────────
   async_send_goal(goal) ───────────────────►   receive goal
                          ◄────────────────────  accept/reject
   goal_response_callback(handle)
        handle == nullptr  → REJECTED
        handle != nullptr  → ACCEPTED, navigating
                          ◄────────────────────  feedback (distance_remaining)
   feedback_callback(fb)   (repeats ~1 Hz)
                          ◄────────────────────  result (code + result msg)
   result_callback(WrappedResult)
        SUCCEEDED / ABORTED / CANCELED

   (optional, Phase 6)
   async_cancel_goal(handle) ─────────────────►  preempt → CANCELED
```

**Three callbacks, three concerns:**
- `goal_response_callback` — did the server even take the job? (null handle = no)
- `feedback_callback` — progress while it runs (don't block here; just log)
- `result_callback` — terminal outcome; this is where you `shutdown()` or
  trigger the next action

**Why not just block?** `async_send_goal` returns a `std::shared_future`. You
*could* `spin_until_future_complete`, but then the executor can't process the
feedback stream or TF while waiting. The callback pattern keeps the single
executor thread free — the same reasoning as `std::future` continuations vs
`future.get()` blocking in *C++ Concurrency in Action*.

**Cancellation (Phase 6 hook).** `async_cancel_goal(handle)` requests preempt;
the server stops the robot and the `result_callback` fires with `CANCELED`.
This is *the* reason patrol uses an action, not a service: a service call can't
be interrupted mid-drive.

---

### 13e. RViz displays — what to look for in each panel

#### Global Costmap — topic `/global_costmap/costmap`
- **What you see:** A coloured overlay over the whole map. Walls (static layer)
  are lethal (usually purple/pink), with a **gradient halo** (inflation) fading
  to free space. Live obstacles from `/scan_filtered` also get inflated.
- **Check:** The halo thickness ≈ your `inflation_radius` (0.25 m = 5 cells at
  0.05 m/cell). If walls have *no* halo, the inflation layer isn't loaded. If
  the whole room is lethal, `cost_scaling_factor` is too low or radius too big.
- **Why it matters:** The planner routes through *low-cost* cells. Too-thin
  inflation → paths hug walls; too-thick → planner refuses doorways.

#### Local Costmap — topic `/local_costmap/costmap`
- **What you see:** A small 3×3 m coloured square that *follows the robot*
  (rolling window, odom frame). Only obstacle + inflation layers — no static
  map.
- **Check:** As you put a chair near the robot, a lethal blob + halo should
  appear and then **clear** when removed (raytracing). If it never clears,
  raytrace range is misconfigured.
- **Why it matters:** This is what the controller dodges in real time.

#### Path — topic `/plan`
- **What you see:** A line (often green) from the robot to the goal, the global
  plan.
- **Check:** It should bend *around* inflated walls, never cross lethal cells.
  It updates when you set a new goal or when replanning fires.

#### Map — topic `/map`
- Same as Phase 2: static occupancy grid. Durability **Transient Local** or it
  shows "no map received."

#### LaserScan — topic `/scan_filtered`
- Live front-arc returns. When localized, dots land on the map walls. These are
  also what *marks* obstacles into both costmaps.

#### PoseArray `/particle_cloud` + Pose `/amcl_pose`
- Same as Phase 2. **Watch these while navigating** — if the cloud spreads back
  out mid-drive, AMCL is losing localization and Nav2 will start planning from a
  wrong pose. Slow down or re-localize.

#### Tool: "Nav2 Goal"
- Publishes `/goal_pose` (geometry_msgs/PoseStamped, map frame). A small bridge
  inside `bt_navigator` turns it into a `/navigate_to_pose` action goal — so
  RViz goals and your C++ client hit the *same* server.

---

### 13f. Costmap layers — the mental model

A costmap is a stack of **layers**, combined per-cell into one cost grid:

```
 final cost = max over layers (per cell)

 ┌─ static_layer ───── the saved map (walls = lethal). global only.
 ├─ obstacle_layer ─── live /scan_filtered hits marked lethal; rays clear free space
 └─ inflation_layer ── gradient buffer grown outward from every lethal cell
```

- **marking** = add an obstacle where a beam ends.
- **clearing** (raytracing) = erase obstacles along a beam that passed through
  (so a cat that walked away stops blocking).
- **inflation** turns a hard wall into a soft "cost hill" so the planner prefers
  the middle of free space, not wall-hugging paths.

Cell cost values: `0` free … `253` inscribed (robot center would touch) …
`254` lethal … `255` unknown.

---

### 13g. Verify every topic from the command line

Run on the Jetson (env from §10), full stack up:

```bash
# ── Lifecycle states (all must be 'active') ───────────────────────────────────
for n in controller_server planner_server behavior_server bt_navigator waypoint_follower; do
  echo -n "$n: "; ros2 lifecycle get /$n
done
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server

# ── Actions ───────────────────────────────────────────────────────────────────
ros2 action list
# Expect: /navigate_to_pose  /follow_waypoints  /spin  /backup  /wait
ros2 action info /navigate_to_pose
# Expect: Action server: 1  (bt_navigator)

# ── Planner output ─────────────────────────────────────────────────────────────
ros2 topic echo /plan --once | grep -c position
# >0 only after a goal is set; the count = number of path poses

# ── Controller output ───────────────────────────────────────────────────────────
ros2 topic hz /cmd_vel
# Expect: ~10 Hz with nonzero values while a goal is active; silent when idle

# ── Costmaps ────────────────────────────────────────────────────────────────────
ros2 topic hz /global_costmap/costmap     # ~1 Hz
ros2 topic hz /local_costmap/costmap      # ~2 Hz
ros2 topic echo /global_costmap/costmap --once | grep -E "width|height|resolution"

# ── Localization still healthy under nav load ────────────────────────────────────
ros2 topic hz /particle_cloud             # ~2 Hz
ros2 topic echo /amcl_pose --once | grep -A3 position

# ── Scan feeding the costmaps ────────────────────────────────────────────────────
ros2 topic hz /scan_filtered              # ~10 Hz

# ── TF (Nav2 consumes, does not change) ──────────────────────────────────────────
ros2 run tf2_ros tf2_echo map base_footprint   # robot pose, tracks reality
ros2 run tf2_ros tf2_echo map odom             # AMCL correction, stable

# ── Node list sanity ─────────────────────────────────────────────────────────────
ros2 node list | sort
# Phase 3 must include (on top of Phase 2):
#   /controller_server /planner_server /behavior_server
#   /bt_navigator /waypoint_follower /lifecycle_manager_navigation

# ── End-to-end goal from CLI (no RViz needed) ────────────────────────────────────
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
# Streams feedback, then prints the result code. Same path the C++ client drives.
```

---

### 13h. How Phase 3 connects backward and forward

```
Phase 1 (map)      → living_room_v1.pgm/.yaml   → global_costmap static layer
Phase 2 (AMCL)     → map→odom TF + /amcl_pose   → planner/controller "where am I"
Phase 3 (Nav2)     → /navigate_to_pose action   → REUSED as Phase 4's drive primitive
                     /cmd_vel from controller    → replaces the old patrol_node raw cmd_vel
                     nav_goal_client_node.cpp    → grows into patrol_manager goal sequencer
                     async_cancel_goal           → Phase 6 cancel-on-cat-sighting
```

The whole Phase 1+2+3 stack (`t1`→`t4`) becomes the fixed "navigation
substrate." Phase 4 onward adds *application* logic (FSM, waypoints, capture,
mail, cat reaction) that only ever talks to Nav2 through the action interface —
never raw `/cmd_vel` again (plan.md rule).

---

## 14. C++ lessons to study (from `nav_goal_client_node.cpp`)

> A study checklist to go over later. Every item below appears in the ~110-line
> `src/nav_goal_client_node.cpp` — read the line, then the concept. Grouped from
> "ROS 2 action plumbing" → "modern C++ / concurrency" → "next-phase extensions."
> The recurring theme (plan.md): **react to work completing via callbacks/futures
> instead of blocking the thread** — the `std::future` idea from *C++ Concurrency
> in Action*, applied to a long-running robot action.

### 14a. ROS 2 action-client mechanics

- [ ] **`rclcpp_action::Client<ActionT>`** — the templated action-client type.
      How it differs from a service client (streams feedback + is cancelable).
- [ ] **`rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose")`**
      — the factory; the node owns the client; the action *name* must match the
      server (`bt_navigator`).
- [ ] **`wait_for_action_server(5s)`** — discovery/handshake before sending; the
      `5s` is a `std::chrono` literal (see 14c). What "server not available" means.
- [ ] **`SendGoalOptions`** — the struct bundling the **three callbacks**:
  - [ ] `goal_response_callback(GoalHandle::SharedPtr)` — **accept vs reject**; a
        **null** handle means rejected. (Why a server would reject.)
  - [ ] `feedback_callback(handle, shared_ptr<const Feedback>)` — the progress
        stream (`distance_remaining`). Rule: **don't block here**, just react.
  - [ ] `result_callback(WrappedResult)` — terminal outcome via
        **`rclcpp_action::ResultCode`** switch: `SUCCEEDED` / `ABORTED` / `CANCELED`.
- [ ] **`async_send_goal(goal, opts)`** — fire-and-react; returns a
      **`std::shared_future<GoalHandle::SharedPtr>`** (the futures link, 14b).
- [ ] **`ClientGoalHandle<ActionT>`** — the handle that represents the in-flight
      goal; needed to query status and to **cancel** (14d).
- [ ] **Action interface anatomy** — read `nav2_msgs/action/NavigateToPose.action`:
      the `Goal` / `Result` / `Feedback` three-part message and how the generated
      C++ types (`NavigateToPose::Goal`, `::Feedback`, `WrappedResult`) map to it.

### 14b. The futures / concurrency connection (the core learning goal)

- [ ] **`std::shared_future` vs `std::future`** — why `async_send_goal` returns a
      *shared* future; multiple observers / copyable.
- [ ] **Callbacks as continuations** — reacting to a future completing *without*
      calling `.get()`. Contrast with **`rclcpp::spin_until_future_complete`**
      (which blocks the executor — wrong here: it would freeze feedback + TF).
- [ ] **Why not block?** The single executor thread must keep spinning to deliver
      feedback, TF, and timers while the goal is in flight. Map this to
      `std::future`/continuation reasoning in *C++ Concurrency in Action*
      (`std::packaged_task`, `std::async`, `future.then`-style flow).
- [ ] **Executor model** — `rclcpp::spin(node)` in `main`; how callbacks are
      dispatched on it; where `rclcpp::shutdown()` (in `result_callback`) ends it.
- [ ] **Deferred one-shot send** — `create_wall_timer(500ms, …)` then
      `timer_->cancel()` inside the callback: *why* defer the send until the
      executor is actually spinning (so the callbacks can fire).

### 14c. Modern C++ idioms used

- [ ] **Type aliases** — `using NavigateToPose = …;`
      `using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;`
      (taming long template names).
- [ ] **`std::chrono` literals** — `using namespace std::chrono_literals;` →
      `500ms`, `5s`. Where the `using` belongs and why.
- [ ] **Lambdas with capture** — `[this](…){ … }` for each callback; capturing
      `this` to call `get_logger()` / member state; lambda vs `std::bind`
      (the timer uses `std::bind(&NavGoalClient::send_goal, this)` — compare).
- [ ] **Smart pointers** — `std::make_shared<NavGoalClient>()`,
      `SharedPtr` typedefs everywhere, `std::shared_ptr<const Feedback>`
      (const-correct shared ownership). Node lifetime vs `shared_from_this`.
- [ ] **Class-as-node pattern** — `class NavGoalClient : public rclcpp::Node`,
      members initialized in the ctor, RAII ownership of client_/timer_.
- [ ] **Parameters** — `declare_parameter` + `get_parameter(...).as_double()`;
      typed access; passing `-p goal_x:=…` on the CLI.
- [ ] **`<cmath>` + quaternion-from-yaw** — `std::sin(yaw/2)`, `std::cos(yaw/2)`;
      why yaw→quaternion, and why `<cmath>` must be included (it was, deliberately).
- [ ] **Logging macros** — `RCLCPP_INFO/WARN/ERROR(get_logger(), fmt, …)`;
      printf-style formatting.

### 14d. Extensions to implement next (study + do)

- [ ] **`async_cancel_goal(handle)` — the cancel path (Milestone 2 + Phase 6).**
      Currently Ctrl-C only calls `rclcpp::shutdown()`, so the *robot keeps driving*
      even though the client exits. Wire a real SIGINT handler that calls
      `async_cancel_goal`, waits for the `CANCELED` result, *then* shuts down. This
      is the exact primitive Phase 6 reuses for **cancel-on-cat-sighting**.
- [ ] **Store the `GoalHandle`** returned via the goal-response future so you can
      cancel or query it later (needed for cancel + for goal sequencing).
- [ ] **Callback groups / executors** — when Phase 4 chains multiple goals
      (waypoint sequencer / FSM), how to avoid deadlocks: reentrant vs mutually-
      exclusive callback groups, `MultiThreadedExecutor`.
- [ ] **`NavigateThroughPoses`** — the multi-pose action (vs single
      `NavigateToPose`); how the client code changes for a pose *list*.
- [ ] **Result/feedback message fields** — beyond `distance_remaining`:
      `navigation_time`, `number_of_recoveries`; using them for patrol telemetry.

### 14e. Where to read the reference source (from §9)

- [ ] `rclcpp_action/include/rclcpp_action/client.hpp` — `Client<>`,
      `SendGoalOptions`, `async_send_goal`, `async_cancel_goal`.
- [ ] `rclcpp_action/include/rclcpp_action/client_goal_handle.hpp` —
      `WrappedResult`, `ResultCode`.
- [ ] `nav2_msgs/action/NavigateToPose.action` — Goal/Result/Feedback fields.
- [ ] `nav2_bt_navigator/src/bt_navigator.cpp` — the **server** side: how the
      action is hosted and how the behavior tree drives planner + controller.
