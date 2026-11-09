# Phase 4 — Patrol Behavior (Nav2-native) Status & Reference

Companion to [`plan.md`](plan.md), [`phase3-status.md`](phase3-status.md),
[`phase2-status.md`](phase2-status.md), [`architecture.md`](architecture.md),
and [`hardware.md`](hardware.md).

Phase 4 = build a **Nav2-native patrol manager** that wakes on schedule,
drives through recorded waypoints, captures photos at each waypoint, returns
home, sends one email bundle, and goes back to sleep.

> **STATUS: PHASE 4 REACHED (2026-07-17, Session 4).**
> All required acceptance criteria (§2) are now confirmed live: full waypoint
> cycles complete reliably (strafe + NavfnPlanner + vy-feedback fix, proven
> Session 3), per-waypoint capture/mail works, home-return works, the stall
> guard is validated, and — the last open item — continuous unattended
> looping (`loop_patrol=true`, 10s pause between cycles, repeating until the
> `t5_patrol.sh` process is stopped) is now confirmed working live. See
> §11 Session 4 for the looping fix and §11.5/Sessions 1–3 for the full
> diagnostic trail behind the earlier reliability fixes. Remaining items are
> stretch/nice-to-have (§2) and forward links into Phase 5/6/7 (§12).

---

## 1. Purpose and scope

**Goal.** Replace open-loop patrol driving with a Nav2-action-driven manager:

- `SLEEPING → WAKING → PATROLLING(waypoint i) → CAPTURE(i) → … → RETURNING → SLEEPING`
- Keep a hook for `INVESTIGATING` (Phase 6).

**Why this phase matters (from `plan.md`).**
- This is the project’s core C++ architecture phase:
  FSM + action callbacks + callback-group concurrency.
- It reuses Phase 3’s `NavigateToPose` action client pattern from
  `src/nav_goal_client_node.cpp`.
- It carries forward the Strategy-style design from
  `include/cat_patrol_robot/patterns/patrol_pattern.hpp`, but now the
  motion primitive is Nav2 goals (not raw `/cmd_vel`).

**What changes vs Phase 3.**
- Phase 3 proved single-goal navigation; Phase 4 sequences many goals.
- Phase 4 adds scheduling (`patrol_period_sec`) and cycle orchestration.
- Photo capture at each waypoint becomes part of the patrol pipeline.
- Return-home becomes explicit Nav2 goal to recorded `home` pose.

**Scope.**
- One map (`living_room_v1`).
- 4–6 waypoints + 1 home pose.
- Single-room unattended patrol cycle.
- Wake-up scheduling by ROS timer only (systemd timer integration deferred to
  Phase 7 as planned).

---

## 2. Acceptance criteria

Phase 4 is done when all required items pass:

### Required
- [x] `patrol_manager` package builds and launches cleanly.
- [x] Node reaches `SLEEPING` after startup and wakes itself by timer.
- [x] On wake, it sends Nav2 goals for all configured waypoints in order.
- [x] At each waypoint, one photo is saved to disk and path appended to cycle list.
- [x] After last waypoint, robot navigates to home pose.
- [x] After arriving home, one mail request is published per captured
      waypoint photo (design changed from one bundled mail to per-waypoint
      mail per explicit user request, Session 2 — see §11 Session 2; intent
      of the criterion — photos reliably emailed — is met).
- [x] FSM returns to `SLEEPING` and repeats next cycle (if `loop_patrol=true`)
      — confirmed live 2026-07-17 (Session 4): continuous looping with a 10s
      pause between cycles, running until `t5_patrol.sh` is stopped.
- [x] No raw `/cmd_vel` publishes from `patrol_manager` (Nav2-only movement).
- [x] Executor remains responsive during active navigation (no callback starvation).

### Nice-to-have (stretch)
- [ ] Publish patrol telemetry (current waypoint index, distance remaining,
      recovery count if available).
- [ ] Add `NavigateThroughPoses` variant for A/B test vs sequential goals.
- [ ] Add a tiny waypoint recorder helper node for faster map-pose capture.

---

## 3. What we need to create

| File | Purpose |
|---|---|
| `../patrol_manager/config/patrol_manager_params.yaml` | Phase 4 runtime params (schedule, topics, waypoints, home pose) |
| `../patrol_manager/launch/patrol_manager.launch.py` | Brings up patrol manager with params |
| `../patrol_manager/include/patrol_manager/patrol_strategy.hpp` | Strategy interface for patrol behaviors |
| `../patrol_manager/include/patrol_manager/waypoint_patrol_strategy.hpp` | Waypoint implementation strategy |
| `../patrol_manager/src/waypoint_patrol_strategy.cpp` | Strategy logic scaffold |
| `../patrol_manager/src/patrol_manager_node.cpp` | Main FSM + Nav2 action client + capture + mail trigger |
| `../patrol_manager/CMakeLists.txt` + `../patrol_manager/package.xml` | Build wiring and dependencies |
| `~/myscripts2/t5_patrol.sh` | Convenience launch wrapper (mirrors t1/t2/t2.5/t3/t4) |

Notes:
- Package location is workspace-level: `yahboomcar_ws/src/patrol_manager`.
- Reuse references from existing code:
  - Action client pattern: `cat_patrol_robot/src/nav_goal_client_node.cpp`
  - JPEG conversion/save logic: `cat_patrol_robot/src/patrol_node.cpp`
  - Strategy-style organization: `cat_patrol_robot/include/cat_patrol_robot/patterns/patrol_pattern.hpp`

---

## 4. Files to create (before first session)

### 4a. Create package scaffold

```bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src
ros2 pkg create patrol_manager \
  --build-type ament_cmake \
  --dependencies rclcpp rclcpp_action nav2_msgs geometry_msgs sensor_msgs std_msgs

mkdir -p /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/include/patrol_manager
mkdir -p /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/src
mkdir -p /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/config
mkdir -p /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/launch
```

### 4b. `../patrol_manager/config/patrol_manager_params.yaml`

This keeps **all Phase 4 behavior** in one editable YAML file,
including waypoint data (no extra parser dependency needed).

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/config/patrol_manager_params.yaml << 'EOF'
patrol_manager:
  ros__parameters:
    # --- Scheduling ---
    start_on_boot: true
    loop_patrol: true
    patrol_period_sec: 1800.0

    # --- Navigation action ---
    navigate_action_name: "navigate_to_pose"
    nav_goal_timeout_sec: 180.0
    settle_after_arrival_sec: 1.0

    # --- Topics ---
    image_topic: "/camera/color/image_raw"
    mail_request_topic: "/cat_patrol/mail_request"
    state_topic: "/patrol_manager/state"

    # --- Capture + mail ---
    image_save_dir: "/tmp/cat_patrol_images"
    mail_subject: "Cat patrol photos"
    smtp_to_address: "user@example.com"

    # --- Frame assumptions ---
    map_frame: "map"

    # --- Home pose in map frame (x, y, yaw radians) ---
    home_pose: [0.0, 0.0, 0.0]

    # --- Waypoints in map frame ---
    # Keep all 3 arrays same length.
    waypoint_names: ["wp1", "wp2", "wp3", "wp4"]
    waypoint_x: [ 1.20,  2.00,  1.80,  0.80]
    waypoint_y: [-0.40,  0.30,  1.10,  0.90]
    waypoint_yaw: [0.00, 1.57, 3.14, -1.57]
EOF
```

> Replace the sample pose values with your real room waypoints before Session 1.

### 4c. `../patrol_manager/launch/patrol_manager.launch.py`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/launch/patrol_manager.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/config/patrol_manager_params.yaml',
        description='Full path to patrol manager params YAML',
    )

    patrol_node = Node(
        package='patrol_manager',
        executable='patrol_manager_node',
        name='patrol_manager',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        patrol_node,
    ])
EOF
```

### 4d. `../patrol_manager/include/patrol_manager/patrol_strategy.hpp`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/include/patrol_manager/patrol_strategy.hpp << 'EOF'
#ifndef PATROL_MANAGER__PATROL_STRATEGY_HPP_
#define PATROL_MANAGER__PATROL_STRATEGY_HPP_

#include <string>
#include <vector>

namespace patrol_manager
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string name;
};

class PatrolStrategy
{
public:
  virtual ~PatrolStrategy() = default;
  virtual std::string name() const = 0;
  virtual void reset() = 0;
  virtual bool has_next() const = 0;
  virtual Pose2D next() = 0;
};

}  // namespace patrol_manager

#endif  // PATROL_MANAGER__PATROL_STRATEGY_HPP_
EOF
```

### 4e. `../patrol_manager/include/patrol_manager/waypoint_patrol_strategy.hpp`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/include/patrol_manager/waypoint_patrol_strategy.hpp << 'EOF'
#ifndef PATROL_MANAGER__WAYPOINT_PATROL_STRATEGY_HPP_
#define PATROL_MANAGER__WAYPOINT_PATROL_STRATEGY_HPP_

#include <cstddef>
#include <vector>

#include "patrol_manager/patrol_strategy.hpp"

namespace patrol_manager
{

class WaypointPatrolStrategy : public PatrolStrategy
{
public:
  explicit WaypointPatrolStrategy(std::vector<Pose2D> waypoints);

  std::string name() const override;
  void reset() override;
  bool has_next() const override;
  Pose2D next() override;

private:
  std::vector<Pose2D> waypoints_;
  std::size_t index_{0};
};

}  // namespace patrol_manager

#endif  // PATROL_MANAGER__WAYPOINT_PATROL_STRATEGY_HPP_
EOF
```

### 4f. `../patrol_manager/src/waypoint_patrol_strategy.cpp`

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/src/waypoint_patrol_strategy.cpp << 'EOF'
#include "patrol_manager/waypoint_patrol_strategy.hpp"

#include <stdexcept>

namespace patrol_manager
{

WaypointPatrolStrategy::WaypointPatrolStrategy(std::vector<Pose2D> waypoints)
: waypoints_(std::move(waypoints))
{}

std::string WaypointPatrolStrategy::name() const
{
  return "waypoint_patrol";
}

void WaypointPatrolStrategy::reset()
{
  index_ = 0;
}

bool WaypointPatrolStrategy::has_next() const
{
  return index_ < waypoints_.size();
}

Pose2D WaypointPatrolStrategy::next()
{
  if (!has_next()) {
    throw std::runtime_error("WaypointPatrolStrategy::next called with no remaining waypoints");
  }
  return waypoints_[index_++];
}

}  // namespace patrol_manager
EOF
```

### 4g. `../patrol_manager/src/patrol_manager_node.cpp` (starter implementation)

This is a **Phase 4 starter**, intentionally compact. Build it, run one cycle,
then harden incrementally in Session 1–2.

```bash
cat > /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/src/patrol_manager_node.cpp << 'EOF'
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include "patrol_manager/waypoint_patrol_strategy.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

namespace patrol_manager
{

enum class State
{
  SLEEPING,
  WAKING,
  PATROLLING,
  CAPTURE,
  RETURNING,
  INVESTIGATING,
};

class PatrolManagerNode : public rclcpp::Node
{
public:
  PatrolManagerNode()
  : rclcpp::Node("patrol_manager")
  {
    // Parameters
    start_on_boot_ = declare_parameter<bool>("start_on_boot", true);
    loop_patrol_ = declare_parameter<bool>("loop_patrol", true);
    patrol_period_sec_ = declare_parameter<double>("patrol_period_sec", 1800.0);
    settle_after_arrival_sec_ = declare_parameter<double>("settle_after_arrival_sec", 1.0);

    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    mail_request_topic_ = declare_parameter<std::string>("mail_request_topic", "/cat_patrol/mail_request");
    state_topic_ = declare_parameter<std::string>("state_topic", "/patrol_manager/state");
    image_save_dir_ = declare_parameter<std::string>("image_save_dir", "/tmp/cat_patrol_images");
    mail_subject_ = declare_parameter<std::string>("mail_subject", "Cat patrol photos");
    mail_to_ = declare_parameter<std::string>("smtp_to_address", "user@example.com");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    const auto home_pose = declare_parameter<std::vector<double>>("home_pose", std::vector<double>{0.0, 0.0, 0.0});
    if (home_pose.size() == 3) {
      home_.x = home_pose[0];
      home_.y = home_pose[1];
      home_.yaw = home_pose[2];
      home_.name = "home";
    }

    const auto names = declare_parameter<std::vector<std::string>>("waypoint_names", std::vector<std::string>{});
    const auto xs = declare_parameter<std::vector<double>>("waypoint_x", std::vector<double>{});
    const auto ys = declare_parameter<std::vector<double>>("waypoint_y", std::vector<double>{});
    const auto yaws = declare_parameter<std::vector<double>>("waypoint_yaw", std::vector<double>{});

    if (!(names.size() == xs.size() && xs.size() == ys.size() && ys.size() == yaws.size())) {
      throw std::runtime_error("waypoint arrays must have equal sizes");
    }
    std::vector<Pose2D> waypoints;
    waypoints.reserve(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
      waypoints.push_back(Pose2D{xs[i], ys[i], yaws[i], names[i]});
    }
    strategy_ = std::make_unique<WaypointPatrolStrategy>(std::move(waypoints));

    std::filesystem::create_directories(image_save_dir_);

    // Callback groups
    fsm_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    io_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // I/O
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = io_group_;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr msg) { last_image_ = msg; },
      sub_opts);

    mail_pub_ = create_publisher<std_msgs::msg::String>(mail_request_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    // NOTE: the action client shares the MutuallyExclusive fsm_group_ so that
    // goal/feedback/result callbacks are serialized with the FSM tick. Without
    // this, result callbacks land in the node default group and race the tick
    // on state_/strategy_/cycle_image_paths_ under MultiThreadedExecutor.
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose", fsm_group_);

    // Timers
    fsm_timer_ = create_wall_timer(100ms, std::bind(&PatrolManagerNode::tick, this), fsm_group_);
    wake_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(patrol_period_sec_)),
      [this]() { wake_requested_ = true; },
      fsm_group_);

    state_ = start_on_boot_ ? State::WAKING : State::SLEEPING;
    publish_state();

    RCLCPP_INFO(get_logger(), "PatrolManager started. strategy=%s waypoints=%zu",
                strategy_->name().c_str(), names.size());
  }

private:
  static geometry_msgs::msg::PoseStamped to_pose(const Pose2D & p, const std::string & frame, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame;
    out.header.stamp = stamp;
    out.pose.position.x = p.x;
    out.pose.position.y = p.y;
    out.pose.orientation.z = std::sin(p.yaw / 2.0);
    out.pose.orientation.w = std::cos(p.yaw / 2.0);
    return out;
  }

  void tick()
  {
    switch (state_) {
      case State::SLEEPING:
        if (wake_requested_) {
          wake_requested_ = false;
          transition(State::WAKING);
        }
        break;

      case State::WAKING:
        cycle_image_paths_.clear();
        strategy_->reset();
        // Non-blocking readiness check — never block inside the FSM timer (see §9).
        // Stay in WAKING and re-check next tick until the server is up.
        if (!action_client_->action_server_is_ready()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "navigate_to_pose action server unavailable; staying in WAKING");
          break;
        }
        send_next_waypoint_goal();
        break;

      case State::PATROLLING:
        // Waiting for action result callback to transition.
        break;

      case State::CAPTURE:
        if (save_current_image()) {
          if (strategy_->has_next()) {
            send_next_waypoint_goal();
          } else {
            transition(State::RETURNING);
            send_home_goal();
          }
        }
        break;

      case State::RETURNING:
        // Waiting for home-goal result callback.
        break;

      case State::INVESTIGATING:
        // Reserved for Phase 6.
        break;
    }
  }

  void send_next_waypoint_goal()
  {
    if (!strategy_->has_next()) {
      transition(State::RETURNING);
      send_home_goal();
      return;
    }

    const auto wp = strategy_->next();
    auto goal = NavigateToPose::Goal();
    goal.pose = to_pose(wp, map_frame_, now());

    transition(State::PATROLLING);
    RCLCPP_INFO(get_logger(), "Navigating to waypoint '%s' x=%.2f y=%.2f yaw=%.2f",
                wp.name.c_str(), wp.x, wp.y, wp.yaw);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "distance remaining: %.2f m", fb->distance_remaining);
      };

    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          this->transition(State::CAPTURE);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Waypoint goal failed (code=%d)", static_cast<int>(result.code));
          this->transition(State::SLEEPING);
        }
      };

    action_client_->async_send_goal(goal, opts);
  }

  void send_home_goal()
  {
    auto goal = NavigateToPose::Goal();
    goal.pose = to_pose(home_, map_frame_, now());

    RCLCPP_INFO(get_logger(), "Returning home x=%.2f y=%.2f yaw=%.2f", home_.x, home_.y, home_.yaw);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          send_mail_request();
          transition(State::SLEEPING);
          if (!loop_patrol_) {
            RCLCPP_INFO(get_logger(), "loop_patrol=false, cycle complete");
          }
        } else {
          RCLCPP_ERROR(get_logger(), "Home goal failed (code=%d)", static_cast<int>(result.code));
          transition(State::SLEEPING);
        }
      };

    action_client_->async_send_goal(goal, opts);
  }

  bool save_current_image()
  {
    if (!last_image_ || last_image_->data.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No image available yet");
      return false;
    }

    try {
      // Manual sensor_msgs::Image -> cv::Mat conversion (same approach as
      // patrol_node.cpp). Avoids a cv_bridge dependency/ABI risk on this build.
      const std::string & enc = last_image_->encoding;
      const int rows = static_cast<int>(last_image_->height);
      const int cols = static_cast<int>(last_image_->width);
      const int step = static_cast<int>(last_image_->step);
      cv::Mat bgr_image;
      if (enc == "rgb8") {
        cv::Mat rgb(rows, cols, CV_8UC3, const_cast<uint8_t*>(last_image_->data.data()), step);
        cv::cvtColor(rgb, bgr_image, cv::COLOR_RGB2BGR);
      } else if (enc == "bgr8") {
        cv::Mat src(rows, cols, CV_8UC3, const_cast<uint8_t*>(last_image_->data.data()), step);
        bgr_image = src.clone();
      } else if (enc == "mono8") {
        cv::Mat mono(rows, cols, CV_8UC1, const_cast<uint8_t*>(last_image_->data.data()), step);
        cv::cvtColor(mono, bgr_image, cv::COLOR_GRAY2BGR);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "Unsupported encoding '%s'", enc.c_str());
        return false;
      }
      if (bgr_image.empty()) {
        return false;
      }
      std::ostringstream fn;
      fn << "snap_" << now().nanoseconds() << ".jpg";
      const auto fp = (std::filesystem::path(image_save_dir_) / fn.str()).string();
      if (cv::imwrite(fp, bgr_image)) {
        cycle_image_paths_.push_back(fp);
        RCLCPP_INFO(get_logger(), "Captured: %s", fp.c_str());
        return true;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "save_current_image exception: %s", e.what());
    }
    return false;
  }

  void send_mail_request()
  {
    std::ostringstream oss;
    oss << "{\"subject\":\"" << mail_subject_ << "\",";
    oss << "\"to\":\"" << mail_to_ << "\",";
    oss << "\"paths\":[";
    for (size_t i = 0; i < cycle_image_paths_.size(); ++i) {
      if (i > 0) {oss << ",";}
      oss << "\"" << cycle_image_paths_[i] << "\"";
    }
    oss << "]}";

    std_msgs::msg::String msg;
    msg.data = oss.str();
    mail_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published mail request with %zu images", cycle_image_paths_.size());
  }

  void transition(State s)
  {
    state_ = s;
    publish_state();
  }

  void publish_state()
  {
    std_msgs::msg::String s;
    switch (state_) {
      case State::SLEEPING: s.data = "SLEEPING"; break;
      case State::WAKING: s.data = "WAKING"; break;
      case State::PATROLLING: s.data = "PATROLLING"; break;
      case State::CAPTURE: s.data = "CAPTURE"; break;
      case State::RETURNING: s.data = "RETURNING"; break;
      case State::INVESTIGATING: s.data = "INVESTIGATING"; break;
    }
    state_pub_->publish(s);
  }

private:
  // Parameters / config
  bool start_on_boot_{true};
  bool loop_patrol_{true};
  double patrol_period_sec_{1800.0};
  double settle_after_arrival_sec_{1.0};
  std::string image_topic_;
  std::string mail_request_topic_;
  std::string state_topic_;
  std::string image_save_dir_;
  std::string mail_subject_;
  std::string mail_to_;
  std::string map_frame_;

  // State
  State state_{State::SLEEPING};
  bool wake_requested_{false};
  Pose2D home_;
  std::vector<std::string> cycle_image_paths_;

  // Concurrency
  rclcpp::CallbackGroup::SharedPtr fsm_group_;
  rclcpp::CallbackGroup::SharedPtr io_group_;

  // ROS entities
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr wake_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mail_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  sensor_msgs::msg::Image::SharedPtr last_image_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  // Strategy
  std::unique_ptr<PatrolStrategy> strategy_;
};

}  // namespace patrol_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<patrol_manager::PatrolManagerNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
EOF
```

### 4h. CMakeLists + package.xml (new package)

`../patrol_manager/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(patrol_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

include_directories(include)

add_library(waypoint_patrol_strategy src/waypoint_patrol_strategy.cpp)
ament_target_dependencies(waypoint_patrol_strategy rclcpp)

add_executable(patrol_manager_node src/patrol_manager_node.cpp)
target_link_libraries(patrol_manager_node waypoint_patrol_strategy ${OpenCV_LIBS})
ament_target_dependencies(patrol_manager_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs sensor_msgs std_msgs
)

install(TARGETS
  waypoint_patrol_strategy
  patrol_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
```

`../patrol_manager/package.xml` key dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>patrol_manager</name>
  <version>0.0.0</version>
  <description>Nav2-native patrol manager for waypoint cycles, capture, and return-home.</description>
  <maintainer email="jetson@todo.todo">jetson</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>nav2_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>libopencv-dev</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```



### 4i. `~/myscripts2/t5_patrol.sh`

```bash
cat > ~/myscripts2/t5_patrol.sh << 'EOF'
#!/usr/bin/env bash
export ROS_DOMAIN_ID=28
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds.xml
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash
ros2 launch patrol_manager patrol_manager.launch.py
EOF
chmod +x ~/myscripts2/t5_patrol.sh
```

### 4j. Build

```bash
export ROS_DOMAIN_ID=28
source /opt/ros/humble/setup.bash
cd /home/jetson/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select patrol_manager
source install/setup.bash

# verify
ros2 pkg executables patrol_manager | grep patrol_manager_node
```

---

## 5. Architecture

```
Jetson
──────────────────────────────────────────────────────────────────────────
t1.sh  yahboomcar_bringup (driver, EKF, robot_state_publisher, joy safety)
t2.sh  lidar
t2.5.sh scan_front_filter     /scan -> /scan_filtered
t3_amcl.sh localization       map_server + amcl + lifecycle_manager_localization
t4_nav.sh  navigation         planner/controller/behaviors/bt_navigator

NEW (Phase 4)
t5_patrol.sh patrol_manager
  ├─ FSM: SLEEPING / WAKING / PATROLLING / CAPTURE / RETURNING
  ├─ Strategy: WaypointPatrolStrategy (ordered map-frame poses)
  ├─ Action client -> /navigate_to_pose (Nav2 bt_navigator)
  ├─ Sub /camera/color/image_raw
  ├─ Save JPEGs -> /tmp/cat_patrol_images
  ├─ Pub /cat_patrol/mail_request (JSON with attachment paths)
  └─ Pub /patrol_manager/state (operator observability)

Action flow:
  patrol_manager --goal--> bt_navigator
                <--feedback--
                <--result----
```

---

## 6. Pre-flight checklist

### 6a. One-time before first session

- [ ] Build `patrol_manager` and verify executable appears.
- [ ] Confirm launch args:
      `ros2 launch patrol_manager patrol_manager.launch.py --show-args`
- [ ] Fill real waypoint/home values in `patrol_manager_params.yaml`.
- [ ] Verify camera topic live:
      `ros2 topic hz /camera/color/image_raw`
- [ ] Verify mail node pipeline (already used by existing package) is active.

### 6b. Every session

- [ ] Battery above your safe threshold before autonomous cycle.
- [ ] Environment clear enough for planned waypoints.
- [ ] Bring up `t1` + `t2` + `t2.5` + `t3` + `t4` first.
- [ ] Confirm localization converged before starting patrol.
- [ ] Keep joystick ready for safety override.

---

## 7. Session procedure

### Step A — Jetson terminals (in order)

1. Bringup:
   `bash ~/myscripts2/t1.sh`
2. LiDAR:
   `bash ~/myscripts2/t2.sh`
3. Scan filter:
   `bash ~/myscripts2/t2.5.sh`
4. Localization:
   `bash ~/myscripts2/t3_amcl.sh`
5. Navigation:
   `bash ~/myscripts2/t4_nav.sh`
6. Patrol manager:
   `bash ~/myscripts2/t5_patrol.sh`

### Step B — RViz on host

- Show: Map, TF, LaserScan, Global/Local costmaps, Path, RobotModel.
- Ensure AMCL pose is reasonable before patrol begins.

### Step C — Validate patrol manager state machine

- Watch state topic:
  `ros2 topic echo /patrol_manager/state`
- Expected transitions for one full cycle:
  `SLEEPING -> WAKING -> PATROLLING -> CAPTURE -> ... -> RETURNING -> SLEEPING`

### Step D — Verify one full cycle

- Confirm each waypoint produces one `goal succeeded` in patrol logs.
- Confirm one photo file appears per waypoint in `image_save_dir`.
- Confirm one mail request message is published at cycle end.
- Confirm robot returns to configured home pose and stops.

### Step E — Verify unattended repeat

- Wait one full `patrol_period_sec` interval.
- Confirm automatic wake and second cycle start.

---

## 8. Pitfalls and recovery

- **Action server unavailable at wake**
  - Symptom: repeated “navigate_to_pose action server unavailable”.
  - Fix: start `t4_nav.sh` first; verify Nav2 lifecycle active.

- **No images captured**
  - Symptom: CAPTURE state loops with “No image available yet”.
  - Fix: verify camera topic name and frame rate; check Astra node is alive.

- **Waypoint arrays mismatch**
  - Symptom: node throws on startup.
  - Fix: keep `waypoint_names/x/y/yaw` equal length.

- **Robot cannot reach a waypoint**
  - Symptom: goal aborted repeatedly.
  - Fix: move waypoint to free space in map; check costmaps and map alignment.

- **Callback starvation / laggy state updates**
  - Symptom: delayed state changes while nav feedback is busy.
  - Fix: keep FSM timer + action result callback in the MutuallyExclusive group,
    image sub in Reentrant, run `MultiThreadedExecutor(2+)`.

- **Regression to old chassis behavior**
  - Symptom: angular.z behaves like strafe (or vice versa).
  - Fix: re-check hardware wheel orientation and baseline bringup behavior from
    Phase 3 resolution notes.

---

## 9. Concept anchors (Phase 4 learning focus)

- **FSM as orchestration layer**
  - The FSM does not drive motors directly.
  - It decides *what phase should run next*.

- **Action callbacks as event stream**
  - Goal response/feedback/result become asynchronous events.
  - Avoid blocking waits inside timer callbacks.

- **Callback groups + executor**
  - `MutuallyExclusive` (`fsm_group_`): FSM tick **and** action goal/feedback/result
    callbacks. Because the result callback mutates FSM state (`state_`, `strategy_`,
    `cycle_image_paths_`), it must be serialized with the tick — not Reentrant.
  - `Reentrant` (`io_group_`): image subscriber only.
  - `MultiThreadedExecutor`: allows both groups to progress concurrently.

- **Strategy pattern reuse**
  - `PatrolStrategy` keeps room for future patrol styles:
    random walk, perimeter sweep, threat-driven routing.
  - `WaypointPatrolStrategy` is the first concrete strategy.

---

## 10. Commands cheat-sheet

```bash
# Build only patrol_manager
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --symlink-install --packages-select patrol_manager
source install/setup.bash

# Run patrol manager directly
ros2 run patrol_manager patrol_manager_node --ros-args \
  --params-file ~/yahboomcar_ros2_ws/yahboomcar_ws/src/patrol_manager/config/patrol_manager_params.yaml

# Run launch wrapper
ros2 launch patrol_manager patrol_manager.launch.py

# Observe state transitions
ros2 topic echo /patrol_manager/state

# Observe mail payload publication
ros2 topic echo /cat_patrol/mail_request

# Check active action server
ros2 action list | grep navigate_to_pose

# Check camera stream
ros2 topic hz /camera/color/image_raw

# Check created photos
ls -lt /tmp/cat_patrol_images | head
```

---

## 11. Phase 4 session log

### Session 1 — 2026-07-09

**Accomplished this session**
- Applied review fixes #1–#4 to this doc's code blocks (action client in
  `fsm_group_`; non-blocking `action_server_is_ready()`; manual `cv::Mat`
  conversion instead of `cv_bridge`; prose reconciled).
- Created the `patrol_manager` package at workspace level
  (`yahboomcar_ws/src/patrol_manager`) with the fixed code. Builds clean;
  `ros2 pkg executables patrol_manager` → `patrol_manager_node`. **No rebuild
  needed to change params** (launch loads the src YAML by absolute path).
- New helper scripts in `~/myscripts2/`:
  - `t5_patrol.sh`  — launch patrol_manager
  - `t6_camera.sh`  — Astra color+depth ONLY (sources `software/library_ws`;
    `uvc_product_id:=0x050f`). The staged t1–t4 do NOT start the camera.
  - `t7_mail.sh`    — `mail_node` standalone (inherits `CAT_PATROL_SMTP_*`
    from `~/.bashrc`: user `peter.botnet@gmail.com`, host smtp.gmail.com).
  - `getpose.sh`    — prints map→base_footprint as `x / y / yaw(rad)`.

**Map situation (IMPORTANT — resolved, not a mismatch)**
- The original `living_room_v1` map was wrong; remapped on the host
  (slam_toolbox in Humble Docker) → `my_map.pgm/.yaml`.
- The new map was installed on the bot by **overwriting the `living_room_v1`
  name** (kept the filename so no config change). Confirmed: `maps/living_room_v1.pgm`
  is 45655 bytes, mtime 2026-07-09 06:30 == host `my_map.pgm` size. `amcl_params.yaml`
  `yaml_filename` still points at `living_room_v1.yaml` and correctly serves the
  NEW map. **Waypoints below are valid against the loaded map.**
- Earlier "map appears 2 s then goes grey" was two `/map` publishers — a
  slam_toolbox instance still running alongside map_server. Fixed by stopping slam.

**Waypoints configured:** 10 + home, captured live by driving on the new map
(written to `patrol_manager/config/patrol_manager_params.yaml`):
| wp | x | y | yaw |
|----|------|------|------|
| 1 | 1.585 | -0.120 | -0.115 |
| 2 | 3.151 | -0.511 | -0.272 |
| 3 | 5.133 | -0.672 | 0.135 |
| 4 | 6.402 | -0.476 | 0.150 |
| 5 | 6.478 | -1.132 | 2.904 |
| 6 | 3.644 | -0.392 | 3.036 |
| 7 | 3.183 | 1.502 | 1.855 |
| 8 | 2.987 | 2.178 | 2.992 |
| 9 | 3.344 | 0.709 | -1.312 |
| 10 | 1.954 | -0.277 | -2.966 |
| home | 0.331 | -0.086 | 0.043 |

First-run config: `loop_patrol=false`, `smtp_to_address=peter.botnet@gmail.com`,
`start_on_boot=true`.

**Full cycle result:** ❌ BLOCKED — Nav2 goal for **wp5** failed (aborted).
- Robot ended at **x=6.766, y=-1.405, yaw=3.074** — *past* wp5, jammed into the
  far corner. wp4 (6.402,-0.476) and wp5 (6.478,-1.132) are both at the far
  end (x≈6.4–6.5); wp5 also demands a ~166° turnaround (yaw 2.904) hard against
  the far wall.
- **Photos captured:** wp1–wp4 reached and should have saved to
  `/tmp/cat_patrol_images` (verify count next session). Not confirmed this session.
- **Home return:** did NOT happen — on a waypoint-goal failure the current FSM
  goes straight to `SLEEPING` and abandons the cycle (no home, no email).

**Prime suspects for the wp5 abort (investigate next session, in order)**
1. **wp5 too close to the far wall / inside costmap inflation** → planner or
   controller can't reach goal tolerance. Most likely.
2. **Tight turnaround (166°) in a confined corner** → recoveries exhausted → abort.
3. Global/local costmap or inflation radius too aggressive for that corner.

**Resume plan for next session**
1. Bring up t1→t4 + t6 (camera) + t7 (mail). Confirm map is steady (single
   `/map` publisher — `ros2 topic info /map -v`), AMCL converged.
2. Inspect the abort: rerun and watch `t4_nav` logs + RViz global/local costmap
   at wp5. Note the exact result code the patrol logs print
   ("Waypoint goal failed (code=…)": 4=ABORTED, 5=CANCELED).
3. Fix options (pick based on what the costmap shows):
   - **Re-capture wp5** ~30–40 cm further from the wall and/or with a friendlier
     orientation (drive there, `getpose.sh`, edit the arrays).
   - Send wp5 goal by clicking it in RViz first to confirm Nav2 can reach it
     standalone before blaming patrol_manager.
   - Loosen `xy_goal_tolerance` / `yaw_goal_tolerance` in `nav2_params.yaml`, or
     reduce inflation near that corner.
4. **Code hardening to consider (from §14d):** on a waypoint failure, instead of
   abandoning the whole cycle (→SLEEPING), either (a) retry the goal N times, or
   (b) skip the failed waypoint and continue. Currently one bad goal kills the
   run and skips home+email. This is the biggest robustness gap.

**Notes:** Everything up to wp5 worked — package, FSM, Nav2 action client,
per-waypoint navigation, and state transitions are all functioning. The blocker
is a single unreachable/hard waypoint plus the abandon-on-failure policy, not a
structural problem.

### Session 2 — 2026-07-10

**Static-map analysis of the wp5 abort (no robot movement, done offline).**
- Parsed `living_room_v1.pgm`/`.yaml` directly. wp5 (6.478, -1.132) sits in
  mapped **free** space; the nearest mapped wall is ~1.3 m east (not literally
  against a wall as first suspected). The robot's final resting spot
  (6.766, -1.405) is also mapped-free, ~0.4 m SE of wp5 — consistent with it
  getting stuck/drifting mid-rotation or mid-recovery, not colliding with a
  mapped wall.
- wp5's commanded yaw (2.904 rad ≈ 166°) is a near-total reversal right as the
  robot arrives in that corner — prime suspect for triggering MPPI/recovery
  churn, possibly compounded by a live obstacle (`/scan_filtered`) not present
  in the static map. **Static map alone can't confirm this** — needs a live
  RViz costmap watch during the approach (see original resume plan step 2,
  still open).

**Code hardening applied (§14d item, "biggest robustness gap"):** in
`patrol_manager_node.cpp`, a waypoint goal failure no longer abandons the
whole cycle to `SLEEPING`. The result callback now logs the failed waypoint
name/code and calls `send_next_waypoint_goal()` directly, which either sends
the next waypoint or — if none remain — transitions to `RETURNING` and sends
the home goal. So one bad waypoint is now skipped (no photo for it) instead
of losing home-return + the mail request for the whole cycle. Home-goal
failure behavior is unchanged (still → `SLEEPING`, no mail, since the robot
never actually arrived home). Rebuilt clean:
`colcon build --symlink-install --packages-select patrol_manager` — 23.5s, no errors.

**Not yet done (needs a live session with operator present):**
- Recapture wp5 pulled back ~30–40 cm from the corner and/or reorder its yaw
  so it isn't a ~166° in-place turn in the tightest part of the corner.
- Before trusting a new wp5 pose in patrol_manager, send a one-off
  `NavigateToPose` goal there via RViz "Nav2 Goal" to confirm Nav2 can reach
  it standalone.
- Re-run a full cycle with `loop_patrol=false` to confirm: wp5 no longer
  aborts, AND (as a fallback) the skip-and-continue fix now gets the robot
  home + one mail even if some other waypoint fails.

**Same-day live run (after a full stack restart):** full cycle completed —
robot returned home, images sent. wp5 was "really difficult" to reach (didn't
abort outright this time, but struggled — consistent with the corner/166°-turn
suspicion above, still not conclusively root-caused). **wp6 was skipped** —
this is the skip-and-continue hardening from earlier today working as
designed (wp6 likely hit a similar Nav2 abort; check the patrol_manager
terminal for the "Waypoint 'wp6' goal failed (code=...)" line next session to
see why). Also investigated a separate live "robot does not move, nav reports
failed points" incident during this session: the local-costmap
"Message Filter dropping" burst turned out to be a one-time startup transient
(stopped after ~15s, absent from the 7/09 log), and the real symptom reported
("very weak twitch then nothing" at wp1) pointed at hardware/power rather than
Nav2 config — not fully diagnosed before the stack was restarted and started
working. **Worth keeping an eye on battery voltage under load** if this
recurs.

**Feature change applied (user request):** photos are no longer bundled into
one email at cycle end. `patrol_manager_node.cpp` now calls a new
`send_waypoint_mail(name, path)` immediately after each successful capture in
`CAPTURE` state (one email per waypoint, subject suffixed with the waypoint
name, single attachment). The old `send_mail_request()` (batched, sent only
after arriving home) was removed per explicit user preference — **no email is
sent on home arrival**, just a log line with the photo count. `current_waypoint_`
member added so the CAPTURE state knows which waypoint's name to label the
mail with. Rebuilt clean.

**wp5/wp6 ROOT CAUSE FOUND (from this run's controller_server + patrol_manager
logs, not guesswork):** wp5 took 165.6s and wp6 took 177.3s to resolve (vs.
~9–16s for wp1–4). `distance remaining` feedback shows the mechanism precisely:
- Early on, it oscillates hard (1.53→1.09→0.46→1.41→0.51→1.76→0.79 m...) — the
  global planner is recomputing a materially different path almost every
  replan cycle, not converging on one route.
- Then it goes dead flat — stuck at exactly `0.30 m` for ~30s, later flat at
  `0.16–0.18 m` for another ~30s. Zero net XY translation during these
  stretches.
- Every ~10s during this, `controller_server` logs `ERROR: Failed to make
  progress` → `[follow_path] Aborting handle` → clears the local costmap →
  BT immediately restarts the goal from scratch. **26 of these abort/retry
  cycles fired across the wp5+wp6 window in this one run.**

**Mechanism:** wp5 (yaw 2.904 ≈166°) and wp6 (yaw 3.036 ≈174°) both demand a
huge final-heading change vs. wp1–4 (≤16°). The robot gets close in XY
(~0.3 m) then has to grind through a large in-place rotation to match the
final orientation — likely moving cautiously because `ObstaclesCritic` is wary
of the nearby wall during the turn. `nav2_controller::SimpleProgressChecker`
(the plugin in use) **only measures XY displacement** — it is blind to
rotation. So while the robot is legitimately working on the turn, the checker
sees "no XY movement in 10s," aborts, and the BT restarts the goal from
scratch — over and over. wp5 eventually won this race by chance; wp6 didn't
and hit `ABORTED` (code=6) for good. This also explains the earlier
"weak twitch then nothing" report — that was this exact stall caught mid-cycle,
not a hardware/power issue as first suspected.

**Fix applied (`cat_patrol_robot/config/nav2_params.yaml`):** switched
`progress_checker_plugin` from `nav2_controller::SimpleProgressChecker` to
`nav2_controller::PoseProgressChecker` (also a stock Nav2 plugin, in the same
`nav2_controller` package — confirmed via `dpkg -L ros-humble-nav2-controller`
and the installed header, not custom code). It extends SimpleProgressChecker
with `required_movement_angle` (set to 0.5 rad ≈29°) — rotating that much
within the time window now also counts as progress, not just XY movement.
Also raised `movement_time_allowance` 10.0 → 20.0s for extra headroom. YAML
config only, **no rebuild — requires a T4 restart** to take effect. **Not yet
tested live** — next session: restart T4, re-run wp5/wp6, confirm no more
abort/retry thrashing.

**SAFETY INCIDENT: blind backup into an unseen object.** During one of the
abort/retry cycles above (or a similar stall), the robot drove backward into
something behind it, got physically stuck, and risked damaging the rear
antennas — with no awareness anything was wrong. Root cause: Nav2's stock
recovery behavior tree (`navigate_to_pose_w_replanning_and_recovery.xml`)
alternates `ClearingActions` and `BackUp backup_dist="0.50" backup_speed="0.10"`
in its `RecoveryFallback` `RoundRobin`. `BackUp` drives the robot backward
using the local costmap for "safety" — but `/scan_filtered` is deliberately
restricted to the front ~220° arc (the fix for the lidar-mount/antenna
map-collapse issue, see [[project_slam_map_collapse]]/earlier sessions), so
the local costmap has **zero knowledge of anything behind the robot**. `BackUp`
can confidently reverse into something it is structurally blind to.

**Fix applied (two-pronged, config only, no rebuild):**
1. New custom BT XML `cat_patrol_robot/config/navigate_to_pose_no_blind_backup.xml`
   — identical to the stock tree except `BackUp` is replaced with
   `Spin spin_dist="1.57"` in the `RoundRobin` recovery fallback. Spin only
   rotates in place using the safely-observed front arc; it can't blindly
   translate into unmapped space. `bt_navigator.default_nav_to_pose_bt_xml` in
   `nav2_params.yaml` now points at this file (absolute path required).
2. Removed `"backup"` from `behavior_server.behavior_plugins` entirely
   (`["spin", "backup", "wait"]` → `["spin", "wait"]`) — defense in depth, so
   no `/backup` action server exists at all and it can't be invoked by any BT
   or by hand, not just the patrol one.

**Correction found on first T4 restart attempt:** bt_navigator failed to
activate — `"backup" action server not available after waiting for 1.00s"` /
`Error loading XML file: navigate_through_poses_w_replanning_and_recovery.xml`.
Cause: bt_navigator loads AND VALIDATES **both** `default_nav_to_pose_bt_xml`
and `default_nav_through_poses_bt_xml` at activation, even though
patrol_manager only ever sends `NavigateToPose` goals, never
`NavigateThroughPoses`. The stock through-poses tree still referenced
`BackUp`, which no longer has an action server, so the whole bt_navigator node
failed to activate — breaking the entire nav stack, not just recovery.
**Fixed:** added a matching
`cat_patrol_robot/config/navigate_through_poses_no_blind_backup.xml` (same
`BackUp`→`Spin` substitution) and pointed
`default_nav_through_poses_bt_xml` at it too. Both XMLs validated
(`xml.etree.ElementTree`) and `nav2_params.yaml` re-validated as YAML.
**Requires a T4 restart. Not yet tested live.**

**NEW CAPABILITY: cmd_vel-vs-vel_raw stall guard, `patrol_manager_node.cpp`.**
This robot has no motor-current/torque sensing and no bump/rear sensor, but
`Mcnamu_driver_X3.py` already publishes `/vel_raw` (actual measured velocity
from wheel encoders) alongside the commanded `/cmd_vel` — enough to detect a
physical stall without new hardware. Added `check_stall()`, called every FSM
tick (100ms) during `PATROLLING`/`RETURNING`: if commanded speed exceeds
`stall_cmd_vel_threshold` (0.03, param) while measured `/vel_raw` speed stays
below `stall_vel_raw_threshold` (0.02, param) continuously for
`stall_timeout_sec` (6.0, param), it's a stall. On detection: cancels the
active `NavigateToPose` goal (`active_goal_handle_`, captured via
`goal_response_callback` in both `send_next_waypoint_goal` and
`send_home_goal`), transitions to `SLEEPING`, and emails a
"STALL near <waypoint>" alert (no attachment) via the same
`/cat_patrol/mail_request` pipe. Both result callbacks now guard on
`state_ == State::SLEEPING` at entry so the async cancel's eventual result
(CANCELED) doesn't resurrect the cycle after the stall guard already stopped
it. New params in `patrol_manager_params.yaml`:
`cmd_vel_topic`/`vel_raw_topic`/`stall_timeout_sec`/`stall_cmd_vel_threshold`/
`stall_vel_raw_threshold`. Rebuilt clean. **Not yet tested live against a real
stall** — next session, worth deliberately checking it doesn't false-trigger
during a legitimate slow final rotation (6s should be short enough for MPPI
creep but comfortably clear of a stall; watch for false positives).

**wp5 STILL not reaching reliably even after the PoseProgressChecker +
no-blind-backup fixes — user identified the deeper architectural cause by
eye:** wp5 is trivially reachable if the robot overshoots past it and
approaches from the far side instead of arriving directly from wp4. Root
cause: `planner_server`'s `GridBased` plugin was `nav2_navfn_planner/NavfnPlanner`
— a pure (x,y) Dijkstra/A* search with **no concept of heading at all**. It
always draws the shortest-ish straight path to a waypoint's *position*, and
the ENTIRE final-orientation problem (wp5's 166°) is left to be resolved by
in-place rotation after arrival — which is exactly what was stalling. MPPI's
own receding horizon (`prune_distance: 1.5`, ~3s) is also far too short to
discover a multi-meter loop-around on its own; this was never something either
stage of the pipeline could find, not a tuning gap.

**Fix applied (`cat_patrol_robot/config/nav2_params.yaml`, `planner_server`):**
switched `GridBased` from `nav2_navfn_planner/NavfnPlanner` to
`nav2_smac_planner/SmacPlannerHybrid` (already installed:
`ros-humble-nav2-smac-planner`) — a Hybrid-A* planner that searches in
**(x, y, heading)** space, so it can natively plan a path that swings past a
waypoint and approaches from the other side already facing the target
heading. Key params: `motion_model_for_search: "DUBIN"` (forward-only curves —
matches the rear-blind-lidar no-reverse constraint), `minimum_turning_radius:
0.10` (robot can rotate in place; this just bounds how tight the analytic
curves are), `angle_quantization_bins: 72` (5° resolution),
`downsample_costmap: true` / `downsampling_factor: 2` (Orin NX is
CPU-constrained — `astra_camera_node` alone runs ~75-85% of a core — this
keeps Hybrid-A* planning latency down). YAML validated. **Config only, no
rebuild, requires a T4 restart. Not yet tested live** — next session: confirm
(a) it actually finds loop-around approaches for wp5/wp6, (b) planning time
stays well under the 1Hz replan budget on this hardware, (c) path quality is
acceptable for MPPI to follow (a `smoother_server` may be worth adding later
if Hybrid-A* paths look jagged — not added yet, keeping this change scoped).

**Live test of SmacPlannerHybrid — confirmed loaded (`ros2 param get` showed
`nav2_smac_planner/SmacPlannerHybrid`, custom BT XMLs, `PoseProgressChecker`,
`behavior_plugins=['spin','wait']` — all prior fixes DID load), but "no
improvement" reported. `planner_server` log showed the actual mechanism:**
```
GridBased: failed to create plan, no valid path found.
Planning algorithm GridBased failed to generate a valid path to (6.48, -1.13)
[compute_path_to_pose] [ActionServer] Aborting handle.
```
Hybrid-A* is now hard-FAILING to find any forward-only (DUBIN) path to wp5 at
all — worse in one sense than NavFn (which always found *some* path). Also:
`Planner loop missed its desired rate of 20.0000 Hz. Current loop rate is
1.17 Hz` (was near-instant with NavFn) — Hybrid-A* is much heavier than NavFn
on this CPU-constrained Orin NX.

**STALL GUARD VALIDATED LIVE (unplanned but confirmed working):** battery
voltage dropped to 11.1V mid-session; the stall guard correctly fired and sent
a stall alert. First real-world confirmation the cmd_vel-vs-vel_raw mechanism
works as designed. Robot needs a recharge before further testing.

**NEW CAPABILITY: waypoint visualization markers, `patrol_manager_node.cpp`.**
Added `publish_waypoint_markers()` — publishes a latched (`transient_local`)
`visualization_msgs/MarkerArray` to `/patrol_manager/waypoints` once at
startup: one ARROW + one TEXT_VIEW_FACING label per waypoint (blue) plus home
(red), built directly from the same `Pose2D` data the FSM navigates to (not a
separate copy that could drift out of sync). Add a MarkerArray display in
RViz on that topic to see waypoints against the costmap/inflation. New
`visualization_msgs` dependency added to `package.xml`/`CMakeLists.txt`.
Rebuilt clean.

**NEW SYMPTOM IDENTIFIED BY EYE (user observation, not from logs): the robot
frequently "lands next to the arrow, not on it"** — same commanded heading as
the waypoint, but positionally offset to the side by some distance — and
closing that gap causes exactly the slow "turmoil" (turning/replanning) seen
in the earlier stalls. **Diagnosis:** `SmacPlannerHybrid` searches on a
discretized (x,y,heading) grid; `downsample_costmap: true` /
`downsampling_factor: 2` (added last session for CPU headroom) doubled the
planning cell size to 10cm. Hybrid-A* is supposed to finish with an exact
"analytic expansion" — a precise Dubin curve straight into the real continuous
goal pose — but when that final connection doesn't cleanly succeed (e.g. near
a tight spot), it falls back to the last quantized grid node instead: correct
heading, but snapped to a coarse cell — i.e. beside the arrow. Because this
robot cannot strafe or reverse (forward+rotate only), closing a purely
*lateral* residual requires an awkward curve-in/rotate/creep/rotate-back
dance — slow and replan-heavy, matching the observed "turmoil." **This may
well be the same mechanism behind the original wp5/wp6 stalls, now seen more
clearly** — likely more "small lateral offset, hard to correct without
strafing" than "large final rotation" per se.

**Fix applied (`cat_patrol_robot/config/nav2_params.yaml`):**
1. Reverted `downsample_costmap` to `false` / `downsampling_factor: 1` —
   trading the CPU savings back for full 5cm planning resolution, to reduce
   the quantization snapping directly.
2. Loosened `general_goal_checker.xy_goal_tolerance` 0.15 → 0.20 as a buffer
   against whatever quantization/analytic-expansion slop remains.
YAML validated. **Config only, no rebuild, requires a T4 restart AND a
battery recharge first. Not yet tested live.**

---

## 11.5 RESUME CHECKLIST (start here next session)

### What's true right now (2026-07-11, end of session)

- **CONFIRMED WORKING LIVE, end to end:** strafe (Omni MPPI), the odom
  vy-feedback fix, and the NavfnPlanner revert were all tested together this
  session and the robot successfully completed waypoint navigation, including
  the previously-trivial-but-broken "short straight hop" case. This is the
  first time Phase 4's wp5/wp6-era reliability problem has actually been
  resolved rather than just reasoned about.
- **One open minor artifact:** at one waypoint the robot did a full
  unnecessary 360° spin before still reaching the goal successfully. Low
  severity (self-corrected, did not block the cycle) — not yet investigated.
  See §11 Session 3 for candidate causes (TwirlingCritic vs. residual
  short-path orientation noise vs. a one-off AMCL correction jump).
- **`patrol_manager` binary current** with: skip-and-continue on waypoint
  failure, per-waypoint email, the cmd_vel-vs-vel_raw stall guard (validated
  live), `/patrol_manager/waypoints` RViz markers. No pending code changes.
- **`yahboomcar_base_node` binary current and live-tested** — odom vy-zeroing
  fix confirmed working (this is what let strafe actually help).
- **`nav2_params.yaml` current and live-tested**: `PoseProgressChecker`,
  no-blind-backup BT XMLs, `behavior_plugins: [spin, wait]`,
  `xy_goal_tolerance: 0.20`, Omni MPPI strafe (`vy_max: 0.10`),
  `min_y_velocity_threshold: 0.01`, and now **`planner_server.GridBased:
  nav2_navfn_planner/NavfnPlanner`** (reverted from SmacPlannerHybrid, which
  is kept commented in the YAML as a fallback).

### Do this, in order

1. Bring up t1→t5 as usual. Re-confirm wp5/wp6 specifically (the original
   large-final-heading cases, 166-174°) still complete reliably via Omni's
   blended strafe+rotate — this is the one regression risk NavfnPlanner
   reintroduces if a future run exposes it (only one live test so far).
2. Decide whether the single 360°-spin artifact is worth chasing:
   - If it recurs or gets worse, watch `/amcl_pose` at the exact moment it
     starts (rules localization drift in/out) and check the RViz global Path
     shape for that waypoint (rules out a NavfnPlanner orientation-fill
     artifact on a short path segment in/out).
   - If it stays rare and harmless (self-corrects, cycle still succeeds),
     it's reasonable to leave it and move on — Phase 4's core reliability
     goal is met.
3. **Next natural milestone:** re-enable `loop_patrol: true` and run a real
   unattended cycle — this has never actually been tested yet (always run
   with `loop_patrol=false` so far). Confirms the scheduled-repeat behavior
   from the acceptance criteria in §2.

### Known-good reference values (confirmed working live 2026-07-11)

- `xy_goal_tolerance: 0.20`, `yaw_goal_tolerance: 0.25`
- `PoseProgressChecker`: `required_movement_radius: 0.5`,
  `required_movement_angle: 0.5`, `movement_time_allowance: 20.0`
- Stall guard: `stall_timeout_sec: 6.0`, `stall_cmd_vel_threshold: 0.03`,
  `stall_vel_raw_threshold: 0.02`
- Planner: `nav2_navfn_planner/NavfnPlanner` (SmacPlannerHybrid config
  preserved commented-out in `nav2_params.yaml` if ever needed again)
- Strafe (MPPI): `motion_model: Omni`, `vy_max: 0.10`, `vy_std: 0.15`,
  `min_y_velocity_threshold: 0.01`

---

### Session 3 — 2026-07-11

**User observation driving this session:** even after the SmacPlannerHybrid +
goal-tolerance fixes queued at end of Session 2, waypoint arrival is still
sometimes slow — robot ends up "next to the waypoint, facing the right way"
and grinds through a slow forward-only curve-in/rotate/creep correction to
close a small lateral gap. User pointed out the robot is holonomic (mecanum,
can strafe) and this capability was never actually used by Nav2 — confirmed
correct: `nav2_params.yaml`'s MPPI `FollowPath` had `motion_model: "DiffDrive"`,
`vy_max: 0.0`, `vy_std: 0.0` the whole time (comment already noted "works now,
but... keep off unless needed" from the Session 3 hardware-fix note, but was
never flipped on).

**Fix applied #1 (`cat_patrol_robot/config/nav2_params.yaml`, `FollowPath`):**
switched MPPI to holonomic: `motion_model: "Omni"`, `vy_max: 0.10`,
`vy_std: 0.15` (kept modest vs. `vx_max: 0.18` — mecanum rollers can still slip
under lateral load on this floor, per Session 3 hardware-fix caution). Chosen
scope: full Omni everywhere (not gated to near-goal only) — user's explicit
choice over a near-goal-only gate. Existing critics
(`GoalCritic`/`GoalAngleCritic` activate within `threshold_to_consider` 0.5/0.3m;
`PathAlignCritic`/`PathFollowCritic` dominate during transit) already bias
vy usage toward the final approach without a hard gate.

**Fix applied #2 (same file, `controller_server`):** found and fixed a stale
`min_y_velocity_threshold: 0.5` (leftover from when `vy_max` was 0). This
thresholds the *odometry-read* y-velocity fed back to MPPI as its current-state
estimate each control cycle (not the outgoing cmd_vel) — with `vy_max` now
0.10, a 0.5 threshold would always report vy=0 to the controller regardless of
real motion, silently defeating fix #1's feedback loop. Lowered to `0.01`.

**Fix applied #3 (real bug found while verifying #2, `yahboomcar_base_node/src/base_node_X3.cpp`):**
dispatched an Explore agent to trace the full cmd_vel/odom vy pipeline before
trusting fixes #1/#2 live. Confirmed:
- Command path is fine: Nav2 `/cmd_vel` → `Mcnamu_driver_X3.py:108-128`
  (`cmd_vel_callback`) reads `vy = msg.linear.y`, applies trim, calls
  `self.car.set_car_motion(vx, vy_corrected, w_corrected)` straight to the MCU
  — vy is never zeroed or dropped here.
- Feedback path was NOT fine: MCU-measured velocity → driver publishes
  `vel_raw` (real vy) → `base_node_X3.cpp`'s `OdomPublisher` subscribes
  `vel_raw`, integrates dead-reckoning pose correctly using the real
  `linear_velocity_y_`, but published `nav_msgs/Odometry` on `odom_raw` with
  the twist field hardcoded: `odom.twist.twist.linear.y = linear_velocity_y_;`
  immediately followed by `odom.twist.twist.linear.y = 0.0; // vy = 0.0` (old
  lines 126-127) — with a tight covariance (`odom.twist.covariance[7]:
  0.0001`). `yahboomcar_bringup_X3_launch.py` feeds `odom_raw` into a
  `robot_localization` `ekf_node` (`ekf_x1_x3.yaml`) whose `odom0_config` has
  **vy fusion enabled** (index 7 = true), remapped to `/odom` — exactly the
  topic Nav2's `controller_server` reads (`odom_topic: /odom` in
  `nav2_params.yaml`). Net effect: the EKF was fusing a confidently-zero vy
  measurement every cycle, so `/odom`'s reported y-velocity would always read
  ~0 regardless of real strafe motion — actively undermining fix #1/#2 even
  though the command path and position (x/y pose) integration were both
  correct. **Not dead code — a live bug**, just invisible before strafe existed
  because nobody looked at vy feedback when it was always commanded to 0
  anyway.
- **Fix:** deleted the hardcoded `odom.twist.twist.linear.y = 0.0;` override
  (base_node_X3.cpp), letting the real `linear_velocity_y_` value survive into
  the published message. Rebuilt clean:
  `colcon build --symlink-install --packages-select yahboomcar_base_node`
  (23s, only pre-existing unused-variable warnings, unrelated).

**LIVE TEST RESULT (same day, after restarting t1/t4/t5 with all three fixes
loaded):** mixed. Strafe visibly works and sometimes finds the right point
efficiently by strafing directly. But intermittently, on a waypoint that
should be trivial (goal ~10 inches straight ahead, same heading as current —
no correction needed at all), the robot instead starts rotating and eventually
fails to reach the target. User's own read: "it loses its actual location."
Robot brought home and `t1`/`t5` stopped to investigate before continuing.

**Root cause analysis (reasoning from code/config, not yet confirmed by live
logs):** two candidate mechanisms considered —
- **(A) Planner/controller motion-model mismatch (judged more likely, and
  chosen to fix first):** `planner_server` was still `SmacPlannerHybrid` with
  `motion_model_for_search: DUBIN` — a car-like, forward-only search with no
  concept that this robot can strafe. `FollowPath`'s `PathAlignCritic`
  (weight 14.0) and `PathFollowCritic` (weight 5.0) both dominate and push the
  controller to imitate whatever path Hybrid-A* found. If the DUBIN search
  snaps to a path with an unnecessary curve/rotation near the goal — plausible
  even for a "trivial" waypoint, due to grid quantization — the controller
  faithfully reproduces that spurious rotation instead of just strafing
  straight there. This is structural (tied to specific waypoint geometry/grid
  quantization), which fits the "sometimes great, sometimes awful" pattern
  better than random noise would.
- **(B) Position drift from mecanum slip during strafe:** checked
  `ekf_x1_x3.yaml` — heading (yaw) is fused from the IMU only
  (`odom0_config` has yaw index false), so a bad wheel-vy reading can't
  directly corrupt heading in the EKF. It could still corrupt the estimated
  x/y position between AMCL corrections (vx/vy are fused from `/odom_raw`
  with a tight, never-revisited covariance), which could make Nav2 briefly
  believe the goal isn't where it physically is. Considered less likely as
  the *primary* cause (would look more like random per-run noise than a
  waypoint-geometry-linked pattern) but not ruled out — worth checking
  `/amcl_pose` stability during a future bad occurrence if (A) doesn't fully
  resolve it.

**Fix applied for (A) (`cat_patrol_robot/config/nav2_params.yaml`,
`planner_server`):** reverted `GridBased.plugin` from
`nav2_smac_planner/SmacPlannerHybrid` back to `nav2_navfn_planner/NavfnPlanner`
— pure (x,y) Dijkstra/A* search, obstacle-aware, heading-blind. The whole
reason for adopting SmacPlannerHybrid on 2026-07-10 was that a non-holonomic
robot had no way to fix a large final-heading error except a slow
post-arrival point-turn (wp5/wp6's 166-174°), so Hybrid-A* pre-solved
orientation via a loop-around detour. That workaround assumed the robot
couldn't strafe — no longer true now that `FollowPath.motion_model: Omni` is
active. `PoseProgressChecker` (already credits in-place rotation as progress,
set 2026-07-10) plus Omni MPPI should now close final position AND heading
concurrently via a blended vx/vy/wz move, without needing the planner to
invent a detour. Bonus: NavfnPlanner is far lighter on this CPU-constrained
Orin NX (SmacPlannerHybrid was measured dropping to ~1.17Hz vs a 20Hz target).
Old SmacPlannerHybrid config kept as a commented fallback block in the YAML,
labeled to restore if this regresses wp5/wp6 specifically. YAML validated.
**Config-only, no rebuild, requires a T4 restart. Not yet tested live.**

**LIVE TEST OF THE PLANNER REVERT (same day): WORKED.** User confirmed the
NavfnPlanner-back + Omni MPPI combo resolved the spurious-rotation-on-a-trivial-
approach symptom — theory (A) confirmed as the real mechanism, no need to
pursue theory (B) (localization/slip) for this specific symptom.

**One remaining minor artifact, single occurrence:** at one waypoint the robot
did a full unnecessary 360° turn before still successfully reaching the goal.
Not investigated in depth (low severity — self-corrected, cycle succeeded).
Plausible causes, not yet distinguished: (a) `TwirlingCritic`
(weight 10.0, meant to discourage aimless spin) getting momentarily outvoted
by `PathAlignCritic`/`GoalAngleCritic` on a short/degenerate path segment where
NavfnPlanner's post-hoc per-pose orientation (filled in from the tangent
between consecutive grid-resolution path points, since NavfnPlanner itself has
no heading concept) is noisy for a very short hop, causing MPPI to chase a
spurious intermediate heading before correcting; or (b) a one-off AMCL
correction jump (residual theory (B) mechanism) making the robot briefly
believe its heading was wrong. Left as a known open item — see open questions.

**Status: strafe (Omni MPPI) + vy feedback fix + NavfnPlanner revert are ALL
now confirmed working live.** Core Phase 4 wp5/wp6-era reliability problem
appears resolved. Remaining open item is the single 360°-spin artifact above
(cosmetic/minor, did not prevent success).

- Scheduler repeat cycle: not reached yet this session
- Callback-group/executor behavior under load: not reached yet this session
- Notes: strafe-enable + planner-revert work took the whole session; live-
  tested successfully same day. No full unattended (`loop_patrol: true`)
  patrol cycle run yet with these changes — next session's natural next step.

### Final results

- patrol_period_sec used: ___
- waypoint count: ___
- capture success rate per cycle: ___
- unattended cycle: PASS / blockers ___
- Phase 4 acceptance criteria: all met / blockers ___

---

### Session 4 — 2026-07-17

**User request:** turn the proven single-cycle run (loop_patrol=false) into a
real continuous loop — repeat cycles back-to-back until `t5_patrol.sh` is
killed, with a short pause between runs (10s), instead of waiting on the
30-minute `patrol_period_sec` schedule timer.

**Root cause of why this didn't already work:** `loop_patrol_` was declared
and read from params but never actually consulted anywhere except a log
line in `send_home_goal()`'s success branch — it didn't gate anything. The
only thing that ever re-armed `wake_requested_` was `wake_timer_`, a
periodic (non-one-shot) wall timer firing every `patrol_period_sec_` (1800s
default) regardless of `loop_patrol_`.

**Fix applied (`patrol_manager_node.cpp`, `patrol_manager_params.yaml`):**
- New param `patrol_pause_sec` (default 10.0).
- New `schedule_next_cycle()`: one-shot wall timer (cancels itself on first
  fire) that sets `wake_requested_ = true` after `patrol_pause_sec_`.
- `send_home_goal()`'s success branch now actually branches on
  `loop_patrol_`: true → log + `schedule_next_cycle()` (next cycle starts
  ~10s after arriving home); false → stays asleep as before (unchanged
  behavior, still the safe single-cycle default for a first run).
- Home-goal **failure** and the stall guard's forced-SLEEPING path are
  deliberately left as hard stops (no auto re-wake) — those indicate a real
  problem worth an operator look, so they don't feed back into the loop.
- `patrol_manager_params.yaml`: `loop_patrol: false` → `true`, added
  `patrol_pause_sec: 10.0`.
- Old `wake_timer_`/`patrol_period_sec_` scheduled-wake mechanism left
  in place untouched (still useful as a first-wake / long-idle-recovery
  backstop); harmless overlap with the new pause timer since both just set
  the same `wake_requested_` flag.
- Rebuilt clean: `colcon build --symlink-install --packages-select
  patrol_manager` (~35s, no errors).

**LIVE TEST CONFIRMED (same day):** brought up t1→t5 with `loop_patrol: true`
and `patrol_pause_sec: 10.0`. The FSM cycles
`SLEEPING → WAKING → … → RETURNING → SLEEPING → (10s pause) → WAKING → …`
repeatedly and unattended, running continuously until `t5_patrol.sh` is
stopped, as requested. **This was the last open item blocking Phase 4 — all
required acceptance criteria (§2) are now met. Phase 4 reached.**

---

## 12. Open questions / forward links

- **Single unexplained 360° spin (2026-07-11).** One waypoint during the
  successful strafe+NavfnPlanner live test made a full unnecessary in-place
  rotation before still reaching the goal. Not yet root-caused (candidates:
  TwirlingCritic momentarily outvoted, NavfnPlanner's post-hoc per-pose
  orientation being noisy on a short path segment, or a one-off AMCL
  correction jump). Low priority — cosmetic, self-corrected, didn't block the
  cycle — but worth a closer look if it recurs or worsens.
- **Phase 5 data flywheel.** Are captured waypoint photos sufficient quality
  for early cat-detector dataset bootstrapping?
- **Phase 6 interruption model.** On cat detection, do we cancel active goal
  immediately and resume from same waypoint index afterward?
- **NavigateThroughPoses vs sequential NavigateToPose.** Is stop-and-capture at
  each waypoint easier with sequential goals (usually yes for this project).
- **Phase 7 scheduling hardening.** Move timer wake-up to systemd timers once
  patrol behavior is stable.

---

## 13. Learning guide — Phase 4 deep dive

### 13a. Data-flow diagram

```text
   patrol_manager (FSM)
        │
        ├─ subscribe /camera/color/image_raw
        │         └─ save JPEG -> /tmp/cat_patrol_images/*.jpg
        │
        ├─ action client /navigate_to_pose
        │         ├─ goal -> bt_navigator
        │         ├─ feedback <- distance_remaining
        │         └─ result <- succeeded/aborted/canceled
        │
        ├─ publish /patrol_manager/state
        └─ publish /cat_patrol/mail_request (JSON with file paths)
```

### 13b. Component responsibilities

- **`patrol_manager_node`**
  - Owns FSM + schedule + action client + image capture + mail publish.
- **`WaypointPatrolStrategy`**
  - Owns waypoint ordering and iteration.
- **Nav2 stack (`t4_nav`)**
  - Owns actual path planning/control/recovery.
- **Mail node (existing pipeline)**
  - Owns SMTP sending and attachment handling.

### 13c. Why callback-group split matters

Without split, a long callback (or accidental blocking wait) can delay FSM
transitions, image updates, and action feedback handling. The split makes the
node’s behavior predictable under load.

### 13d. What to verify in RViz during Phase 4

- Path appears to each waypoint.
- Local/global costmaps stay healthy.
- Robot stops near waypoint pose before capture.
- Return-home path is valid at cycle end.

---

## 14. C++ lessons checklist (Phase 4)

### 14a. FSM and async orchestration
- [ ] Encode state transitions explicitly.
- [ ] Keep transition side effects small and deterministic.
- [ ] Handle action result codes (`SUCCEEDED` / `ABORTED` / `CANCELED`) distinctly.

### 14b. Executor + callback groups
- [ ] `MutuallyExclusive` group for FSM timer **and** action result callback
      (serialize all FSM-state mutations).
- [ ] `Reentrant` group for the image subscription only.
- [ ] Validate behavior with `MultiThreadedExecutor` thread count changes.

### 14c. Strategy pattern continuation
- [ ] Keep `PatrolStrategy` interface minimal.
- [ ] Add one future strategy stub without touching FSM logic.

### 14d. Practical hardening (next)
- [ ] Add goal timeout/cancel path with `async_cancel_goal`.
- [ ] Add retry policy for transient nav failures.
- [ ] Add capture timeout/fallback if camera stream stalls.
- [ ] Add cycle summary metrics (durations, success counts) in logs.
