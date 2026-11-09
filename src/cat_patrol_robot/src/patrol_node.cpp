// =============================================================================
// patrol_node.cpp — Cat Patrol Robot  (ROS 2, C++17)
//
// WHAT THIS FILE IS:
//   This is the main (and only) C++ source file for the cat patrol robot.
//   It contains the ENTIRE brain of the robot: reading sensors, deciding what
//   to do, commanding the wheels, saving photos, and asking for emails.
//
// HOW IT WORKS (high-level):
//   The robot runs a "finite state machine" (FSM).  Think of it like a
//   flowchart with boxes:
//
//     [WaitingForTf] --> [Patrol] --> [Capture] --> [ReturnHome] --> [Idle]
//                             |                                        |
//                             +--- (cat seen?) --> [CatApproach] ------+
//                             |                                        |
//                             +---- (idle timer fires) <---------------+
//
//   A timer fires ~20 times per second (every 50 ms).  Each tick, the code
//   checks "which state am I in?" and runs the matching logic (drive forward,
//   take a photo, steer home, etc.).
//
// WHAT YOU NEED TO KNOW ABOUT ROS 2 TO READ THIS:
//   - A "node" is a single program that can send/receive messages.
//   - Nodes talk via "topics" (named channels). A node "publishes" data on
//     a topic; another node "subscribes" to receive that data.
//   - Messages have types: Twist (velocity), Image (camera picture),
//     LaserScan (LiDAR distances), Bool (true/false), String (text).
//   - "TF" (Transform Framework) tells you where things are in space.
//     For example: "where is the robot (base_footprint) relative to where
//     it started (odom)?"
//   - "Parameters" are settings you can change without recompiling, via
//     a YAML config file or command-line arguments.
// =============================================================================

// ---------------------------------------------------------------------------
// INCLUDES — pulling in code that other people wrote so we can use it
// ---------------------------------------------------------------------------

// This is OUR header file.  It declares the PatrolNode class, the
// PatrolState enum, and all the member variables.  The compiler needs to
// see the class declaration before we can write the method bodies here.
#include "cat_patrol_robot/patrol_node.hpp"
#include "cat_patrol_robot/patterns/classic_pattern.hpp"
#include "cat_patrol_robot/patterns/till_obstacle_back_pattern.hpp"

#include <algorithm>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>
#include <cmath>
#include <filesystem>
#include <sstream>

#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"

// This "using" lets us write things like "500ms" instead of
// "std::chrono::milliseconds(500)".  It's just a convenience.
using namespace std::chrono_literals;

// ===========================================================================
// Everything in this file lives inside the "cat_patrol_robot" namespace.
// A namespace is like a folder for names — it prevents clashes if two
// libraries both define a class called "PatrolNode".
// ===========================================================================
namespace cat_patrol_robot
{

// ---------------------------------------------------------------------------
// normalize_angle  (a free helper function, not part of any class)
// ---------------------------------------------------------------------------
// WHAT:  Takes any angle in radians and wraps it into the range (-pi, +pi].
//
// WHY:   Angles can grow without limit.  For example, if you turn 350 degrees
//        clockwise, the raw angle might be -6.1 radians.  But that's almost
//        the same as +0.17 radians (10 degrees counter-clockwise).  Steering
//        logic needs the SHORTEST turn direction, so we normalize first.
//
// HOW:   Keep subtracting or adding full circles (2*pi) until the value
//        falls within (-pi, +pi].
//
// EXAMPLE:
//   normalize_angle(3.5)   => 3.5           (already in range)
//   normalize_angle(7.0)   => 7.0 - 6.283  => ~0.717
//   normalize_angle(-4.0)  => -4.0 + 6.283 => ~2.283
//
// The word "static" here means this function is only visible inside this
// .cpp file — no other file can call it directly.
// ---------------------------------------------------------------------------
static double normalize_angle(double a)
{
  // M_PI is a constant ≈ 3.14159265...  (pi, 180 degrees)
  while (a > M_PI) {a -= 2.0 * M_PI;}   // too far positive: subtract a full turn
  while (a < -M_PI) {a += 2.0 * M_PI;}  // too far negative: add a full turn
  return a;
}

// ===========================================================================
// CONSTRUCTOR — runs once when the node is created
// ===========================================================================
// In C++, a "constructor" is a special function that runs automatically when
// you create an object.  Here it sets up EVERYTHING the robot needs before
// the main loop starts: parameters, publishers, subscribers, timers, etc.
//
// "rclcpp::Node" is the ROS 2 base class for all nodes.  By inheriting from
// it (see the header file), PatrolNode gets all the ROS plumbing for free:
// logging, parameter handling, creating topics, timers, etc.
//
// The ": rclcpp::Node("patrol_node", options)" part is called an
// "initializer list" — it calls the parent class constructor with the
// node name "patrol_node" before our code body runs.
// ===========================================================================
PatrolNode::PatrolNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("patrol_node", options)
{
  // =========================================================================
  // STEP 1: Declare parameters
  // =========================================================================
  // "declare_parameter" tells ROS: "this node uses a setting called X, and
  // if nobody provides a value, use this default."  The actual value can
  // come from a YAML config file, a launch file, or the command line.
  //
  // This is powerful: you can change speeds, topic names, timeouts, etc.
  // WITHOUT recompiling the C++ code.  Just edit the YAML and relaunch.
  // =========================================================================

  // --- Topic names (strings) ---
  // These define WHICH ROS topics this node talks on.
  // The robot driver listens on /cmd_vel for velocity commands.
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  // The camera publishes color images here.
  image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
  // The LiDAR (laser scanner) publishes distance measurements here.
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  // The Yahboom driver accepts a Bool to turn the buzzer on/off.
  buzzer_topic_ = declare_parameter<std::string>("buzzer_topic", "Buzzer");
  // A separate joystick node publishes true/false here when a human takes over.
  joy_topic_ = declare_parameter<std::string>("joy_active_topic", "/JoyState");

  // --- TF frame names ---
  // "odom" is the origin: where the robot thinks it started.
  // "base_footprint" is the robot itself (center of its wheels on the ground).
  // TF can answer: "where is base_footprint relative to odom?"
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

  // --- File and mail settings ---
  // Folder where captured JPEG photos are saved on disk.
  image_save_dir_ = declare_parameter<std::string>("image_save_dir", "/tmp/cat_patrol_images");
  // Topic where we publish a JSON "please send email" request.
  mail_request_topic_ = declare_parameter<std::string>("mail_request_topic", "/cat_patrol/mail_request");
  // Subject line of the email.
  mail_subject_ = declare_parameter<std::string>("mail_subject", "Cat patrol photos");
  // Who receives the email.
  mail_to_ = declare_parameter<std::string>("smtp_to_address", "user@example.com");
  // Topic where an external cat-detection node publishes true/false.
  cat_detected_topic_ = declare_parameter<std::string>("cat_detected_topic", "/cat_patrol/cat_detected");
  // cat_detector_cpp's per-frame detections (carries the white/brown identity).
  detections_topic_ = declare_parameter<std::string>(
    "detections_topic", "/cat_detector_cpp/detections");
  // Email one snapshot when a white/brown cat is recognized DURING a patrol.
  cat_recognized_mail_enabled_ = declare_parameter<bool>("cat_recognized_mail_enabled", true);
  cat_recognized_min_conf_ = declare_parameter<double>("cat_recognized_min_conf", 0.6);
  cat_recognized_cooldown_sec_ =
    declare_parameter<double>("cat_recognized_cooldown_sec", 30.0);
  // Email one snapshot at the start and end of every patrol loop.
  loop_boundary_mail_enabled_ = declare_parameter<bool>("loop_boundary_mail_enabled", true);
  // Battery voltage topic (published by Mcnamu_driver_X3 as Float32 on "voltage")
  voltage_topic_ = declare_parameter<std::string>("voltage_topic", "voltage");
  low_voltage_warn_v_ = declare_parameter<double>("low_voltage_warn_v", 11.0);

  // --- Behavior toggles and numeric tuning ---
  use_lidar_ = declare_parameter<bool>("use_lidar_obstacle_stop", false);
  obstacle_min_range_ = declare_parameter<double>("obstacle_min_range_m", 0.35);
  obstacle_sector_deg_ = declare_parameter<double>("obstacle_sector_deg", 60.0);

  // Depth-camera obstacle detection (works without LiDAR)
  depth_image_topic_ = declare_parameter<std::string>("depth_image_topic", "/camera/depth/image_raw");
  depth_obstacle_min_m_ = declare_parameter<double>("depth_obstacle_min_m", 0.50);
  depth_sector_width_ratio_ = declare_parameter<double>("depth_sector_width_ratio", 0.33);
  // Max acceptable age (sec) of a depth frame before we treat depth as DEAD
  // and fail-safe by reporting an obstacle (so the robot stops moving forward).
  depth_max_age_sec_ = declare_parameter<double>("depth_max_age_sec", 1.5);

  // Patrol pattern selection: "classic" or "till_obstacle_back_images_turn"
  patrol_pattern_name_ = declare_parameter<std::string>("patrol_pattern", "classic");
  forward_timeout_sec_ = declare_parameter<double>("forward_timeout_sec", 60.0);
  drive_back_timeout_sec_ = declare_parameter<double>("drive_back_timeout_sec", 60.0);
  pattern_log_buffer_period_sec_ = declare_parameter<double>("pattern_log_buffer_period_sec", 60.0);
  pattern_log_file_path_ = declare_parameter<std::string>(
    "pattern_log_file_path", "/tmp/cat_patrol_pattern.log");
  // Optional .wav file played when the patrol cycle finishes the 180° turn.
  // Empty string disables playback.  The path may use ~ which is expanded
  // below using $HOME.
  completion_sound_path_ = declare_parameter<std::string>(
    "completion_sound_path", "");
  completion_sound_player_ = declare_parameter<std::string>(
    "completion_sound_player", "paplay");
  bt_connect_on_start_ = declare_parameter<bool>("bt_connect_on_start", true);
  bt_speaker_mac_ = declare_parameter<std::string>(
    "bt_speaker_mac", "0C:8E:29:3B:F2:5F");
  bt_speaker_name_ = declare_parameter<std::string>("bt_speaker_name", "RX-V485");
  bt_scan_seconds_ = declare_parameter<int>("bt_scan_seconds", 40);
  bt_connect_script_ = declare_parameter<std::string>("bt_connect_script", "");
  // Expand a leading ~/ using $HOME (declare_parameter doesn't do this).
  if (!completion_sound_path_.empty() && completion_sound_path_[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home) {
      completion_sound_path_ = std::string(home) + completion_sound_path_.substr(1);
    }
  }

  // Forward/backward driving speed during patrol (meters per second).
  linear_speed_ = declare_parameter<double>("linear_speed", 0.2);
  // Turning speed during return-home and alignment (radians per second).
  angular_speed_ = declare_parameter<double>("angular_speed", 0.6);
  // How many seconds each patrol "leg" lasts (forward OR backward).
  patrol_drive_sec_ = declare_parameter<double>("patrol_drive_sec", 10.0);

  // How many photos to take during the 360-degree capture.
  capture_frame_count_ = declare_parameter<int>("capture_frame_count", 12);
  // How fast to spin between photos (radians per second).
  capture_turn_speed_ = declare_parameter<double>("capture_turn_speed", 3.0);
  // How long to spin between each photo (seconds).
  capture_rotate_sec_ = declare_parameter<double>("capture_rotate_sec", 3.0);

  // Proportional gain for the yaw P-controller during capture rotation.
  capture_kp_ = declare_parameter<double>("capture_kp", 2.0);
  // Minimum angular speed to prevent motor stall during capture rotation.
  capture_min_angular_speed_ = declare_parameter<double>("capture_min_angular_speed", 0.15);
  // Yaw tolerance for "close enough" during capture rotation (rad).
  capture_yaw_tolerance_ = declare_parameter<double>("capture_yaw_tolerance", 0.10);
  // Extra angle added to final capture closure before completion beep.
  capture_completion_extra_deg_ = declare_parameter<double>("capture_completion_extra_deg", 0.0);

  // Maximum time allowed to drive home before giving up (seconds).
  return_timeout_sec_ = declare_parameter<double>("return_home_max_time_sec", 120.0);
  // "Close enough" distance to home position (meters).
  pos_tol_ = declare_parameter<double>("position_tolerance_m", 0.15);
  // "Close enough" heading angle at home (radians).  0.2 rad ≈ 11 degrees.
  yaw_tol_ = declare_parameter<double>("yaw_tolerance_rad", 0.2);

  // How long to wait between patrol cycles when idle (seconds). 1800 = 30 min.
  patrol_period_sec_ = declare_parameter<double>("patrol_period_sec", 1800.0);
  // Start patrolling immediately when the node launches?
  start_on_boot_ = declare_parameter<bool>("start_patrol_on_boot", true);
  // After finishing one cycle, automatically start another one later?
  loop_patrol_ = declare_parameter<bool>("loop_patrol", true);

  // Enable the optional "cat detected -> approach" behavior?
  cat_detection_enabled_ = declare_parameter<bool>("cat_detection_enabled", false);
  // How fast to creep toward a detected cat (meters per second).
  approach_linear_ = declare_parameter<double>("approach_linear_speed", 0.1);
  // Maximum time to spend approaching a cat before giving up (seconds).
  approach_max_sec_ = declare_parameter<double>("approach_max_sec", 15.0);

  // If true, the node shuts down after completing one full patrol cycle.
  // Useful for testing: run once, verify, stop.
  exit_after_one_cycle_ = declare_parameter<bool>("exit_after_one_cycle", false);
  // If true, respect the joystick override topic.
  use_joy_override_ = declare_parameter<bool>("use_joy_override", false);

  // Environment variable override: if CAT_PATROL_EXIT_ONCE is set (to anything),
  // force exit-after-one-cycle mode.  This lets you control behavior from a
  // shell script without changing the YAML config.
  // std::getenv returns nullptr if the variable is not set.
  if (std::getenv("CAT_PATROL_EXIT_ONCE")) {
    exit_after_one_cycle_ = true;
  }

  // How often the main control loop runs (milliseconds).  50 ms = 20 Hz.
  // This is the "heartbeat" of the state machine.
  const int period_ms = declare_parameter<int>("patrol_timer_period_ms", 50);

  // =========================================================================
  // STEP 2: Create the image save directory (if it doesn't exist yet)
  // =========================================================================
  // std::filesystem::create_directories works like "mkdir -p" in the shell:
  // it creates all parent directories as needed and doesn't fail if the
  // directory already exists.
  std::filesystem::create_directories(image_save_dir_);

  // =========================================================================
  // STEP 3: Create PUBLISHERS
  // =========================================================================
  // A publisher sends messages OUT from this node to a topic.
  // Other nodes (or the robot hardware driver) subscribe to read them.
  //
  // The "10" is the "queue depth" — how many unsent messages to buffer.
  // If the subscriber can't keep up, older messages get dropped.
  // =========================================================================

  // cmd_vel publisher: sends Twist messages (linear + angular velocity).
  // The Yahboom motor driver subscribes to this and moves the wheels.
  //
  // geometry_msgs::msg::Twist has two parts:
  //   linear:  { x, y, z }   — forward, sideways, up  (m/s)
  //   angular: { x, y, z }   — roll, pitch, yaw rates  (rad/s)
  // For a ground robot, we typically only use linear.x (forward) and
  // angular.z (turn left/right).  Mecanum wheels can also use linear.y.
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  // Buzzer publisher: sends true (beep on) or false (beep off).
  buzzer_pub_ = create_publisher<std_msgs::msg::Bool>(buzzer_topic_, 10);

  // Mail request publisher: sends a JSON string to the Python mail_node.
  mail_pub_ = create_publisher<std_msgs::msg::String>(mail_request_topic_, 10);

  // =========================================================================
  // STEP 4: Create SUBSCRIBERS
  // =========================================================================
  // A subscriber listens for messages on a topic and calls a "callback"
  // function every time a new message arrives.
  //
  // "std::bind" connects the callback function to this specific object.
  // "std::placeholders::_1" means "pass the received message as argument 1".
  //
  // rclcpp::SensorDataQoS() is a Quality-of-Service profile optimized for
  // sensors: "best effort" delivery (drop messages if slow), no durability
  // (no storing old messages for late joiners).  This matches how cameras
  // and LiDARs typically publish: high frequency, latest-value-wins.
  // =========================================================================

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PatrolNode::image_cb, this, std::placeholders::_1));

  // Depth camera subscriber for obstacle detection (always active).
  depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PatrolNode::depth_image_cb, this, std::placeholders::_1));

  // LiDAR subscriber (only created if use_lidar_ is true).
  if (use_lidar_) {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PatrolNode::scan_cb, this, std::placeholders::_1));
  }

  // Joystick override subscriber (only created if use_joy_override_ is true).
  if (use_joy_override_) {
    joy_sub_ = create_subscription<std_msgs::msg::Bool>(
      joy_topic_, 10, std::bind(&PatrolNode::joy_cb, this, std::placeholders::_1));
  }

  // Cat detection subscriber: listens for true/false from an external
  // detection node (could be running a neural network, for example).
  cat_sub_ = create_subscription<std_msgs::msg::Bool>(
    cat_detected_topic_, 10, std::bind(&PatrolNode::cat_detected_cb, this, std::placeholders::_1));

  // Cat IDENTITY subscriber: cat_detector_cpp publishes a Detection2DArray per
  // frame with the white/brown identity in results[1]. detections_cb() emails a
  // snapshot when a confident identity arrives mid-patrol. Only created when the
  // feature is enabled so we don't subscribe for nothing.
  if (cat_recognized_mail_enabled_) {
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic_, 10,
      std::bind(&PatrolNode::detections_cb, this, std::placeholders::_1));
  }

  // Battery voltage subscriber (driver publishes ~10 Hz). Used to surface
  // gradual turn degradation that is actually caused by low battery.
  voltage_sub_ = create_subscription<std_msgs::msg::Float32>(
    voltage_topic_, 10, std::bind(&PatrolNode::voltage_cb, this, std::placeholders::_1));

  // =========================================================================
  // STEP 5: Set up the TF (Transform) system
  // =========================================================================
  // TF is how ROS tracks where things are in space.  Imagine a tree of
  // coordinate frames:
  //
  //   map --> odom --> base_footprint --> camera_link
  //                                  --> laser_link
  //
  // The "odom -> base_footprint" transform tells us: "the robot's center
  // is at position (x, y) and heading (yaw) relative to where it started."
  //
  // tf_buffer_ stores recent transforms (a short history).
  // tf_listener_ automatically subscribes to the /tf topic and fills
  // the buffer with incoming transforms.  We just call lookupTransform()
  // later when we need to know where the robot is.
  // =========================================================================
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // =========================================================================
  // STEP 6: Create the main control timer
  // =========================================================================
  // This is the HEARTBEAT of the robot.  Every "period_ms" milliseconds
  // (default 50 ms = 20 times per second), the function patrol_timer_cb()
  // is called.  That function checks the current state and does the right
  // thing (drive, capture, steer home, etc.).
  //
  // "create_wall_timer" uses real wall-clock time (not simulation time).
  // =========================================================================
  patrol_timer_ = create_wall_timer(
    std::chrono::milliseconds(period_ms), std::bind(&PatrolNode::patrol_timer_cb, this));

  // Record the current time — used to measure how long we've been in a state.
  state_enter_time_ = now();

  // =========================================================================
  // STEP 7: (Optional) Create the idle cycle timer for looping patrols
  // =========================================================================
  // If loop_patrol_ is true, we want the robot to automatically start a
  // new patrol after waiting patrol_period_sec_ seconds in Idle state.
  //
  // This timer fires periodically.  When it fires, IF we're in Idle,
  // we transition to Patrol.  If we're not in Idle (already patrolling),
  // we just ignore it.
  //
  // The "[this]() { ... }" syntax is a C++ LAMBDA — an anonymous function
  // defined inline.  "this" is captured so the lambda can access our
  // member variables (state_, etc.).
  // =========================================================================
  if (loop_patrol_) {
    idle_cycle_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(1.0, patrol_period_sec_)),
      [this]() {
        if (state_ == PatrolState::Idle) {
          RCLCPP_INFO(get_logger(),
            "Patrol period elapsed — starting cycle (battery=%.2f V)",
            static_cast<double>(last_voltage_v_.load()));
          transition_to(PatrolState::Patrol);
        }
      });
  }

  // =========================================================================
  // STEP 8: Heartbeat beep — short buzz every 60 s so you know the node is alive
  // =========================================================================
  heartbeat_timer_ = create_wall_timer(
    std::chrono::seconds(60),
    [this]() {
      set_buzzer(true);
      heartbeat_off_timer_ = create_wall_timer(
        std::chrono::milliseconds(150),
        [this]() {
          set_buzzer(false);
          heartbeat_off_timer_->cancel();
        });
    });

  // =========================================================================
  // STEP 9: Create the active patrol pattern
  // =========================================================================
  create_active_pattern();

  // Loud effective-config banner so the operator can VERIFY which YAML/values
  // are actually in effect this run (avoid source-vs-install confusion).
  RCLCPP_INFO(get_logger(),
    "==================== EFFECTIVE PATROL CONFIG ====================");
  RCLCPP_INFO(get_logger(),
    "  patrol_pattern             = %s", patrol_pattern_name_.c_str());
  RCLCPP_INFO(get_logger(),
    "  linear_speed               = %.3f m/s", linear_speed_);
  RCLCPP_INFO(get_logger(),
    "  angular_speed              = %.3f rad/s", angular_speed_);
  RCLCPP_INFO(get_logger(),
    "  capture_frame_count        = %d", capture_frame_count_);
  RCLCPP_INFO(get_logger(),
    "  capture_turn_speed         = %.3f rad/s", capture_turn_speed_);
  RCLCPP_INFO(get_logger(),
    "  capture_kp                 = %.3f", capture_kp_);
  RCLCPP_INFO(get_logger(),
    "  capture_min_angular_speed  = %.3f rad/s", capture_min_angular_speed_);
  RCLCPP_INFO(get_logger(),
    "  capture_yaw_tolerance      = %.3f rad", capture_yaw_tolerance_);
  RCLCPP_INFO(get_logger(),
    "  capture_completion_extra_deg = %.3f deg", capture_completion_extra_deg_);
  RCLCPP_INFO(get_logger(),
    "  forward_timeout_sec        = %.1f", forward_timeout_sec_);
  RCLCPP_INFO(get_logger(),
    "  drive_back_timeout_sec     = %.1f", drive_back_timeout_sec_);
  RCLCPP_INFO(get_logger(),
    "  depth_image_topic          = %s", depth_image_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "  depth_obstacle_min_m       = %.2f", depth_obstacle_min_m_);
  RCLCPP_INFO(get_logger(),
    "  depth_max_age_sec          = %.2f (safe-stop if depth older than this)",
    depth_max_age_sec_);
  RCLCPP_INFO(get_logger(),
    "  voltage_topic              = %s", voltage_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "  low_voltage_warn_v         = %.2f V", low_voltage_warn_v_);
  RCLCPP_INFO(get_logger(),
    "  pattern_log_file_path      = %s", pattern_log_file_path_.c_str());
  RCLCPP_INFO(get_logger(),
    "================================================================");

  // Safety shutdown hook: ensure the chassis is commanded to stop when ROS
  // is shutting down (e.g., Ctrl+C in launch terminal).
  rclcpp::on_shutdown([this]() {
    stop_robot();
    set_buzzer(false);
  });

  // =========================================================================
  // STEP 9: Pick the initial state
  // =========================================================================
  if (start_on_boot_) {
    state_ = PatrolState::WaitingForTf;
    RCLCPP_INFO(get_logger(), "Waiting for TF (odom -> base_footprint) before first patrol...");
  } else {
    transition_to(PatrolState::Idle);
  }

  if (bt_connect_on_start_ && !completion_sound_path_.empty()) {
    start_bluetooth_connect_async();
  }
}

// ===========================================================================
// transition_to — change to a new state
// ===========================================================================
// WHAT:  This is the ONE place where we switch from one FSM state to another.
//        It always stops the robot first (safety!), records the time of the
//        switch, and does any one-time setup the new state needs.
//
// WHY:   Having a SINGLE transition function means:
//        - The robot always stops between states (no runaway wheels).
//        - We never forget to reset the state timer.
//        - State-specific setup is easy to find and maintain.
//
// HOW IT'S CALLED:
//        transition_to(PatrolState::Capture);   // switch to Capture state
//
// IMPORTANT DETAIL about "home":
//        When entering Patrol, we record the robot's current (x, y, yaw)
//        in the odom frame.  Later, ReturnHome drives back to that spot.
//        Since odometry drifts over time (wheel slip, etc.), this "home"
//        position is approximate — it's NOT a GPS coordinate.
// ===========================================================================
void PatrolNode::transition_to(PatrolState s)
{
  // ALWAYS stop the wheels before doing anything else.
  stop_robot();

  // Remember where we came FROM before overwriting state_, so the
  // loop-boundary emails below can tell a real cycle start/end apart from
  // mid-cycle re-entries (e.g. CatApproach → Patrol) and the startup Idle.
  const PatrolState prev = state_;

  // Update the state and record when we entered it.
  state_ = s;
  state_enter_time_ = now();

  // --- State-specific setup ---

  if (s == PatrolState::Patrol) {
    saved_paths_.clear();
    frames_saved_ = 0;
    cat_detected_.store(false);

    // Loop START email: only when a NEW cycle begins (from Idle / first-boot
    // WaitingForTf), not when CatApproach hands control back to Patrol.
    if (loop_boundary_mail_enabled_ &&
        (prev == PatrolState::Idle || prev == PatrolState::WaitingForTf))
    {
      const std::string path = save_snapshot("loopstart");
      if (!path.empty()) {
        send_mail_request({path}, mail_subject_ + " — patrol loop started");
      }
    }

    if (get_odom_pose(home_x_, home_y_, home_yaw_)) {
      RCLCPP_INFO(get_logger(),
        "Recorded home (odom): x=%.3f y=%.3f yaw=%.3f | battery=%.2f V",
        home_x_, home_y_, home_yaw_,
        static_cast<double>(last_voltage_v_.load()));
    } else {
      RCLCPP_WARN(get_logger(), "Could not read odom pose at patrol start — return-home may fail");
    }

    if (active_pattern_) {
      auto ctx = build_patrol_context();
      active_pattern_->on_start(ctx);
    }
  }

  if (s == PatrolState::Capture) {
    // Clear any photos from a previous cycle.
    saved_paths_.clear();
    frames_saved_ = 0;
    // Record when capture started (for the 90-second safety timeout).
    capture_phase_start_ = now();
    // Start in the "Snap" sub-phase (take a picture first, then rotate).
    capture_phase_ = CapturePhase::Snap;
    RCLCPP_INFO(get_logger(), "Capture: will take %d images over 360 degrees", capture_frame_count_);
  }

  if (s == PatrolState::Idle) {
    // Loop END email: only when we return to Idle after a cycle actually ran
    // (Patrol / Capture / ReturnHome / CatApproach). This covers both the
    // till-obstacle pattern (Patrol → Idle via DoneIdle) and the classic
    // pattern (… → ReturnHome → Idle), while skipping the startup Idle.
    if (loop_boundary_mail_enabled_ &&
        (prev == PatrolState::Patrol || prev == PatrolState::Capture ||
         prev == PatrolState::ReturnHome || prev == PatrolState::CatApproach))
    {
      const std::string path = save_snapshot("loopend");
      if (!path.empty()) {
        send_mail_request({path}, mail_subject_ + " — patrol loop ended");
      }
    }
  }

  if (s == PatrolState::CatApproach) {
    // Record when we started approaching (for the approach timeout).
    approach_start_ = now();
    // Turn the buzzer ON to alert the household.
    std_msgs::msg::Bool b;
    b.data = true;
    buzzer_pub_->publish(b);
  }
}

// ===========================================================================
// get_odom_pose — ask TF: "where is the robot right now?"
// ===========================================================================
// WHAT:  Looks up the transform from the odom frame to the robot's base
//        frame, and extracts the 2D position (x, y) and heading (yaw).
//
// WHY:   This is how the robot knows where it is.  The wheel encoders
//        (and optionally IMU) publish transforms to /tf.  We just read them.
//
// RETURNS:
//        true  — success, x/y/yaw are filled in
//        false — TF not available yet (robot just started, wrong frame, etc.)
//
// PARAMETERS (passed by reference — the function WRITES into them):
//        x   — forward position in meters (positive = forward from start)
//        y   — sideways position in meters (positive = left from start)
//        yaw — heading angle in radians (0 = facing original direction)
//
// ABOUT QUATERNIONS:
//        3D rotations in ROS are stored as "quaternions" (4 numbers: x,y,z,w).
//        They avoid a problem called "gimbal lock" that Euler angles have.
//        We convert the quaternion to "roll, pitch, yaw" (3 rotation angles)
//        because for a ground robot we only care about yaw (left/right turn).
//        Roll and pitch are ignored (the robot doesn't tilt much on flat ground).
// ===========================================================================
bool PatrolNode::get_odom_pose(double & x, double & y, double & yaw)
{
  try {
    // Ask the TF buffer: "what is the latest known transform from
    // odom_frame_ to base_frame_?"
    // tf2::TimePointZero means "give me the most recent one you have."
    geometry_msgs::msg::TransformStamped t =
      tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);

    // Extract the position (translation) part.
    x = t.transform.translation.x;
    y = t.transform.translation.y;

    // Extract the rotation part (a quaternion) and convert to yaw.
    tf2::Quaternion q(
      t.transform.rotation.x, t.transform.rotation.y,
      t.transform.rotation.z, t.transform.rotation.w);
    double roll{}, pitch{};  // We don't use these, but getRPY needs them.
    // getRPY = "get Roll, Pitch, Yaw" from the rotation matrix.
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    // If the lookup fails (TF not ready, frames missing, etc.), log a
    // warning but don't spam — WARN_THROTTLE limits to once per 3000 ms.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "TF lookup failed: %s", ex.what());
    return false;
  }
}

// ===========================================================================
// publish_twist — send a velocity command to the robot wheels
// ===========================================================================
// WHAT:  Creates a Twist message with the given velocities and publishes it
//        on the cmd_vel topic.
//
// WHY:   This is THE way to make the robot move in ROS.  The motor driver
//        node subscribes to cmd_vel and translates the velocities into
//        individual wheel speeds.
//
// PARAMETERS:
//   lin_x — forward/backward speed (m/s).  Positive = forward.
//   lin_y — sideways speed (m/s).  Only works on mecanum/omni wheels.
//           For a normal 2-wheel robot, this is always 0.
//   ang_z — turning speed (rad/s).  Positive = counter-clockwise (left).
//           This follows the ROS "right-hand rule" convention.
//
// EXAMPLE CALLS:
//   publish_twist(0.2, 0.0, 0.0)   — drive forward at 0.2 m/s
//   publish_twist(0.0, 0.0, 0.5)   — turn left at 0.5 rad/s
//   publish_twist(0.0, 0.0, 0.0)   — stop (zero velocity)
// ===========================================================================
void PatrolNode::publish_twist(double lin_x, double lin_y, double ang_z)
{
  geometry_msgs::msg::Twist t;
  t.linear.x = lin_x;
  t.linear.y = lin_y;
  t.angular.z = ang_z;
  cmd_pub_->publish(t);
}

// ===========================================================================
// stop_robot — emergency/default stop: publish zero velocity
// ===========================================================================
// Called every time we change states, and whenever something goes wrong.
// Publishing (0, 0, 0) tells the motor driver: "all wheels stop NOW."
// ===========================================================================
void PatrolNode::stop_robot()
{
  publish_twist(0.0, 0.0, 0.0);
}

void PatrolNode::set_buzzer(bool on)
{
  std_msgs::msg::Bool b;
  b.data = on;
  buzzer_pub_->publish(b);
}

// ===========================================================================
// play_completion_sound — spawn the audio player in the background
// ===========================================================================
// WHAT:  If completion_sound_path_ is non-empty, fire-and-forget a shell
//        command like:   aplay -q /home/jetson/bark.wav
//        The shell's trailing "&" detaches the player so the patrol's
//        20 Hz tick loop is not blocked.
//
// WHY system() AND NOT A ROS TOPIC:
//        The Yahboom Jetson has no buzzer-style ROS audio service; the
//        onboard buzzer is already used for the per-frame beeps.  ALSA's
//        aplay is the simplest reliable way to play a .wav file on this
//        hardware, and `command &` is non-blocking once the child detaches.
//
// SAFETY:
//        We only invoke system() if the path looks safe (no shell meta-
//        characters).  This avoids accidental command injection from a
//        misconfigured YAML.
// ===========================================================================
std::string PatrolNode::resolve_bt_connect_script() const
{
  if (!bt_connect_script_.empty() && std::filesystem::exists(bt_connect_script_)) {
    return bt_connect_script_;
  }
  const std::filesystem::path installed{
    "/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/install/cat_patrol_robot/lib/"
    "cat_patrol_robot/connect_bt_speaker.sh"};
  if (std::filesystem::exists(installed)) {
    return installed.string();
  }
  return {};
}

bool PatrolNode::connect_bluetooth_speaker()
{
  const std::string script = resolve_bt_connect_script();
  if (script.empty()) {
    RCLCPP_WARN(get_logger(),
      "connect_bt_speaker.sh not found — set bt_connect_script or rebuild package");
    return false;
  }

  static const std::string forbidden = "`$();|&<>\n\r\"\\'";
  auto safe = [&](const std::string & s) {
    return s.find_first_of(forbidden) == std::string::npos;
  };
  if (!safe(script) || !safe(bt_speaker_mac_) || !safe(bt_speaker_name_)) {
    RCLCPP_WARN(get_logger(), "BT connect: unsafe characters in path/MAC/name");
    return false;
  }

  const std::string cmd = "bash \"" + script + "\" \"" + bt_speaker_mac_ + "\" \"" +
    bt_speaker_name_ + "\" " + std::to_string(bt_scan_seconds_);

  RCLCPP_INFO(get_logger(), "Connecting BT speaker: %s (%s)", bt_speaker_name_.c_str(),
    bt_speaker_mac_.c_str());
  const int rc = std::system(cmd.c_str());
  if (rc != 0) {
    int code = rc;
    if (WIFEXITED(rc)) {
      code = WEXITSTATUS(rc);
    }
    RCLCPP_WARN(get_logger(),
      "BT speaker connect failed (exit %d) — power ON RX-V485, disconnect it from phone, "
      "keep within ~5 m; test: bash %s",
      code, script.c_str());
    return false;
  }
  bt_audio_ready_.store(true);
  RCLCPP_INFO(get_logger(), "BT speaker ready (PulseAudio default sink is bluez)");
  return true;
}

void PatrolNode::start_bluetooth_connect_async()
{
  std::thread([this]() {
    if (connect_bluetooth_speaker()) {
      play_completion_sound();
    }
  }).detach();
}

void PatrolNode::play_completion_sound()
{
  if (completion_sound_path_.empty()) {
    return;  // playback disabled
  }

  if (bt_connect_on_start_ && !bt_audio_ready_.load()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Skipping bark — BT speaker not connected yet");
    return;
  }

  // Reject anything that looks like a shell injection attempt.  These chars
  // have no business in a file path on Linux.
  static const std::string forbidden = "`$();|&<>\n\r\"\\'";
  if (completion_sound_path_.find_first_of(forbidden) != std::string::npos) {
    RCLCPP_WARN(get_logger(),
      "completion_sound_path contains shell metacharacters — refusing to play: %s",
      completion_sound_path_.c_str());
    return;
  }

  // paplay needs the user PulseAudio socket (XDG_RUNTIME_DIR), especially when
  // patrol_node is started from systemd/launch without a full desktop session.
  const char * runtime = std::getenv("XDG_RUNTIME_DIR");
  const std::string runtime_dir = (runtime && runtime[0] != '\0') ?
    std::string(runtime) : ("/run/user/" + std::to_string(getuid()));
  const std::string player = bt_audio_ready_.load() ? "paplay" : completion_sound_player_;
  const std::string cmd = "XDG_RUNTIME_DIR=" + runtime_dir + " " + player + " \"" +
    completion_sound_path_ + "\" >/dev/null 2>&1 &";

  RCLCPP_INFO(get_logger(), "Playing completion sound: %s", cmd.c_str());
  const int rc = std::system(cmd.c_str());
  if (rc != 0) {
    RCLCPP_WARN(get_logger(),
      "completion sound playback returned non-zero (%d) — check BT connection and paplay",
      rc);
  }
}

// ===========================================================================
// obstacle_too_close — is something blocking the front of the robot?
// ===========================================================================
// WHAT:  Returns true if the LiDAR reports an obstacle within the safety
//        distance.  Returns false if LiDAR is disabled.
//
// HOW:   The scan_cb callback (below) constantly updates min_scan_range_
//        with the closest reading in the front cone.  We just check that
//        value here.
//
// THREAD SAFETY:
//        min_scan_range_ is an "atomic" variable.  This means it can be
//        safely read and written from different threads without a mutex.
//        The ".load()" call reads the current value atomically.
// ===========================================================================
bool PatrolNode::obstacle_too_close()
{
  if (!use_lidar_) {
    return false;  // LiDAR not enabled, so we never report obstacles.
  }
  return min_scan_range_.load() < static_cast<float>(obstacle_min_range_);
}

// ===========================================================================
// scan_cb — LiDAR callback: find closest obstacle in front
// ===========================================================================
// WHEN:  Called automatically every time a new LaserScan message arrives
//        from the LiDAR sensor (typically 5-15 Hz).
//
// WHAT:  A LaserScan message contains an array of distances (ranges[]),
//        one per laser beam.  Each beam has a known angle.  We only care
//        about beams in a cone in FRONT of the robot (± half of
//        obstacle_sector_deg_).  We find the smallest valid range.
//
// HOW THE MATH WORKS:
//        The LiDAR scans in a circle.  The message tells us:
//          angle_min      — angle of the first beam (radians)
//          angle_increment — angle step between beams (radians)
//        So beam i has angle = angle_min + i * angle_increment.
//        Angle 0 is straight ahead.
//        We skip beams that are NaN (no return) or outside the sensor's
//        valid range (range_min to range_max).
//
//  "msg" is a SharedPtr — a smart pointer.  It automatically frees the
//  message memory when nobody references it anymore.  In ROS 2, all
//  subscription callbacks receive messages this way.
// ===========================================================================
void PatrolNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float min_r = 1e9f;  // Start with a huge number; any real reading will be smaller.

  // Convert the half-sector from degrees to radians.
  // (Computers always work in radians internally; degrees are for humans.)
  const float half = static_cast<float>(obstacle_sector_deg_ * 0.5 * M_PI / 180.0);

  // Loop through every beam in the scan.
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float r = msg->ranges[i];  // Distance for this beam (meters).

    // Skip invalid readings: NaN, infinity, or outside sensor limits.
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
      continue;
    }

    // Calculate the angle of this beam.
    const float ang = msg->angle_min + static_cast<float>(i) * msg->angle_increment;

    // Is this beam within our front cone?
    if (std::abs(ang) <= half) {
      min_r = std::min(min_r, r);  // Keep track of the closest obstacle.
    }
  }

  // Store the result atomically so the main timer thread can read it safely.
  min_scan_range_ = min_r;
}

// ===========================================================================
// joy_cb — joystick override callback
// ===========================================================================
// WHEN:  Called when the joystick state topic publishes a new Bool message.
//
// WHAT:  If msg->data is true, a human is controlling the robot with a
//        joystick.  The main loop will stop sending its own velocity commands
//        and let the human drive.
//
// ".store()" is the thread-safe way to set an atomic bool.
// ===========================================================================
void PatrolNode::joy_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  joy_active_.store(msg->data);
}

// ===========================================================================
// cat_detected_cb — cat detection callback
// ===========================================================================
// WHEN:  Called when an external cat detector node publishes true/false.
//
// WHAT:  Stores whether a cat is currently visible.  The patrol_tick()
//        function checks this and can switch to the CatApproach state.
// ===========================================================================
void PatrolNode::cat_detected_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  cat_detected_.store(msg->data);
}

// ===========================================================================
// detections_cb — email a snapshot when a white/brown cat is recognized
// ===========================================================================
// WHEN:  Called on every Detection2DArray published by cat_detector_cpp.
//
// WHAT:  Mirrors voice_node.py's logic: each Detection2D carries a generic
//        "cat" hypothesis in results[0] and, when the per-cat classifier
//        fired, a white/brown identity in results[1]. We pick the highest-
//        confidence identity across all boxes in the frame and, if it clears
//        cat_recognized_min_conf_, email one snapshot of the current camera
//        frame.
//
// GATING: Only while a patrol cycle is actively running (is_patrolling), and
//        at most once per cat every cat_recognized_cooldown_sec_ seconds so a
//        lingering sighting doesn't email every inference frame.
// ===========================================================================
void PatrolNode::detections_cb(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  // Feature off, or no camera frame to attach yet — nothing to do.
  if (!cat_recognized_mail_enabled_) {
    return;
  }
  // Only react during an active patrol (matches the chosen behavior).
  if (!is_patrolling()) {
    return;
  }

  // Best (highest-confidence) identity across all boxes in this frame.
  std::string best_label;
  double best_conf = -1.0;
  for (const auto & det : msg->detections) {
    if (det.results.size() < 2) {
      continue;  // identification didn't fire for this box
    }
    const auto & id_hyp = det.results[1].hypothesis;
    if ((id_hyp.class_id == "white" || id_hyp.class_id == "brown") &&
        id_hyp.score > best_conf)
    {
      best_label = id_hyp.class_id;
      best_conf = id_hyp.score;
    }
  }

  if (best_label.empty() || best_conf < cat_recognized_min_conf_) {
    return;
  }

  // Per-cat cooldown so one lingering sighting doesn't email every frame.
  const rclcpp::Time tnow = now();
  const auto it = last_cat_mail_.find(best_label);
  if (it != last_cat_mail_.end() &&
      (tnow - it->second).seconds() < cat_recognized_cooldown_sec_)
  {
    return;
  }

  const std::string path = save_snapshot("cat_" + best_label);
  if (path.empty()) {
    return;  // no camera frame available; try again on the next detection
  }

  last_cat_mail_[best_label] = tnow;
  RCLCPP_INFO(get_logger(), "Recognized %s cat (%.2f) — emailing snapshot",
              best_label.c_str(), best_conf);
  send_mail_request({path}, mail_subject_ + " — " + best_label + " cat recognized");
}

// ===========================================================================
// voltage_cb — store battery voltage; warn on low voltage (throttled)
// ===========================================================================
// WHY:  Yahboom X3 motor torque/speed drops with battery voltage. As the
//       pack drains, wheels slip more during in-place rotation, so the
//       robot physically rotates less than odometry/IMU report. This shows
//       up as "circle gets shorter over time" / "turn-around degrades".
// ===========================================================================
void PatrolNode::voltage_cb(const std_msgs::msg::Float32::SharedPtr msg)
{
  const float v = msg->data;
  last_voltage_v_.store(v);
  if (v > 0.1f && v < static_cast<float>(low_voltage_warn_v_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 15000,
      "Low battery voltage: %.2f V (< %.2f V). "
      "Motor torque is reduced; rotations/turn-arounds will physically fall short of odometry.",
      v, low_voltage_warn_v_);
  }
}

// ===========================================================================
// image_cb — camera image callback
// ===========================================================================
// WHEN:  Called every time a new camera frame arrives (e.g., 30 fps).
//
// WHAT:  Just stores a pointer to the latest image.  We DON'T process it
//        here — that would slow down the callback.  Instead, the capture
//        code grabs last_image_ when it needs a snapshot.
//
// ABOUT shared_ptr:
//        "SharedPtr" is a smart pointer that keeps the underlying data
//        alive as long as at least one SharedPtr points to it.  When
//        we overwrite last_image_ with a new message, the OLD message
//        is freed automatically (if nobody else holds a copy).
//        No manual "delete" or "free" needed — C++ RAII handles it.
// ===========================================================================
void PatrolNode::image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
  last_image_ = msg;
}

// ===========================================================================
// depth_image_cb — extract minimum depth in the centre region for obstacle
//                  detection.  Supports 16UC1 (mm) and 32FC1 (m) encodings.
// ===========================================================================
void PatrolNode::depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!msg || msg->data.empty() || msg->width == 0 || msg->height == 0) {
    return;
  }

  // Record when we last received a valid depth frame so the freshness
  // watchdog in depth_obstacle_ahead() can detect a dead camera node.
  last_depth_msg_secs_.store(now().seconds());

  const int rows = static_cast<int>(msg->height);
  const int cols = static_cast<int>(msg->width);
  const int step = static_cast<int>(msg->step);

  const int margin_x = static_cast<int>(cols * (1.0 - depth_sector_width_ratio_) / 2.0);
  const int margin_y = static_cast<int>(rows * (1.0 - depth_sector_width_ratio_) / 2.0);
  const int x0 = margin_x;
  const int x1 = cols - margin_x;
  const int y0 = margin_y;
  const int y1 = rows - margin_y;

  float min_m = 1e9f;

  if (msg->encoding == "16UC1") {
    for (int r = y0; r < y1; ++r) {
      auto row_ptr = reinterpret_cast<const uint16_t *>(msg->data.data() + r * step);
      for (int c = x0; c < x1; ++c) {
        float d = static_cast<float>(row_ptr[c]) * 0.001f;
        if (d > 0.04f && d < min_m) { min_m = d; }
      }
    }
  } else if (msg->encoding == "32FC1") {
    for (int r = y0; r < y1; ++r) {
      auto row_ptr = reinterpret_cast<const float *>(msg->data.data() + r * step);
      for (int c = x0; c < x1; ++c) {
        float d = row_ptr[c];
        if (std::isfinite(d) && d > 0.04f && d < min_m) { min_m = d; }
      }
    }
  }

  min_depth_range_.store(min_m);
}

bool PatrolNode::depth_obstacle_ahead()
{
  // SAFETY: if no depth frame has been received recently (camera dead /
  // node not running), treat as obstacle so the robot does NOT drive blindly.
  const double last_s = last_depth_msg_secs_.load();
  const double now_s = now().seconds();
  if (last_s <= 0.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Depth camera: no frames received yet on '%s' — treating as obstacle (safe stop)",
      depth_image_topic_.c_str());
    return true;
  }
  const double age = now_s - last_s;
  if (age > depth_max_age_sec_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Depth camera stale: last frame %.1fs ago (> %.1fs) on '%s' — safe stop",
      age, depth_max_age_sec_, depth_image_topic_.c_str());
    return true;
  }
  return min_depth_range_.load() < static_cast<float>(depth_obstacle_min_m_);
}

// ===========================================================================
// create_active_pattern — instantiate the patrol pattern selected by param
// ===========================================================================
void PatrolNode::create_active_pattern()
{
  if (patrol_pattern_name_ == "till_obstacle_back_images_turn") {
    auto p = std::make_unique<TillObstacleBackPattern>();
    p->forward_timeout_sec = forward_timeout_sec_;
    p->drive_back_timeout_sec = drive_back_timeout_sec_;
    p->capture_completion_extra_deg = capture_completion_extra_deg_;
    p->log_buffer_period_sec = pattern_log_buffer_period_sec_;
    p->log_file_path = pattern_log_file_path_;
    active_pattern_ = std::move(p);
  } else {
    active_pattern_ = std::make_unique<ClassicPattern>();
  }
  RCLCPP_INFO(get_logger(), "Active patrol pattern: %s", active_pattern_->name().c_str());
}

// ===========================================================================
// build_patrol_context — assemble the context struct for pattern tick calls
// ===========================================================================
PatrolContext PatrolNode::build_patrol_context()
{
  PatrolContext ctx;
  ctx.publish_twist = [this](double lx, double ly, double az) { publish_twist(lx, ly, az); };
  ctx.stop_robot = [this]() { stop_robot(); };
  ctx.get_odom_pose = [this](double & x, double & y, double & yaw) {
    return get_odom_pose(x, y, yaw);
  };
  ctx.depth_obstacle_ahead = [this]() { return depth_obstacle_ahead(); };
  ctx.lidar_obstacle_ahead = [this]() { return obstacle_too_close(); };
  ctx.save_current_image = [this]() { return save_current_image(); };
  ctx.send_mail_request = [this]() { send_mail_request(); };
  ctx.set_buzzer = [this](bool on) { set_buzzer(on); };
  ctx.play_completion_sound = [this]() { play_completion_sound(); };
  ctx.now = [this]() { return now(); };
  ctx.get_logger = [this]() { return get_logger(); };
  ctx.clock = get_clock();

  ctx.home_x = home_x_;
  ctx.home_y = home_y_;
  ctx.home_yaw = home_yaw_;
  ctx.linear_speed = linear_speed_;
  ctx.angular_speed = angular_speed_;
  ctx.pos_tol = pos_tol_;
  ctx.yaw_tol = yaw_tol_;
  ctx.patrol_drive_sec = patrol_drive_sec_;
  ctx.capture_frame_count = capture_frame_count_;
  ctx.capture_turn_speed = capture_turn_speed_;
  ctx.capture_rotate_sec = capture_rotate_sec_;
  ctx.capture_kp = capture_kp_;
  ctx.capture_min_angular_speed = capture_min_angular_speed_;
  ctx.capture_yaw_tolerance = capture_yaw_tolerance_;
  return ctx;
}

// ===========================================================================
// escape_json — make a string safe to embed inside a JSON string
// ===========================================================================
// WHAT:  Adds backslashes before " and \ characters.
//
// WHY:   When we build JSON by hand (for the mail request), a file path
//        like C:\photos or a subject like He said "hi" would break the
//        JSON syntax.  Escaping them makes the JSON valid.
//
// EXAMPLE:
//   escape_json("hello \"world\"")  =>  "hello \\\"world\\\""
//   escape_json("/tmp/photo.jpg")   =>  "/tmp/photo.jpg"  (unchanged)
//
// "const" at the end means this function doesn't modify any member variables.
// ===========================================================================
std::string PatrolNode::escape_json(const std::string & s) const
{
  std::string out;
  out.reserve(s.size() + 8);  // Pre-allocate roughly enough memory.
  for (char c : s) {                // Loop over each character in the string.
    if (c == '"' || c == '\\') {    // These characters need escaping in JSON.
      out += '\\';                  // Add a backslash before the special char.
    }
    out += c;                       // Add the character itself.
  }
  return out;
}

// ===========================================================================
// send_mail_request — tell the mail_node to email the captured photos
// ===========================================================================
// WHAT:  Builds a JSON string like:
//        {"subject":"Cat patrol photos","to":"you@email.com",
//         "paths":["/tmp/cat_patrol_images/snap_1234_0.jpg", ...]}
//        and publishes it on the mail_request_topic.
//
// WHY:   This C++ node doesn't do email itself — that's complex, needs
//        SMTP libraries, credentials, etc.  Instead, we publish a simple
//        request, and the Python mail_node.py handles the email sending.
//        This is a common ROS pattern: split responsibilities between nodes.
//
// HOW:
//        std::ostringstream (oss) is like a string builder: you write into
//        it with "<<" and then call .str() to get the result.
// ===========================================================================
void PatrolNode::send_mail_request()
{
  // The default request emails the 360° capture batch with the configured
  // subject. Everything is delegated to the explicit-arg overload below.
  send_mail_request(saved_paths_, mail_subject_);
}

void PatrolNode::send_mail_request(
  const std::vector<std::string> & paths, const std::string & subject)
{
  std::ostringstream oss;
  // Build the JSON object piece by piece.
  oss << "{\"subject\":\"" << escape_json(subject) << "\","
      << "\"to\":\"" << escape_json(mail_to_) << "\","
      << "\"paths\":[";
  for (size_t i = 0; i < paths.size(); ++i) {
    if (i > 0) {
      oss << ",";  // JSON arrays need commas between elements.
    }
    oss << "\"" << escape_json(paths[i]) << "\"";
  }
  oss << "]}";

  // Wrap the JSON string in a ROS String message and publish it.
  std_msgs::msg::String out;
  out.data = oss.str();
  mail_pub_->publish(out);
  RCLCPP_INFO(get_logger(), "Published mail request \"%s\" (%zu attachments)",
              subject.c_str(), paths.size());
}

// ===========================================================================
// patrol_timer_cb — THE MAIN LOOP (called 20 times per second)
// ===========================================================================
// WHAT:  This is the "brain" of the robot.  Every tick (50 ms), it checks
//        the current state and calls the right function.
//
// WHY:   A single periodic callback is simpler than having separate timers
//        for each state.  It's easy to reason about: "every tick, the robot
//        does exactly one thing based on its current state."
//
// THE STATE MACHINE:
//   WaitingForTf — keep checking if TF works; once it does, go to Patrol
//   Idle         — do nothing; the idle_cycle_timer will wake us up
//   Patrol       — drive back and forth; check for cats
//   Capture      — take panoramic photos
//   ReturnHome   — navigate back to starting position
//   CatApproach  — creep toward detected cat
//
// JOYSTICK SAFETY:
//   If a human is using the joystick (joy_active_ is true), we IMMEDIATELY
//   stop the robot and skip all autonomous logic.  Human safety first!
// ===========================================================================
void PatrolNode::patrol_timer_cb()
{
  // Safety first: if joystick override is active, stop everything.
  if (joy_active_.load()) {
    stop_robot();
    return;  // Skip the state machine entirely.
  }

  // "switch" is like a series of "if (state_ == X)" checks, but faster
  // and cleaner.  It jumps directly to the matching "case".
  switch (state_) {
    case PatrolState::WaitingForTf: {
      // Wait at least 5 seconds before accepting TF.  The chassis driver
      // (Mcnamu_driver_X3) needs time to open the serial port and start
      // publishing fresh odom.  Without this delay the TF buffer may
      // return a STALE transform from a previous run, causing the node
      // to record a wrong home position.
      const double waited = (now() - state_enter_time_).seconds();
      if (waited < 5.0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for drivers to start (%.0f/5s)...", waited);
        break;
      }
      double x, y, yaw;
      if (get_odom_pose(x, y, yaw)) {
        RCLCPP_INFO(get_logger(), "TF available — starting patrol");
        transition_to(PatrolState::Patrol);
      }
      break;
    }
    case PatrolState::Idle:
      // Do nothing.  The idle_cycle_timer (set up in constructor) will
      // call transition_to(Patrol) when it's time for the next cycle.
      break;
    case PatrolState::Patrol:
      // If cat detection is enabled and a cat was detected, interrupt
      // the patrol and go investigate.
      if (cat_detection_enabled_ && cat_detected_.load()) {
        transition_to(PatrolState::CatApproach);
        return;
      }
      patrol_tick();  // Normal patrol driving logic.
      break;
    case PatrolState::Capture:
      capture_tick();  // Photo-taking logic.
      break;
    case PatrolState::ReturnHome:
      return_home_tick();  // Navigate-back-to-start logic.
      break;
    case PatrolState::CatApproach:
      cat_approach_tick();  // Creep-toward-cat logic.
      break;
  }
}

// ===========================================================================
// patrol_tick — drive back and forth (the patrol pattern)
// ===========================================================================
// WHAT:  Makes the robot drive forward, then backward, then forward, then
//        backward — four "legs" total.  After all four legs, transitions
//        to Capture (take photos).
//
// HOW:   This is "open-loop" control — we drive for a set TIME, not a set
//        DISTANCE.  We don't measure how far we actually went.  This is
//        simple but imprecise (wheel slip, battery voltage, surface
//        friction all affect actual distance).
//
// THE PATTERN:
//   Time 0 to leg       → forward   (phase 0, even → forward)
//   Time leg to 2*leg   → backward  (phase 1, odd  → backward)
//   Time 2*leg to 3*leg → forward   (phase 2, even → forward)
//   Time 3*leg to 4*leg → backward  (phase 3, odd  → backward)
//
// OBSTACLE SAFETY:
//   If LiDAR detects something too close, we stop and wait.  Once the
//   obstacle moves away, we resume.  The time keeps ticking, so the
//   total patrol gets shorter (not extended).
// ===========================================================================
void PatrolNode::patrol_tick()
{
  auto ctx = build_patrol_context();
  PatrolSignal sig = active_pattern_->tick(ctx);

  switch (sig) {
    case PatrolSignal::Continue:
      break;
    case PatrolSignal::DoneNextCapture:
      transition_to(PatrolState::Capture);
      break;
    case PatrolSignal::DoneIdle:
      stop_robot();
      transition_to(PatrolState::Idle);
      reset_idle_loop();
      if (exit_after_one_cycle_) {
        RCLCPP_INFO(get_logger(), "exit_after_one_cycle: shutting down ROS");
        rclcpp::shutdown();
      }
      break;
  }
}

// ===========================================================================
// capture_tick — take a panoramic set of photos
// ===========================================================================
// WHAT:  This is a mini state machine INSIDE the Capture state, with three
//        sub-phases that repeat:
//
//        [Snap] → [Rotate] → [Settle] → [Snap] → [Rotate] → ...
//
//        1. Snap:    Save the current camera image to a JPEG file.
//        2. Rotate:  Spin in place for capture_rotate_sec_ seconds.
//        3. Settle:  Wait 0.8 seconds for the image to stop being blurry
//                    (the camera needs a moment after the robot stops turning).
//
//        After taking capture_frame_count_ photos (e.g., 12), we send the
//        email and move to ReturnHome.
//
// SAFETY:
//        A 90-second timeout prevents infinite looping if the camera never
//        sends an image (broken cable, wrong topic, etc.).
// ===========================================================================
void PatrolNode::capture_tick()
{
  const auto tnow = now();  // Current time (snapshot — used multiple times below).

  // Safety timeout: if we've been in Capture for > 90 seconds, give up.
  const double total_elapsed = (tnow - capture_phase_start_).seconds();
  if (total_elapsed > 90.0) {
    RCLCPP_WARN(get_logger(), "Capture phase timeout after %.0fs", total_elapsed);
    stop_robot();
    send_mail_request();  // Send whatever photos we DID manage to capture.
    transition_to(PatrolState::ReturnHome);
    return;
  }

  // Have we taken enough photos?
  if (frames_saved_ >= capture_frame_count_) {
    stop_robot();
    RCLCPP_INFO(get_logger(), "Capture complete — %d images taken in %.1fs",
                frames_saved_, total_elapsed);
    send_mail_request();  // Email all the photos.
    transition_to(PatrolState::ReturnHome);
    return;
  }

  // Sub-phase durations.
  const double rotate_sec = capture_rotate_sec_;  // How long to rotate between snaps.
  const double settle_sec = 0.8;                  // How long to wait after rotating.

  // Which sub-phase are we in?
  switch (capture_phase_) {
    case CapturePhase::Snap: {
      // Check if we have a valid camera image to save.
      if (!last_image_ || last_image_->data.empty()) {
        // No image yet — warn (but not too often) and wait for next tick.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "No image received yet on %s", image_topic_.c_str());
        return;
      }
      // Try to save the current image to disk as a JPEG.
      if (save_current_image()) {
        set_buzzer(true);
        RCLCPP_INFO(get_logger(), "Captured [%d/%d]", frames_saved_, capture_frame_count_);
        // If we need more photos, start rotating to the next angle.
        if (frames_saved_ < capture_frame_count_) {
          capture_phase_ = CapturePhase::Rotate;
          // Calculate when to stop rotating (current time + rotate duration).
          capture_rotate_end_ = tnow + rclcpp::Duration::from_seconds(rotate_sec);
          RCLCPP_INFO(get_logger(), "Rotating for %.1fs at %.1f rad/s",
                      rotate_sec, capture_turn_speed_);
        }
      }
      break;
    }
    case CapturePhase::Rotate: {
      set_buzzer(false);
      // Are we done rotating?
      if (tnow >= capture_rotate_end_) {
        stop_robot();  // Stop spinning.
        capture_phase_ = CapturePhase::Settle;
        // Calculate when the settle period ends.
        capture_settle_end_ = tnow + rclcpp::Duration::from_seconds(settle_sec);
        RCLCPP_INFO(get_logger(), "Rotation done, settling %.1fs", settle_sec);
      } else {
        // Still rotating: spin in place.
        // angular.z positive = counter-clockwise (left) in ROS convention.
        publish_twist(0.0, 0.0, capture_turn_speed_);
      }
      break;
    }
    case CapturePhase::Settle: {
      // Just wait — don't move.  Let the camera image stabilize.
      if (tnow >= capture_settle_end_) {
        capture_phase_ = CapturePhase::Snap;  // Ready for the next photo!
      }
      break;
    }
  }
}

// ===========================================================================
// save_current_image — convert a ROS Image message to JPEG and save to disk
// ===========================================================================
// WHAT:  Takes the latest camera frame (stored as raw bytes in a ROS Image
//        message), converts it to an OpenCV image (cv::Mat), and writes it
//        as a JPEG file.
//
// WHY:   ROS Image messages are raw pixel arrays with an encoding label
//        (like "rgb8", "bgr8", "mono8").  JPEG files need BGR format.
//        OpenCV's cv::imwrite() handles JPEG compression for us.
//
// ENCODING CONVERSIONS:
//   "rgb8"  — Red/Green/Blue, 8 bits each.  Must convert to BGR for OpenCV.
//   "bgr8"  — Blue/Green/Red, 8 bits each.  OpenCV's native format.
//   "mono8" — Grayscale, 8 bits.  Convert to BGR (3-channel grayscale).
//   "16UC1" / "mono16" — 16-bit depth.  Scale down to 8-bit for display.
//
// RETURNS: true if save succeeded, false otherwise.
//
// ABOUT const_cast:
//   OpenCV's cv::Mat constructor wants a non-const pointer, but the ROS
//   message data is const.  const_cast removes the const.  This is safe
//   because we only READ the data (we don't modify it).  This is a common
//   pattern when interfacing ROS with OpenCV.
// ===========================================================================
bool PatrolNode::last_image_to_bgr(cv::Mat & out)
{
  if (!last_image_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "last_image_to_bgr: last_image_ is null — no frames received on topic '%s'. "
      "Check camera is publishing color images.", image_topic_.c_str());
    return false;
  }
  if (last_image_->width == 0 || last_image_->height == 0 || last_image_->data.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "last_image_to_bgr: image has no pixel data (w=%u h=%u data_size=%zu)",
      last_image_->width, last_image_->height, last_image_->data.size());
    return false;
  }

  cv::Mat bgr_image;  // This will hold the final BGR image for saving.
  try {
    // Extract image metadata from the ROS message.
    const std::string & enc = last_image_->encoding;     // e.g. "rgb8"
    const int rows = static_cast<int>(last_image_->height);  // Image height in pixels.
    const int cols = static_cast<int>(last_image_->width);   // Image width in pixels.
    const int step = static_cast<int>(last_image_->step);    // Bytes per row (including padding).

    // Convert based on encoding.
    if (enc == "rgb8") {
      // Camera sends RGB; OpenCV needs BGR.  Swap red and blue channels.
      cv::Mat rgb(rows, cols, CV_8UC3, const_cast<uint8_t*>(last_image_->data.data()), step);
      cv::cvtColor(rgb, bgr_image, cv::COLOR_RGB2BGR);
    } else if (enc == "bgr8") {
      // Already BGR — just clone (copy) the data.
      cv::Mat src(rows, cols, CV_8UC3, const_cast<uint8_t*>(last_image_->data.data()), step);
      bgr_image = src.clone();
    } else if (enc == "mono8") {
      // Grayscale → BGR (all three channels get the same gray value).
      cv::Mat mono(rows, cols, CV_8UC1, const_cast<uint8_t*>(last_image_->data.data()), step);
      cv::cvtColor(mono, bgr_image, cv::COLOR_GRAY2BGR);
    } else if (enc == "16UC1" || enc == "mono16") {
      // 16-bit depth image (e.g., from a depth camera).  Scale down to
      // 8-bit so we can save it as a visible JPEG.
      // The 255.0/1000.0 factor assumes max depth ≈ 1000 (millimeters).
      cv::Mat raw16(rows, cols, CV_16UC1, const_cast<uint8_t*>(last_image_->data.data()), step);
      cv::Mat mono8;
      raw16.convertTo(mono8, CV_8UC1, 255.0 / 1000.0);
      cv::cvtColor(mono8, bgr_image, cv::COLOR_GRAY2BGR);
    } else {
      // Unknown encoding — we can't handle it.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "Unsupported encoding '%s'", enc.c_str());
      return false;
    }
  } catch (const std::exception & e) {
    // If anything goes wrong during conversion, log and bail out.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Image conversion: %s", e.what());
    return false;
  }

  // If the conversion produced an empty result, bail out.
  if (bgr_image.empty()) {
    return false;
  }

  out = bgr_image;
  return true;
}

bool PatrolNode::save_current_image()
{
  cv::Mat bgr_image;
  if (!last_image_to_bgr(bgr_image)) {
    return false;
  }

  // Build the file path: /tmp/cat_patrol_images/snap_<timestamp>_<count>.jpg
  std::filesystem::path dir(image_save_dir_);
  std::ostringstream fname;
  fname << "snap_" << now().nanoseconds() << "_" << frames_saved_ << ".jpg";
  std::filesystem::path fp = dir / fname.str();  // The "/" operator joins paths.

  // cv::imwrite compresses the image to JPEG and saves it to disk.
  if (cv::imwrite(fp.string(), bgr_image)) {
    saved_paths_.push_back(fp.string());  // Remember the path for the email.
    frames_saved_++;                       // Increment our photo counter.
    return true;
  }
  return false;  // imwrite failed (disk full, permissions, etc.)
}

// save_snapshot — save the current frame as a standalone "<tag>_<ts>.jpg".
// Unlike save_current_image, it does NOT append to saved_paths_ or bump
// frames_saved_ — those belong to the 360° capture batch, which the
// loop-boundary and cat-recognized emails must not disturb.
std::string PatrolNode::save_snapshot(const std::string & tag)
{
  cv::Mat bgr_image;
  if (!last_image_to_bgr(bgr_image)) {
    return "";
  }

  std::filesystem::path dir(image_save_dir_);
  std::ostringstream fname;
  fname << tag << "_" << now().nanoseconds() << ".jpg";
  std::filesystem::path fp = dir / fname.str();

  if (cv::imwrite(fp.string(), bgr_image)) {
    return fp.string();
  }
  RCLCPP_WARN(get_logger(), "save_snapshot: cv::imwrite failed for %s", fp.string().c_str());
  return "";
}

// is_patrolling — true whenever a patrol cycle is actively running. Used to
// gate the cat-recognized email so it never fires while the robot is idle.
bool PatrolNode::is_patrolling() const
{
  return state_ == PatrolState::Patrol || state_ == PatrolState::Capture ||
         state_ == PatrolState::ReturnHome || state_ == PatrolState::CatApproach;
}

// ===========================================================================
// return_home_tick — navigate back to where the patrol started
// ===========================================================================
// WHAT:  Uses a simple two-stage controller to drive the robot back to
//        the (home_x_, home_y_, home_yaw_) position recorded at the
//        start of the patrol.
//
// HOW THE NAVIGATION WORKS (step by step):
//
//   1. Read current pose from TF (where am I now?).
//   2. Calculate distance to home: dist = sqrt(dx^2 + dy^2)
//   3. If close enough (dist < pos_tol_):
//      a. Calculate heading error (how much do I need to turn to face
//         the same direction as when I started?).
//      b. If heading is close enough, we're done → go to Idle.
//      c. Otherwise, turn in place to align.
//   4. If NOT close enough:
//      a. Calculate the angle FROM me TO home: atan2(dy, dx).
//      b. Calculate how much I need to turn: turn = aim - current_yaw.
//      c. If I need to turn a lot, just turn (don't drive forward while
//         turning sharply — that would make a wide arc).
//      d. If I'm roughly facing home, drive forward AND turn slightly.
//
// ABOUT ODOMETRY DRIFT:
//   The robot tracks its position using wheel encoders (counting wheel
//   rotations).  Over time, small errors accumulate: the robot thinks
//   it's at (1.0, 0.0) but it's really at (1.05, -0.03).  This is
//   "drift."  The pos_tol_ (0.15 m) and yaw_tol_ (0.2 rad ≈ 11°)
//   tolerances account for this — we don't need to be pixel-perfect.
//
// TIMEOUT:
//   If we can't get home within return_timeout_sec_ (120s), give up
//   and go to Idle.  This prevents the robot from driving forever
//   if something goes wrong.
// ===========================================================================
void PatrolNode::return_home_tick()
{
  // Have we been trying to go home for too long?
  if ((now() - state_enter_time_).seconds() > return_timeout_sec_) {
    RCLCPP_WARN(get_logger(), "Return home timeout");
    stop_robot();
    transition_to(PatrolState::Idle);
    reset_idle_loop();  // Turn off the buzzer if it was on.
    if (exit_after_one_cycle_) {
      rclcpp::shutdown();  // Terminate the entire node.
    }
    return;
  }

  // Where am I right now?
  double x, y, yaw;
  if (!get_odom_pose(x, y, yaw)) {
    stop_robot();  // Can't read pose — stop and try again next tick.
    return;
  }

  // Vector from current position to home.
  const double dx = home_x_ - x;   // How far home is in x (forward/back).
  const double dy = home_y_ - y;   // How far home is in y (left/right).
  // Euclidean distance = √(dx² + dy²).  This is the straight-line distance.
  const double dist = std::hypot(dx, dy);

  // Log progress (throttled to once per 2 seconds).
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "RETURN dist_to_home=%.3f pos=(%.3f,%.3f) home=(%.3f,%.3f)",
    dist, x, y, home_x_, home_y_);

  // --- STAGE 1: Are we close enough to home in POSITION? ---
  if (dist < pos_tol_) {
    // Yes! Now just fix the heading.
    // ey = heading error: how much to turn to match the original heading.
    const double ey = normalize_angle(home_yaw_ - yaw);
    if (std::abs(ey) < yaw_tol_) {
      // Heading is close enough too — we're home!
      stop_robot();
      RCLCPP_INFO(get_logger(), "Returned home");
      transition_to(PatrolState::Idle);
      reset_idle_loop();
      if (exit_after_one_cycle_) {
        RCLCPP_INFO(get_logger(), "exit_after_one_cycle: shutting down ROS");
        rclcpp::shutdown();
      }
      return;
    }
    // Turn in place toward the desired heading.
    // copysign(speed, error) gives us +speed or -speed depending on which
    // way we need to turn.
    const double w = std::copysign(angular_speed_, ey);
    publish_twist(0.0, 0.0, w);  // No forward motion, just turning.
    return;
  }

  // --- STAGE 2: Drive toward home ---

  // What angle do I need to face to point at home?
  // atan2(dy, dx) gives the angle of the (dx, dy) vector in radians.
  // It correctly handles all quadrants (unlike atan which has blind spots).
  const double aim = std::atan2(dy, dx);

  // How much do I need to turn?  (Desired heading minus current heading.)
  const double turn = normalize_angle(aim - yaw);

  // Proportional angular velocity: turn faster when the error is large,
  // slower when nearly aligned.  "clamp" limits the value to [-angular_speed_, +angular_speed_].
  // capture_kp_ is the proportional gain — makes turning more responsive.
  double w = std::clamp(turn * capture_kp_, -angular_speed_, angular_speed_);

  // Forward speed: drive at 70% of max speed (conservative to allow corrections).
  double v = std::clamp(linear_speed_ * 0.7, 0.0, linear_speed_);

  // If we need to turn a lot (more than ~22 degrees), stop forward motion
  // and just turn.  This prevents the robot from driving in a wide arc
  // when it's facing the wrong way.
  if (std::abs(turn) > yaw_tol_ * 2.0) {
    v = 0.0;  // Pure rotation — no forward movement.
  }

  publish_twist(v, 0.0, w);
}

// ===========================================================================
// reset_idle_loop — clean up when going back to idle
// ===========================================================================
// Turns the buzzer off.  The buzzer might have been turned on during
// CatApproach, and we don't want it beeping forever.
// ===========================================================================
void PatrolNode::reset_idle_loop()
{
  std_msgs::msg::Bool b;
  b.data = false;          // false = buzzer OFF
  buzzer_pub_->publish(b);
}

// ===========================================================================
// cat_approach_tick — slowly creep toward a detected cat
// ===========================================================================
// WHAT:  When cat_detection_enabled_ is on and a cat is detected during
//        patrol, the robot enters this state.  It:
//        - Creeps forward slowly (approach_linear_ speed).
//        - Checks for obstacles (safety stop).
//        - Times out after approach_max_sec_ (don't chase forever).
//        - Goes back to Patrol if the cat disappears.
//
// THE BUZZER:
//   The buzzer was turned ON when we entered CatApproach (in transition_to).
//   It gets turned OFF when we leave this state (timeout or cat lost).
//   This provides an audible alert: "the robot found something!"
// ===========================================================================
void PatrolNode::cat_approach_tick()
{
  // Have we been approaching for too long?
  if ((now() - approach_start_).seconds() > approach_max_sec_) {
    RCLCPP_WARN(get_logger(), "Cat approach timeout");
    std_msgs::msg::Bool b;
    b.data = false;
    buzzer_pub_->publish(b);  // Turn buzzer off.
    transition_to(PatrolState::Patrol);  // Go back to normal patrolling.
    return;
  }

  // Safety: if obstacle too close, stop (but stay in this state).
  if (obstacle_too_close()) {
    stop_robot();
    return;
  }

  // Creep forward slowly toward the cat.
  publish_twist(approach_linear_, 0.0, 0.0);

  // If the cat is no longer detected, stop and go back to patrol.
  if (!cat_detected_.load()) {
    stop_robot();
    std_msgs::msg::Bool b;
    b.data = false;
    buzzer_pub_->publish(b);  // Turn buzzer off.
    transition_to(PatrolState::Patrol);
  }
}

}  // namespace cat_patrol_robot
// (End of the namespace — all PatrolNode code above belongs to cat_patrol_robot.)

// ===========================================================================
// main — the program entry point
// ===========================================================================
// WHAT:  Every C++ program starts here.  This function:
//        1. Initializes the ROS 2 system (rclcpp::init).
//        2. Creates our PatrolNode.
//        3. Spins (processes callbacks) until told to stop.
//        4. Cleans up (rclcpp::shutdown).
//
// "rclcpp::spin(node)" is a blocking call — it sits in a loop, waiting
// for timer events and incoming messages, and dispatches callbacks.
// It only returns when someone calls rclcpp::shutdown() (which happens
// on Ctrl+C, or when exit_after_one_cycle triggers it, or on error).
//
// "argc" and "argv" are the command-line arguments passed to the program.
// ROS uses them for things like node name remapping:
//   ./patrol_node --ros-args -r __node:=my_custom_name
// ===========================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                                    // Step 1: Start ROS 2
  auto node = std::make_shared<cat_patrol_robot::PatrolNode>(); // Step 2: Create the node
  rclcpp::spin(node);                                          // Step 3: Run until shutdown
  rclcpp::shutdown();                                          // Step 4: Clean up
  return 0;                                                    // 0 = success
}
