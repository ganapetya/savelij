// =============================================================================
// patrol_node.hpp — Declaration of the PatrolNode class
// =============================================================================
//
// WHAT IS THIS FILE?
//   This is the HEADER for the patrol robot's main ROS 2 node.  It declares:
//     - The PatrolState enum (the FSM states)
//     - The PatrolNode class (all methods and member variables)
//
//   The METHOD BODIES live in patrol_node.cpp.  This header tells the
//   compiler "what exists" so other files can refer to PatrolNode.
//
// HOW TO READ A C++ HEADER:
//   1. Look at the #includes — they tell you which libraries this class uses.
//   2. Look at public methods — this is the class's "API" (how you use it).
//   3. Look at private methods — internal helpers, not accessible from outside.
//   4. Look at member variables — the class's "memory" (state it carries).
//
// C++ CONCEPT: CLASS = DATA + BEHAVIOR
//   A class bundles VARIABLES (data) and FUNCTIONS (behavior) together.
//   Variables declared inside a class are called "member variables" or "fields."
//   Functions are called "member functions" or "methods."
//   Each PatrolNode object has its OWN copy of all member variables.
// =============================================================================
#ifndef CAT_PATROL_ROBOT__PATROL_NODE_HPP_
#define CAT_PATROL_ROBOT__PATROL_NODE_HPP_

// ===========================================================================
// Standard Library includes
// ===========================================================================
#include <algorithm>   // std::clamp, std::min, std::max
#include <atomic>      // std::atomic — thread-safe variables for sensor callbacks
#include <thread>      // std::thread — background Bluetooth connect on startup
#include <cmath>       // M_PI, std::abs, std::hypot, atan2, etc.
#include <memory>      // std::unique_ptr, std::shared_ptr — smart pointers
#include <string>      // std::string
#include <unordered_map>  // std::unordered_map — per-cat mail cooldown timestamps
#include <vector>      // std::vector — growable array

// OpenCV cv::Mat is used by the image-conversion helper declared below.
#include <opencv2/core.hpp>

// ===========================================================================
// ROS 2 message type includes
// ===========================================================================
// Each ROS 2 message type is auto-generated from a .msg file into a C++
// struct.  The naming convention is:
//   package_name/msg/message_name.hpp
//   → package_name::msg::MessageName (the C++ type)
//
// geometry_msgs::msg::Twist — velocity command: linear (x,y,z) + angular (x,y,z)
// sensor_msgs::msg::Image  — camera frame: raw pixel data + encoding + metadata
// sensor_msgs::msg::LaserScan — LiDAR scan: array of distances at known angles
// std_msgs::msg::Bool     — simple true/false
// std_msgs::msg::String   — UTF-8 text string
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
// vision_msgs::msg::Detection2DArray — cat_detector_cpp's per-frame detections;
// each box carries a generic "cat" hypothesis plus (when the per-cat classifier
// fired) a second white/brown identity hypothesis in results[1].
#include "vision_msgs/msg/detection2_d_array.hpp"

// ===========================================================================
// TF2 (Transform Framework) includes
// ===========================================================================
// TF2 is ROS 2's system for tracking coordinate frames.  It answers:
//   "Where is frame B relative to frame A, at time T?"
//
// tf2_ros::Buffer          — stores recent transforms (a short history)
// tf2_ros::TransformListener — subscribes to /tf and fills the buffer
//
// We use TF to answer: "where is base_footprint relative to odom?"
// → gives us the robot's (x, y, yaw) position from wheel odometry.
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Our own header: the PatrolPattern interface and PatrolContext struct
#include "cat_patrol_robot/patterns/patrol_pattern.hpp"

namespace cat_patrol_robot
{

// ===========================================================================
// PatrolState — the node-level finite state machine
// ===========================================================================
// This enum represents the TOP-LEVEL states of the patrol robot.
// The node transitions between them via transition_to().
//
//   WaitingForTf → Patrol → (pattern runs) → Capture → ReturnHome → Idle
//                       ↘ CatApproach ↗                              ↑
//                                                     (idle timer fires)─┘
//
// Note: when using the "till_obstacle_back_images_turn" pattern, the
// Capture and ReturnHome states are handled INSIDE the pattern, so the
// node goes directly from Patrol → Idle.
enum class PatrolState
{
  WaitingForTf,   // Startup: waiting for TF (odom → base_footprint) to appear
  Idle,           // Doing nothing; waiting for the next patrol cycle timer
  Patrol,         // Active patrol: the selected pattern is running
  Capture,        // Taking panoramic photos (used by "classic" pattern path)
  ReturnHome,     // Navigating back to starting position (classic path)
  CatApproach,    // Creeping toward a detected cat (optional)
};

// ===========================================================================
// PatrolNode — the main ROS 2 node class
// ===========================================================================
// C++ CONCEPT: INHERITANCE FROM rclcpp::Node
//   Every ROS 2 C++ node inherits from rclcpp::Node.  This gives us:
//     - create_publisher<T>(topic, qos)       — send messages
//     - create_subscription<T>(topic, qos, callback) — receive messages
//     - create_wall_timer(period, callback)   — periodic function calls
//     - declare_parameter<T>(name, default)   — configurable settings
//     - get_logger() / now() / get_clock()    — utilities
//
// C++ CONCEPT: "explicit" CONSTRUCTOR
//   "explicit" prevents the compiler from using this constructor for
//   IMPLICIT type conversions.  Without it, writing:
//     PatrolNode node = some_options_object;
//   would silently create a PatrolNode.  With "explicit," you must write:
//     PatrolNode node(some_options_object);
//   This prevents accidental, confusing conversions.
//
// C++ CONCEPT: DEFAULT ARGUMENT ("= rclcpp::NodeOptions()")
//   If no argument is provided, a default-constructed NodeOptions is used.
//   This lets you write: auto node = PatrolNode();  (no options needed)
// ===========================================================================
class PatrolNode : public rclcpp::Node
{
public:
  explicit PatrolNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // =========================================================================
  // Timer and subscription callbacks
  // =========================================================================
  // These are called AUTOMATICALLY by the ROS 2 executor when a timer fires
  // or a message arrives.  They're "event handlers" — you don't call them
  // yourself.
  //
  // C++ CONCEPT: CALLBACK FUNCTIONS
  //   A callback is a function you REGISTER to be called LATER when
  //   something happens (a timer fires, a message arrives, etc.).
  //   ROS 2 takes a std::function (or bound method) and stores it.
  //   The executor loop calls it when the event occurs.

  void patrol_timer_cb();        // 20 Hz heartbeat: runs the state machine
  void image_cb(const sensor_msgs::msg::Image::SharedPtr msg);       // Color camera frames
  void depth_image_cb(const sensor_msgs::msg::Image::SharedPtr msg); // Depth camera frames
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);    // LiDAR scans
  void joy_cb(const std_msgs::msg::Bool::SharedPtr msg);             // Joystick override
  void cat_detected_cb(const std_msgs::msg::Bool::SharedPtr msg);    // External cat detector
  // cat_detector_cpp identity stream: emails a snapshot when a white/brown
  // cat is recognized mid-patrol (see cat_recognized_* parameters).
  void detections_cb(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void voltage_cb(const std_msgs::msg::Float32::SharedPtr msg);      // Battery voltage

  // =========================================================================
  // State machine and motion methods
  // =========================================================================
  void transition_to(PatrolState s);    // Switch FSM state (always stops robot first)
  bool get_odom_pose(double & x, double & y, double & yaw);  // TF lookup: robot pose
  void publish_twist(double lin_x, double lin_y, double ang_z); // Send velocity command
  void stop_robot();                    // Publish zero velocity
  bool obstacle_too_close();            // LiDAR obstacle check
  bool depth_obstacle_ahead();          // Depth camera obstacle check

  // =========================================================================
  // Per-state tick functions (called from patrol_timer_cb)
  // =========================================================================
  void patrol_tick();          // Delegates to active_pattern_->tick()
  void capture_tick();         // Panoramic photo-taking (classic pattern path)
  void return_home_tick();     // Navigate back to home position
  void cat_approach_tick();    // Creep toward detected cat

  // =========================================================================
  // Utility methods
  // =========================================================================
  bool last_image_to_bgr(cv::Mat & out);  // Decode last_image_ → BGR cv::Mat (shared by the savers)
  bool save_current_image();              // Convert ROS Image → JPEG on disk (adds to saved_paths_)
  // Save the current camera frame to a standalone JPEG named "<tag>_<ts>.jpg"
  // WITHOUT touching saved_paths_/frames_saved_ (those belong to the 360°
  // capture batch). Returns the file path, or "" if no frame was saved.
  std::string save_snapshot(const std::string & tag);
  void send_mail_request();               // Publish JSON mail request for the 360° capture batch
  // Publish a JSON mail request for an explicit set of attachment paths and
  // subject — used for the single-snapshot loop-boundary and cat-recognized
  // emails. The no-arg overload above delegates here with saved_paths_.
  void send_mail_request(const std::vector<std::string> & paths, const std::string & subject);
  // True while a patrol cycle is actively running (i.e. not Idle/WaitingForTf).
  bool is_patrolling() const;
  void set_buzzer(bool on);               // Turn onboard buzzer on or off
  void play_completion_sound();           // Spawn audio player in background (no-op if path empty)
  void start_bluetooth_connect_async();   // Background: pair RX-V485, route PulseAudio, play bark
  bool connect_bluetooth_speaker();       // Blocking connect; returns true if bluez sink ready
  std::string resolve_bt_connect_script() const;
  std::string escape_json(const std::string & s) const;  // Escape " and \ for JSON
  void reset_idle_loop();                 // Turn off buzzer when going idle

  // Build the PatrolContext struct passed to pattern strategies
  PatrolContext build_patrol_context();
  // Instantiate the patrol pattern selected by the "patrol_pattern" parameter
  void create_active_pattern();

  // =========================================================================
  // ROS 2 Parameters (loaded from YAML, configurable without recompiling)
  // =========================================================================
  // ROS 2 CONCEPT: PARAMETERS
  //   Parameters are runtime settings declared with declare_parameter().
  //   Their values come from a YAML config file, launch file arguments,
  //   or the command line.  You can even change some while the node runs
  //   using: ros2 param set /patrol_node linear_speed 0.3
  //
  //   Each member variable below is filled from its corresponding parameter
  //   in the constructor.  The naming convention: trailing underscore (_)
  //   marks member variables (distinguishes them from local variables).

  // Topic names (which ROS topics this node uses)
  std::string cmd_vel_topic_;         // Velocity commands to motor driver
  std::string image_topic_;           // Color camera input
  std::string depth_image_topic_;     // Depth camera input
  std::string scan_topic_;            // LiDAR input
  std::string buzzer_topic_;          // Buzzer on/off output
  std::string joy_topic_;             // Joystick override input
  std::string odom_frame_;            // TF frame: odometry origin
  std::string base_frame_;            // TF frame: robot center
  std::string image_save_dir_;        // Directory for saved JPEGs
  std::string mail_request_topic_;    // JSON mail request output
  std::string mail_subject_;          // Email subject line
  std::string mail_to_;               // Email recipient
  std::string cat_detected_topic_;    // Cat detection input
  std::string detections_topic_;      // cat_detector_cpp Detection2DArray input (white/brown identity)
  std::string patrol_pattern_name_;   // Which strategy to use ("classic" etc.)

  // Behavior tuning (numeric parameters from YAML)
  bool use_lidar_{false};                  // Enable LiDAR obstacle stopping?
  double obstacle_min_range_{0.35};        // LiDAR: stop if closer than this (m)
  double obstacle_sector_deg_{60.0};       // LiDAR: front cone width (degrees)
  double depth_obstacle_min_m_{0.50};      // Depth camera: stop threshold (m)
  double depth_sector_width_ratio_{0.33};  // Depth camera: central region ratio
  double forward_timeout_sec_{60.0};       // Max time to drive forward (s)
  double drive_back_timeout_sec_{60.0};    // Max time to drive backward (s)
  double pattern_log_buffer_period_sec_{60.0};  // In-memory log window before flush
  std::string pattern_log_file_path_{"/tmp/cat_patrol_pattern.log"};  // Pattern log file
  std::string completion_sound_path_;                  // Optional .wav played when patrol cycle completes (empty = disabled)
  std::string completion_sound_player_{"paplay"};     // Player when BT ready (paplay routes to default sink)
  bool bt_connect_on_start_{true};                     // Pair/connect BT speaker at node startup
  std::string bt_speaker_mac_{"0C:8E:29:3B:F2:5F"};    // RX-V485 3BF25C
  std::string bt_speaker_name_{"RX-V485"};             // Fallback name filter during scan
  int bt_scan_seconds_{40};                              // How long to scan before giving up
  std::string bt_connect_script_;                      // Optional override path to connect_bt_speaker.sh
  std::atomic_bool bt_audio_ready_{false};             // True after BT sink is default
  double linear_speed_{0.2};               // Forward/backward speed (m/s)
  double angular_speed_{0.6};              // Turning speed (rad/s)
  double patrol_drive_sec_{10.0};          // Duration per leg (classic pattern)
  int capture_frame_count_{12};            // Photos in 360° sweep
  double capture_turn_speed_{3.0};         // Max rotation speed during capture
  double capture_rotate_sec_{3.0};         // (Legacy) timed rotation per step
  double capture_kp_{2.0};                 // Proportional gain for capture yaw P-controller
  double capture_min_angular_speed_{0.15}; // Motor stall floor during capture rotation
  double capture_yaw_tolerance_{0.10};     // Yaw "close enough" threshold for capture (rad)
  double capture_completion_extra_deg_{0.0}; // Extra final rotation before completion beep
  double return_timeout_sec_{120.0};       // Max time to navigate home (s)
  double pos_tol_{0.12};                   // Position "close enough" (m)
  double yaw_tol_{0.2};                    // Heading "close enough" (rad ≈ 11°)
  double patrol_period_sec_{1800.0};       // Idle time between patrol cycles (s)
  bool start_on_boot_{true};               // Start patrolling at launch?
  bool loop_patrol_{true};                 // Auto-repeat after idle period?
  bool cat_detection_enabled_{false};      // React to cat sightings?
  // --- Cat-recognized snapshot email (fires only during an active patrol) ---
  bool cat_recognized_mail_enabled_{true}; // Email a snapshot on white/brown recognition?
  double cat_recognized_min_conf_{0.6};    // Identity confidence bar before emailing
  double cat_recognized_cooldown_sec_{30.0}; // Min seconds between emails for the SAME cat
  // --- Loop-boundary snapshot emails (start/end of each patrol cycle) ---
  bool loop_boundary_mail_enabled_{true};  // Email one snapshot at loop start and loop end?
  double approach_linear_{0.1};            // Cat approach speed (m/s)
  double approach_max_sec_{15.0};          // Max cat approach time (s)
  bool exit_after_one_cycle_{false};       // Shut down after one patrol?
  bool use_joy_override_{false};           // Respect joystick override?

  // =========================================================================
  // Home position (recorded at patrol start for return-home navigation)
  // =========================================================================
  double home_x_{0}, home_y_{0}, home_yaw_{0};

  // =========================================================================
  // Capture state (used by the node's own capture_tick, classic pattern path)
  // =========================================================================
  std::vector<std::string> saved_paths_;   // JPEG file paths for email
  int frames_saved_{0};                    // Photo counter

  // C++ CONCEPT: SMART POINTERS (SharedPtr)
  //   sensor_msgs::msg::Image::SharedPtr is a std::shared_ptr<Image>.
  //   A shared_ptr automatically frees memory when the LAST copy goes away.
  //   Multiple places can hold a SharedPtr to the same message; when all
  //   copies are destroyed, the message is freed.  No manual delete needed.
  //
  //   ROS 2 uses SharedPtr for all subscription messages.  The image_cb
  //   callback stores the latest frame here; save_current_image reads it.
  sensor_msgs::msg::Image::SharedPtr last_image_;

  // Timing for the capture sub-state machine
  rclcpp::Time capture_phase_start_;   // When capture began (overall timeout)
  rclcpp::Time capture_rotate_end_;    // When current rotation step ends
  rclcpp::Time capture_settle_end_;    // When settle wait ends
  enum class CapturePhase { Snap, Rotate, Settle };
  CapturePhase capture_phase_{CapturePhase::Snap};

  // =========================================================================
  // Thread-safe sensor state
  // =========================================================================
  // C++ CONCEPT: std::atomic
  //   Sensor callbacks (image_cb, scan_cb, depth_image_cb) run in the
  //   executor's callback thread.  The main patrol_timer_cb also runs
  //   there, but we use atomic for safety in case the node is ever run
  //   with a multi-threaded executor.
  //
  //   std::atomic<T> guarantees that reads and writes are INDIVISIBLE —
  //   you never see a half-written value.  No mutex needed for simple
  //   load/store operations.
  //
  //   Usage:
  //     min_scan_range_.store(value);  // Write  (or just: min_scan_range_ = value)
  //     float r = min_scan_range_.load();  // Read
  //
  //   The initial values are "safe defaults": large range = no obstacle,
  //   false = no joystick, no cat.
  std::atomic<float> min_scan_range_{1000.0f};   // Closest LiDAR reading (m)
  std::atomic<float> min_depth_range_{1000.0f};  // Closest depth reading (m)
  std::atomic_bool joy_active_{false};           // Is joystick controlling?
  std::atomic_bool cat_detected_{false};         // Is a cat visible?

  // Depth-camera freshness watchdog: if we don't get a depth frame within
  // depth_max_age_sec_, depth_obstacle_ahead() returns TRUE (fail-safe stop)
  // instead of FALSE (drive blindly forward).
  std::atomic<double> last_depth_msg_secs_{0.0};   // ROS time of last depth msg
  double depth_max_age_sec_{1.5};                  // Older than this => unsafe

  // Battery voltage monitor — patrol behavior is sensitive to voltage
  // (motors deliver less torque as battery drains, causing wheel slip and
  // smaller physical rotations than odometry reports).
  std::atomic<float> last_voltage_v_{0.0f};        // Latest /voltage reading
  std::string voltage_topic_;                      // Topic to subscribe
  double low_voltage_warn_v_{11.0};                // Warn below this (3S Li-ion: 11.1V nominal cell-low)

  // Cat approach phase timing
  rclcpp::Time approach_start_;

  // Per-identity cooldown for the cat-recognized email: label ("white"/"brown")
  // → time of the last snapshot emailed for that cat. Keeps one lingering
  // sighting from emailing every inference frame.
  std::unordered_map<std::string, rclcpp::Time> last_cat_mail_;

  // =========================================================================
  // FSM state and timers
  // =========================================================================
  PatrolState state_{PatrolState::Idle};    // Current top-level state
  rclcpp::Time state_enter_time_;          // When we entered current state

  // ROS 2 CONCEPT: TIMERS
  //   rclcpp::TimerBase::SharedPtr is a smart pointer to a ROS timer.
  //   Timers fire a callback at a fixed interval (wall-clock time).
  //
  //   SharedPtr (std::shared_ptr) ensures the timer stays alive as long
  //   as at least one SharedPtr points to it.  When the node is destroyed,
  //   these SharedPtrs are destroyed too, stopping the timers.
  rclcpp::TimerBase::SharedPtr patrol_timer_;       // 20 Hz main loop
  rclcpp::TimerBase::SharedPtr idle_cycle_timer_;   // Periodic patrol restart
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;    // 60s alive beep
  rclcpp::TimerBase::SharedPtr heartbeat_off_timer_; // Turn off heartbeat beep

  // =========================================================================
  // Publishers and Subscribers
  // =========================================================================
  // ROS 2 CONCEPT: PUBLISHERS AND SUBSCRIBERS
  //   Publisher<T>:    sends messages of type T to a topic.
  //     cmd_pub_->publish(twist_msg);
  //
  //   Subscription<T>: receives messages of type T from a topic.
  //     The constructor registers a callback; ROS calls it automatically.
  //
  //   Both are stored as SharedPtrs.  The template parameter <T> is the
  //   message type (Twist, Image, Bool, etc.).
  //
  //   "10" in create_publisher/subscription is the QUEUE DEPTH:
  //   how many unsent/unprocessed messages to buffer before dropping.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;    // → /cmd_vel
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr buzzer_pub_;       // → /Buzzer
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mail_pub_;       // → /cat_patrol/mail_request
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;       // ← color camera
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_; // ← depth camera
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;    // ← LiDAR
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr joy_sub_;             // ← joystick
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cat_sub_;             // ← cat detector (Bool)
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_; // ← cat_detector_cpp
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_sub_;      // ← battery

  // =========================================================================
  // TF (Transform Framework) objects
  // =========================================================================
  // C++ CONCEPT: unique_ptr vs shared_ptr
  //   unique_ptr<T>:  SOLE OWNER of the object.  Only one unique_ptr can
  //     point to a given object.  Cannot be copied, only moved.  Slightly
  //     more efficient than shared_ptr (no reference counting overhead).
  //     Use when ownership is clear: "this class and ONLY this class owns it."
  //
  //   shared_ptr<T>:  SHARED OWNERSHIP.  Multiple shared_ptrs can point to
  //     the same object.  A reference count tracks how many exist; the
  //     object is freed when the count reaches zero.  Use when ownership
  //     is shared or when the API requires it.
  //
  //   tf2_ros::TransformListener needs a shared_ptr because it may be
  //   used internally by ROS executor callbacks.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;        // Stores recent transforms
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  // Auto-subscribes to /tf

  // =========================================================================
  // Active patrol pattern (Strategy pattern — see patrol_pattern.hpp)
  // =========================================================================
  // C++ CONCEPT: POLYMORPHISM VIA unique_ptr<BaseClass>
  //   active_pattern_ is a unique_ptr to the BASE class (PatrolPattern).
  //   At runtime, it points to a DERIVED class (ClassicPattern or
  //   TillObstacleBackPattern).  When we call active_pattern_->tick(),
  //   C++ automatically calls the CORRECT derived class's tick() method.
  //   This is "runtime polymorphism" — the key mechanism behind the
  //   Strategy design pattern.
  std::unique_ptr<PatrolPattern> active_pattern_;
};

}  // namespace cat_patrol_robot

#endif  // CAT_PATROL_ROBOT__PATROL_NODE_HPP_
