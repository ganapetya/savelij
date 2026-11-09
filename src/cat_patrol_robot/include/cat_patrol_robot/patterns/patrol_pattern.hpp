// =============================================================================
// patrol_pattern.hpp — The "Strategy" design pattern for patrol behaviors
// =============================================================================
//
// C++ DESIGN PATTERN: STRATEGY
//   The Strategy pattern lets you swap algorithms at runtime.  Instead of
//   hard-coding one patrol behavior (forward/back) into the main node, we
//   define an INTERFACE (PatrolPattern) and let concrete STRATEGIES implement
//   it (ClassicPattern, TillObstacleBackPattern, etc.).
//
//   The node doesn't know or care WHICH pattern is running.  It just calls:
//     active_pattern_->tick(ctx);
//   And the current pattern decides what the robot should do.
//
//   Benefits:
//     - Add new patrol behaviors without touching the main node code.
//     - Switch patterns via a YAML parameter (no recompilation).
//     - Each pattern is a self-contained file — easy to test in isolation.
//
// C++ CONCEPT: HEADER GUARDS (#ifndef / #define / #endif)
//   These three lines prevent the compiler from including this file twice.
//   Without them, if two files both #include this header, you'd get
//   "redefinition" errors.  The guard name follows ROS 2 convention:
//     PACKAGE__PATH__FILENAME_HPP_
//
//   Modern alternative: "#pragma once" (does the same thing, shorter,
//   but not in the C++ standard — most compilers support it though).
// =============================================================================
#ifndef CAT_PATROL_ROBOT__PATTERNS__PATROL_PATTERN_HPP_
#define CAT_PATROL_ROBOT__PATTERNS__PATROL_PATTERN_HPP_

// ---------------------------------------------------------------------------
// Standard Library headers
// ---------------------------------------------------------------------------
#include <atomic>       // std::atomic — thread-safe variables (not used here,
                        // but available for patterns that need thread safety)
#include <cmath>        // M_PI, sin, cos, etc.
#include <functional>   // std::function — wraps any callable (function, lambda,
                        // method) into a uniform type.  See PatrolContext below.
#include <string>       // std::string — dynamic-length text
#include <vector>       // std::vector — dynamic array (growable list)

// ---------------------------------------------------------------------------
// ROS 2 headers
// ---------------------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"         // Core ROS 2 C++ library: Node, Time,
                                     // Logger, Duration, etc.
#include "sensor_msgs/msg/image.hpp" // sensor_msgs::msg::Image — camera frames

// ---------------------------------------------------------------------------
// Everything in this file lives inside the "cat_patrol_robot" namespace.
// A namespace prevents name collisions: if another package also defines
// "PatrolPattern", there's no conflict because ours is actually
// "cat_patrol_robot::PatrolPattern".
// ---------------------------------------------------------------------------
namespace cat_patrol_robot
{

// ===========================================================================
// PatrolSignal — what the pattern tells the node after each tick
// ===========================================================================
// C++ CONCEPT: SCOPED ENUM (enum class)
//   "enum class" is a TYPE-SAFE enumeration introduced in C++11.
//   Unlike old-style "enum", the values are SCOPED — you must write
//   PatrolSignal::Continue, not just Continue.  This prevents
//   accidental name collisions and implicit conversions to int.
//
//   Each value tells the node what to do AFTER this tick:
//     Continue         — keep calling tick() next cycle
//     DoneNextCapture  — pattern finished; node should start taking photos
//     DoneIdle         — pattern finished; node should go idle
// ===========================================================================
enum class PatrolSignal
{
  Continue,
  DoneNextCapture,
  DoneIdle,
};

// ===========================================================================
// PatrolContext — a bundle of capabilities passed INTO a pattern
// ===========================================================================
// C++ CONCEPT: DEPENDENCY INJECTION
//   Instead of giving patterns direct access to the node (tight coupling),
//   we pass a lightweight struct with ONLY what they need.  This is
//   "dependency injection" — the pattern RECEIVES its dependencies rather
//   than reaching out to find them.
//
// C++ CONCEPT: std::function<SIGNATURE>
//   std::function is a "type-erased callable wrapper."  That means it can
//   hold ANY callable with the matching signature:
//     - A plain function:        void my_stop() { ... }
//     - A lambda:                [this]() { stop_robot(); }
//     - A bound member function: std::bind(&Node::stop, this)
//
//   Signature format:  std::function< RETURN_TYPE (PARAM_TYPES...) >
//     std::function<void()>                — takes nothing, returns nothing
//     std::function<bool(double&, ...)>    — takes references, returns bool
//     std::function<void(double, double, double)> — takes 3 doubles
//
//   The node fills these in using lambdas (see build_patrol_context in
//   patrol_node.cpp).  The pattern calls them like regular functions:
//     ctx.stop_robot();           // calls the node's stop_robot()
//     ctx.publish_twist(0.2, 0, 0);  // calls the node's publish_twist()
//
// WHY NOT JUST PASS A NODE POINTER?
//   Passing the whole node would expose hundreds of methods and variables.
//   The pattern could accidentally call transition_to(), modify state,
//   access timers, etc.  PatrolContext exposes a MINIMAL, SAFE interface.
//   This is the "Interface Segregation Principle" (the I in SOLID).
// ===========================================================================
struct PatrolContext
{
  // --- Robot actions (the pattern calls these to make things happen) ---

  // Move the robot: linear.x (fwd/back), linear.y (strafe), angular.z (turn)
  std::function<void(double, double, double)> publish_twist;

  // Stop all wheel motion immediately (publishes zero velocity)
  std::function<void()> stop_robot;

  // Ask TF: "where is the robot?"  Returns (x, y, yaw) via references.
  // Returns true if pose is available, false if TF not ready.
  //
  // C++ CONCEPT: PASSING BY REFERENCE (double &)
  //   The ampersand (&) means "pass by reference" — the function WRITES
  //   directly into the caller's variables.  Without &, it would only
  //   modify local copies (pass by value).
  //   Example:
  //     double x, y, yaw;
  //     if (ctx.get_odom_pose(x, y, yaw)) {
  //       // x, y, yaw are now filled in by the function
  //     }
  std::function<bool(double &, double &, double &)> get_odom_pose;

  // Obstacle detection — returns true if something is dangerously close
  std::function<bool()> depth_obstacle_ahead;  // Uses depth camera
  std::function<bool()> lidar_obstacle_ahead;  // Uses LiDAR (if enabled)

  // Save the current color camera frame to a JPEG on disk.
  // Returns true if saved successfully, false if no image available.
  std::function<bool()> save_current_image;

  // Publish a JSON message asking the mail_node to email saved photos
  std::function<void()> send_mail_request;

  // Turn the onboard buzzer on (true) or off (false)
  std::function<void(bool)> set_buzzer;

  // Play the "patrol completed" sound (e.g. bark.wav).  Non-blocking:
  // the node spawns the player in a detached thread, so the pattern's
  // tick loop is not stalled.  If no sound is configured, this is a no-op.
  std::function<void()> play_completion_sound;

  // Get the current ROS time.  rclcpp::Time is a high-resolution timestamp
  // (nanosecond precision).  Subtraction gives rclcpp::Duration.
  //   auto dt = ctx.now() - some_earlier_time;
  //   double seconds = dt.seconds();  // convert Duration to double
  std::function<rclcpp::Time()> now;

  // Get the ROS logger for this node (used with RCLCPP_INFO, RCLCPP_WARN, etc.)
  std::function<rclcpp::Logger()> get_logger;

  // Shared clock — use for RCLCPP_*_THROTTLE macros instead of
  // rclcpp::Clock::make_shared() which heap-allocates every call.
  rclcpp::Clock::SharedPtr clock;

  // --- Data from the node (read-only values set before each tick) ---

  // Home position recorded at patrol start (odometry frame).
  // The robot navigates back here after patrolling.
  double home_x{0};
  double home_y{0};
  double home_yaw{0};    // Heading in radians (0 = original direction)

  // Motion parameters (loaded from YAML config, see cat_patrol_params.yaml)
  double linear_speed{0.2};     // Forward/backward speed (m/s)
  double angular_speed{0.6};    // Turning speed (rad/s)
  double pos_tol{0.15};         // "Close enough" distance to home (meters)
  double yaw_tol{0.2};          // "Close enough" heading error (radians, ~11°)
  double patrol_drive_sec{10.0}; // Duration of each forward/backward leg

  // Panoramic capture parameters
  int capture_frame_count{12};    // How many photos in the 360° sweep
  double capture_turn_speed{3.0}; // Max rotation speed during capture (rad/s)
  double capture_rotate_sec{3.0}; // (Legacy) timed rotation duration per step

  // Closed-loop rotation tuning (capture P-controller)
  double capture_kp{2.0};                 // Proportional gain for yaw error
  double capture_min_angular_speed{0.15}; // Floor speed to prevent motor stall (rad/s)
  double capture_yaw_tolerance{0.10};     // "Close enough" yaw threshold (rad)
};

// ===========================================================================
// PatrolPattern — abstract base class (the Strategy interface)
// ===========================================================================
// C++ CONCEPT: ABSTRACT CLASS / PURE VIRTUAL FUNCTIONS
//   A class with at least one "= 0" (pure virtual) method is ABSTRACT —
//   you CANNOT create an instance of it directly.  You MUST create a
//   derived class (like ClassicPattern) that OVERRIDES all pure virtuals.
//
//   Why?  The base class defines WHAT must happen (interface), not HOW
//   (implementation).  Different derived classes provide different HOWs.
//
// C++ CONCEPT: VIRTUAL DESTRUCTOR
//   "virtual ~PatrolPattern() = default;" ensures that when you delete
//   a pattern through a base pointer (e.g., unique_ptr<PatrolPattern>),
//   the DERIVED class destructor runs too.  Without "virtual", only
//   the base destructor runs, potentially leaking resources.
//   Rule of thumb: if a class has virtual methods, give it a virtual destructor.
//
// C++ CONCEPT: "= default"
//   Tells the compiler: "generate the default implementation for me."
//   For a destructor, that means "do the normal cleanup" (free members, etc.).
//   It's a way to be explicit: "yes, I thought about this, and the default is fine."
//
// C++ CONCEPT: "= 0" (PURE VIRTUAL)
//   "virtual void on_start(...) = 0;" means: "this function HAS NO body here.
//   Every derived class MUST provide its own implementation."
//   If a derived class forgets to override a pure virtual, the compiler
//   will refuse to compile it ("cannot instantiate abstract class").
//
// C++ CONCEPT: "override"
//   In derived classes, "override" tells the compiler: "I intend to override
//   a virtual from the base class."  If you misspell the function name or
//   get the signature wrong, the compiler catches it immediately.  Always
//   use "override" — it prevents subtle bugs where you accidentally create
//   a NEW function instead of overriding the base one.
// ===========================================================================
class PatrolPattern
{
public:
  virtual ~PatrolPattern() = default;

  // Called ONCE when the patrol starts.  Initialize your state here.
  virtual void on_start(PatrolContext & ctx) = 0;

  // Called every 50 ms (20 Hz).  Return a signal telling the node what to do.
  // This is where the pattern's main logic lives.
  virtual PatrolSignal tick(PatrolContext & ctx) = 0;

  // Called when the patrol is interrupted or stopped.  Clean up here.
  // Default implementation does nothing — override if you need cleanup.
  //
  // C++ CONCEPT: "/*ctx*/"
  //   Commenting out a parameter name ("PatrolContext & /*ctx*/") tells the
  //   compiler: "I know this parameter exists, but I don't use it."
  //   This avoids "unused parameter" warnings without removing the parameter.
  virtual void on_stop(PatrolContext & /*ctx*/) {}

  // Return a human-readable name for this pattern (used in log messages)
  virtual std::string name() const = 0;
};

}  // namespace cat_patrol_robot

#endif  // CAT_PATROL_ROBOT__PATTERNS__PATROL_PATTERN_HPP_
