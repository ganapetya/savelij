// =============================================================================
// odom_drift_checker_node.cpp
// =============================================================================
//
// Phase 0 deliverable for the cat-patrol robot project.
//
// What it does
// ------------
//   - Subscribes to /odom (the EKF-fused odometry from robot_localization).
//   - Captures the very first pose as "home".
//   - Integrates total path length between consecutive odom samples.
//   - Each time the robot leaves home (> away_threshold_m_) and then comes
//     back (< return_threshold_m_), prints one summary line:
//       path traveled vs straight-line drift vs yaw error.
//   - Every status_period_sec_, prints a short heartbeat so you can tell
//     the node is alive even before you've moved.
//   - Cross-checks /odom (topic) against the odom -> base_footprint TF
//     (independent path) once per heartbeat. If the two disagree by more
//     than ~1 cm something is wrong (e.g. EKF crashed but topic still latched).
//
// Why this exercise exists
// ------------------------
// SLAM / AMCL / Nav2 all assume odom is "good enough". On a small home
// robot with rubber wheels on carpet, /odom can drift surprisingly fast,
// and you want to know that *before* you spend a week debugging Nav2.
//
// C++ / ROS 2 idioms exercised here
// ---------------------------------
//   rclcpp::Node subclass         - constructor injection, member SharedPtr<>s
//   create_subscription           - QoS, std::bind callback
//   create_wall_timer             - lambda callback (the alternative style)
//   tf2_ros::Buffer + Listener    - lookupTransform at TimePointZero
//   std::optional<>               - "home pose not yet captured" without sentinels
//   declare_parameter<>           - parameter declarations with type-safe defaults
//   RCLCPP_INFO / WARN            - structured-ish logging
//
// =============================================================================

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/utils.h>                                  // tf2::getYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>      // converts geometry_msgs <-> tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <optional>
#include <string>


class OdomDriftChecker : public rclcpp::Node
{
public:
  OdomDriftChecker()
  : Node("odom_drift_checker"),
    // tf_buffer_ uses this node's clock so it ages out old transforms
    // consistently with ROS time. Required if we ever set use_sim_time:=true.
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- Parameters ----
    // declare_parameter<T>() returns the resolved default *if* the parameter
    // wasn't already overridden via the launch file or CLI. The <T> form is
    // type-safe: a string passed where a double is expected becomes a clean
    // launch-time error, not a silent runtime surprise.
    return_threshold_m_  = this->declare_parameter<double>("return_threshold_m",  0.30);
    away_threshold_m_    = this->declare_parameter<double>("away_threshold_m",    0.60);
    status_period_sec_   = this->declare_parameter<double>("status_period_sec",   2.0);
    odom_topic_          = this->declare_parameter<std::string>("odom_topic",     "/odom");
    odom_parent_frame_   = this->declare_parameter<std::string>("odom_parent_frame",   "odom");
    odom_child_frame_    = this->declare_parameter<std::string>("odom_child_frame",    "base_footprint");

    if (away_threshold_m_ <= return_threshold_m_) {
      RCLCPP_WARN(this->get_logger(),
        "away_threshold_m (%.3f) should be > return_threshold_m (%.3f); "
        "loop-completion detection will be flaky.",
        away_threshold_m_, return_threshold_m_);
    }

    // ---- Subscription ----
    // QoS(10) = depth-10 keep-last, the conventional default for odometry.
    // std::bind to a member function is the "obvious" callback style; lambdas
    // (see the timer below) are the other equally idiomatic option.
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&OdomDriftChecker::onOdom, this, std::placeholders::_1));

    // ---- Timer ----
    // create_wall_timer takes std::chrono::duration. Lambda callback because
    // the body is small and capturing `this` is the obvious thing to do here.
    const auto period = std::chrono::duration<double>(status_period_sec_);
    status_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { this->logStatus(); });

    RCLCPP_INFO(this->get_logger(),
      "odom_drift_checker started.\n"
      "  listening on        : %s\n"
      "  TF chain monitored  : %s -> %s\n"
      "  return threshold    : %.2f m\n"
      "  away   threshold    : %.2f m\n"
      "  status period       : %.1f s\n"
      "Drive a closed loop; I'll print one summary each time you return near home.",
      odom_topic_.c_str(),
      odom_parent_frame_.c_str(), odom_child_frame_.c_str(),
      return_threshold_m_, away_threshold_m_, status_period_sec_);
  }

private:
  // Plain 2D pose. We don't need 3D for a flat-floor robot, and rolling our
  // own struct keeps the math obvious. Using a struct (not a tuple or four
  // doubles) is the small-but-real C++ "make illegal states unrepresentable"
  // discipline: you can't accidentally swap x and y at the call site.
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Pose2D current{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      tf2::getYaw(msg->pose.pose.orientation),
    };

    // First message: capture home. std::optional means we don't need a
    // sentinel value or a "have_home" bool flag — `home_pose_.has_value()`
    // is the boolean and `*home_pose_` is the value, in one type.
    if (!home_pose_.has_value()) {
      home_pose_ = current;
      last_pose_ = current;
      RCLCPP_INFO(this->get_logger(),
        "Captured home pose: x=%.3f y=%.3f yaw=%.1f deg",
        current.x, current.y, current.yaw * 180.0 / M_PI);
      return;
    }

    // Integrate path length: sum of Euclidean distances between consecutive
    // odom samples. This is "absolute distance traveled" per the plan.
    // It is NOT the straight-line distance from home (that's d_home below).
    cumulative_path_m_ += std::hypot(
      current.x - last_pose_.x,
      current.y - last_pose_.y);
    last_pose_ = current;

    // Distance from home in the odom frame.
    const double d_home = std::hypot(
      current.x - home_pose_->x,
      current.y - home_pose_->y);

    // Edge-triggered "we just came back home" detection.
    // Hysteresis (away_threshold > return_threshold) prevents the message
    // from spamming when you're parked exactly at the boundary.
    if (was_away_ && d_home < return_threshold_m_) {
      const double yaw_err_deg =
        normalizeAngle(current.yaw - home_pose_->yaw) * 180.0 / M_PI;

      RCLCPP_INFO(this->get_logger(),
        "RETURN HOME  | x=%+.3f y=%+.3f yaw=%+.1f deg | "
        "path=%.2f m | drift=%.3f m | yaw_err=%+.1f deg",
        current.x, current.y, current.yaw * 180.0 / M_PI,
        cumulative_path_m_, d_home, yaw_err_deg);
      was_away_ = false;
    } else if (d_home > away_threshold_m_) {
      was_away_ = true;
    }
  }

  void logStatus()
  {
    if (!home_pose_.has_value()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for first /odom message...");
      return;
    }

    const double d_home = std::hypot(
      last_pose_.x - home_pose_->x,
      last_pose_.y - home_pose_->y);

    // ---- Independent TF cross-check ----
    // We already know the pose from the /odom topic. Looking it up via TF is
    // a *separate* pipeline (the EKF publishes both — topic via DDS, TF via
    // /tf), so if the two disagree something is wrong upstream.
    //
    // tf2::TimePointZero means "give me the latest available transform".
    // The educational alternative is to look up at a *specific* timestamp:
    //   tf2_ros::fromMsg(msg->header.stamp)  // requires <tf2_ros/time.h>
    // which would let you align the lookup with the moment the odom message
    // was generated. We use TimePointZero here for the heartbeat because
    // there's no triggering message — we just want "now".
    try {
      const auto tf =
        tf_buffer_.lookupTransform(odom_parent_frame_, odom_child_frame_,
                                   tf2::TimePointZero);

      const double tf_x = tf.transform.translation.x;
      const double tf_y = tf.transform.translation.y;
      const double tf_topic_disagreement = std::hypot(
        tf_x - last_pose_.x, tf_y - last_pose_.y);

      RCLCPP_INFO(this->get_logger(),
        "STATUS | x=%+.3f y=%+.3f yaw=%+.1f deg | "
        "path=%.2f m | from_home=%.3f m | tf_vs_topic=%.4f m",
        last_pose_.x, last_pose_.y, last_pose_.yaw * 180.0 / M_PI,
        cumulative_path_m_, d_home, tf_topic_disagreement);

      // 1 cm is well above EKF / serialization noise but well below any
      // real "they disagree" event. Tune up if it spams on this robot.
      if (tf_topic_disagreement > 0.01) {
        RCLCPP_WARN(this->get_logger(),
          "TF vs /odom disagreement %.3f m > 1 cm — EKF or TF chain wedged?",
          tf_topic_disagreement);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(),
        "TF lookup %s -> %s failed: %s",
        odom_parent_frame_.c_str(), odom_child_frame_.c_str(), ex.what());
    }
  }

  static double normalizeAngle(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // ---- Subscriptions / timers ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr                              status_timer_;

  // ---- TF infrastructure ----
  // Order matters: tf_buffer_ must be constructed before tf_listener_, and
  // the listener holds a reference to the buffer. Declaration order here is
  // the same order they appear in the constructor's initializer list. C++
  // initializes members in declaration order regardless of how they're
  // listed in the initializer — so getting both right keeps it sane.
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ---- Runtime state ----
  std::optional<Pose2D> home_pose_;
  Pose2D                last_pose_;
  double                cumulative_path_m_{0.0};
  bool                  was_away_{false};

  // ---- Parameters (resolved in ctor) ----
  double      return_threshold_m_{0.30};
  double      away_threshold_m_{0.60};
  double      status_period_sec_{2.0};
  std::string odom_topic_{"/odom"};
  std::string odom_parent_frame_{"odom"};
  std::string odom_child_frame_{"base_footprint"};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomDriftChecker>());
  rclcpp::shutdown();
  return 0;
}
