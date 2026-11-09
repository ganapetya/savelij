// =============================================================================
// classic_pattern.cpp — Forward/backward patrol with LiDAR obstacle safety
// =============================================================================
//
// CONTROL STYLE: OPEN-LOOP, TIME-BASED
//   This pattern uses "open-loop" timing: drive for N seconds, then switch
//   direction.  It does NOT measure how far the robot actually traveled —
//   it just trusts the clock.  This is simple but imprecise: if wheels
//   slip on a smooth floor, the robot might travel less than expected.
//
//   Compare with TillObstacleBackPattern which uses closed-loop odometry
//   for both driving (measure distance to home) and capture rotation
//   (measure yaw angle).
//
// PHASE CALCULATION:
//   With 4 legs and patrol_drive_sec = 2.0:
//     Time 0.0 → 2.0s:  phase 0 (even → forward)
//     Time 2.0 → 4.0s:  phase 1 (odd  → backward)
//     Time 4.0 → 6.0s:  phase 2 (even → forward)
//     Time 6.0 → 8.0s:  phase 3 (odd  → backward)
//     Time ≥ 8.0s:       done → DoneNextCapture
//
//   static_cast<int>(elapsed / leg) gives which leg we're on (integer
//   division rounds down).  Even phases go forward, odd phases go backward.
// =============================================================================

#include "cat_patrol_robot/patterns/classic_pattern.hpp"

namespace cat_patrol_robot
{

// ===========================================================================
// on_start — record the starting time (everything is relative to this)
// ===========================================================================
void ClassicPattern::on_start(PatrolContext & ctx)
{
  start_time_ = ctx.now();
  RCLCPP_INFO(ctx.get_logger(),
    "[classic] Driving forward/back for %.1f s per leg (4 legs)",
    ctx.patrol_drive_sec);
}

// ===========================================================================
// tick — called every 50ms; decide: forward, backward, or done?
// ===========================================================================
PatrolSignal ClassicPattern::tick(PatrolContext & ctx)
{
  // Safety: if LiDAR detects an obstacle too close, STOP.
  // The clock keeps running, so the patrol gets shorter (we don't extend
  // the time to compensate for obstacle pauses).
  if (ctx.lidar_obstacle_ahead()) {
    ctx.stop_robot();
    return PatrolSignal::Continue;  // Stay in patrol, try again next tick
  }

  // How many seconds since patrol started
  const double elapsed = (ctx.now() - start_time_).seconds();
  const double leg = ctx.patrol_drive_sec;  // Duration of one forward or back leg
  const double total = leg * 4.0;           // Total patrol time (4 legs)

  // All 4 legs done?
  if (elapsed >= total) {
    ctx.stop_robot();
    RCLCPP_INFO(ctx.get_logger(), "[classic] Patrol drive done");
    // Tell the node: "I'm done — please start taking photos now"
    return PatrolSignal::DoneNextCapture;
  }

  // Which leg are we on?
  //
  // C++ CONCEPT: static_cast<int>(...)
  //   Explicitly converts a double to an int (truncates toward zero).
  //   C++ has several cast types:
  //     static_cast    — compile-time checked, normal conversions
  //     dynamic_cast   — for polymorphic classes (checks at runtime)
  //     reinterpret_cast — dangerous, raw pointer reinterpretation
  //     const_cast     — adds/removes "const"
  //   Always prefer static_cast for simple type conversions.
  int phase = static_cast<int>(elapsed / leg);

  // C++ CONCEPT: TERNARY OPERATOR (condition ? true_value : false_value)
  //   A compact if/else that returns a value.  Reads as:
  //   "if phase is even, go forward; if odd, go backward."
  //
  // MATH: modulo 2 (% 2) gives 0 for even, 1 for odd numbers.
  double vx = (phase % 2 == 0) ? ctx.linear_speed : -ctx.linear_speed;

  RCLCPP_INFO_THROTTLE(ctx.get_logger(), *rclcpp::Clock::make_shared(), 2000,
    "[classic] %s: %.1f / %.1fs", (vx > 0 ? "FWD" : "BACK"), elapsed, total);

  // Publish velocity: forward/backward speed, no strafe, no turn
  ctx.publish_twist(vx, 0.0, 0.0);
  return PatrolSignal::Continue;
}

}  // namespace cat_patrol_robot
