// =============================================================================
// till_obstacle_back_pattern.cpp — Implementation of the 4-phase patrol
// =============================================================================
// This file contains the METHOD BODIES for TillObstacleBackPattern.
// The class is DECLARED in the matching .hpp header file; the compiler
// needs both to build a complete program.
//
// THE FULL SEQUENCE:
//   1. DriveForward   — linear.x = speed → roll straight until depth camera
//                       reports an obstacle within depth_obstacle_min_m.
//   2. DriveBackHome  — linear.x = -speed → reverse until odometry says
//                       we're within pos_tol of the recorded home position.
//   3. CaptureImages  — spin in place taking N photos at evenly-spaced yaw
//                       targets (closed-loop, NOT timed).  Beep on each snap.
//                       After last photo, rotate back to start yaw (full 360°).
//                       Then email the photos via send_mail_request().
//   4. TurnAround     — rotate 180° (π radians) so the robot faces the
//                       opposite direction from where it started, then stop.
// =============================================================================

#include "cat_patrol_robot/patterns/till_obstacle_back_pattern.hpp"

#include <cmath>  // M_PI, std::abs, std::copysign, std::hypot, std::clamp
#include <fstream>

namespace cat_patrol_robot
{

// ===========================================================================
// normalize_angle — wrap any angle to the range (-π, +π]
// ===========================================================================
// MATH CONCEPT:
//   Angles in radians can exceed ±π.  For example, if you rotate 350°
//   counter-clockwise, the raw angle is ~6.1 radians.  But that's equivalent
//   to -10° (i.e., -0.17 rad).  Navigation code needs the SHORTEST turn
//   direction, so we normalize first.
//
//   This function keeps subtracting or adding 2π (one full turn) until
//   the angle falls within (-π, +π].
//
// C++ CONCEPT: "static" FREE FUNCTION
//   The "static" keyword here means "internal linkage" — this function is
//   only visible within this .cpp file.  Other .cpp files can have their
//   own static normalize_angle() without conflict.  This is the C++ way
//   of saying "this is a private helper, not part of the public API."
// ===========================================================================
static double normalize_angle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

// ===========================================================================
// on_start — called once when the patrol begins
// ===========================================================================
void TillObstacleBackPattern::on_start(PatrolContext & ctx)
{
  frames_saved_ = 0;
  saved_paths_.clear();
  turn_target_set_ = false;
  buffered_logs_.clear();
  log_window_start_ = ctx.now();
  log_flush_pending_ = false;
  enter_phase(Phase::DriveForward, ctx);
  RCLCPP_INFO(ctx.get_logger(),
    "[till_obstacle_back] Pattern started — driving forward until obstacle");
  append_buffered_log(ctx, "Pattern started");
}

// ===========================================================================
// on_stop — called when the patrol is interrupted (e.g., joystick override)
// ===========================================================================
void TillObstacleBackPattern::on_stop(PatrolContext & ctx)
{
  ctx.stop_robot();  // Safety: always zero velocity when stopping
  append_buffered_log(ctx, "Pattern stopped");
  maybe_flush_buffered_logs(ctx, true);
}

// ===========================================================================
// enter_phase — transition to a new phase
// ===========================================================================
// DESIGN PATTERN: SINGLE TRANSITION POINT
//   ALL phase changes go through this one function.  This guarantees:
//   - The robot always stops between phases (safety)
//   - The phase timer always resets
//   - Phase-specific initialization always runs
//   Having a single transition point makes bugs much easier to find.
// ===========================================================================
void TillObstacleBackPattern::enter_phase(Phase p, PatrolContext & ctx)
{
  const Phase prev_phase = phase_;
  ctx.stop_robot();              // Safety: stop wheels before switching
  phase_ = p;                    // Update current phase
  phase_start_ = ctx.now();      // Record when this phase started

  if (prev_phase == Phase::CaptureImages && p != Phase::CaptureImages) {
    // Requirement: never write logs to file during photo circle.
    // As soon as capture finishes, flush deferred logs.
    maybe_flush_buffered_logs(ctx, true);
  }

  // Phase-specific setup
  if (p == Phase::CaptureImages) {
    capture_step_ = CaptureStep::Snap;
    last_good_yaw_valid_ = false;
    stuck_tick_count_ = 0;
    beep_remaining_ = 0;
    beep_is_on_ = false;
    completion_beep_pending_ = false;

    double x, y, yaw;
    if (ctx.get_odom_pose(x, y, yaw)) {
      capture_start_yaw_ = yaw;
      last_good_yaw_ = yaw;
      last_good_yaw_valid_ = true;
      RCLCPP_INFO(ctx.get_logger(),
        "[till_obstacle_back] Capture start — recording yaw=%.3f rad", yaw);
      append_buffered_log(ctx, "Capture started");
    }
  }
}

void TillObstacleBackPattern::append_buffered_log(PatrolContext & ctx, const std::string & message)
{
  const auto t = ctx.now();
  if (log_window_start_.nanoseconds() == 0) {
    log_window_start_ = t;
  }

  buffered_logs_.push_back(std::to_string(t.nanoseconds()) + " " + message);

  if ((t - log_window_start_).seconds() >= log_buffer_period_sec) {
    log_flush_pending_ = true;
  }
}

void TillObstacleBackPattern::maybe_flush_buffered_logs(PatrolContext & ctx, bool force)
{
  if (buffered_logs_.empty()) {
    return;
  }

  if (!force) {
    if (!log_flush_pending_) {
      return;
    }
    if (phase_ == Phase::CaptureImages) {
      // Never flush to disk while photo circle is running.
      return;
    }
  }

  std::ofstream out(log_file_path, std::ios::app);
  if (!out.is_open()) {
    RCLCPP_WARN_THROTTLE(ctx.get_logger(), *ctx.clock, 10000,
      "[till_obstacle_back] Could not open log file: %s", log_file_path.c_str());
    return;
  }

  for (const auto & line : buffered_logs_) {
    out << line << '\n';
  }
  out.flush();
  buffered_logs_.clear();
  log_window_start_ = ctx.now();
  log_flush_pending_ = false;
}

// ===========================================================================
// tick — main dispatch (called 20 times per second by the node)
// ===========================================================================
// C++ CONCEPT: SWITCH STATEMENT
//   "switch (phase_)" is like a series of "if / else if" checks, but the
//   compiler can optimize it into a jump table — O(1) dispatch instead of
//   O(N) comparisons.  Each "case" label is a possible value.  The
//   "return" exits the switch and the function at the same time.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick(PatrolContext & ctx)
{
  maybe_flush_buffered_logs(ctx, false);
  switch (phase_) {
    case Phase::DriveForward:   return tick_drive_forward(ctx);
    case Phase::DriveBackHome:  return tick_drive_back_home(ctx);
    case Phase::CaptureImages:  return tick_capture(ctx);
    case Phase::TurnAround:     return tick_turn_around(ctx);
    case Phase::Done:           return PatrolSignal::DoneIdle;
  }
  return PatrolSignal::DoneIdle;  // Unreachable, but keeps the compiler happy
}

// ===========================================================================
// Phase 1: DriveForward — drive straight until obstacle detected
// ===========================================================================
// ROBOTICS CONCEPT: DEPTH-CAMERA OBSTACLE DETECTION
//   The depth camera (e.g., Orbbec Astra) outputs a 2D image where each
//   pixel's value is the DISTANCE to whatever that pixel sees.  The node's
//   depth_image_cb() finds the minimum distance in a central region of the
//   depth image.  If that minimum is below depth_obstacle_min_m (e.g., 0.5 m),
//   depth_obstacle_ahead() returns true.
//
//   This is simpler and cheaper than LiDAR, but only covers the camera's
//   field of view (typically 60-90° horizontal).  LiDAR covers 360° but
//   costs more and needs a separate sensor.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick_drive_forward(PatrolContext & ctx)
{
  // How long have we been in this phase?
  const double elapsed = (ctx.now() - phase_start_).seconds();

  // Safety timeout: if we've been driving for too long without hitting
  // anything, assume something is wrong and proceed to the next phase.
  if (elapsed > forward_timeout_sec) {
    RCLCPP_WARN(ctx.get_logger(),
      "[till_obstacle_back] Forward timeout (%.0fs) — switching to reverse", elapsed);
    enter_phase(Phase::DriveBackHome, ctx);
    append_buffered_log(ctx, "Forward timeout; switching to reverse");
    return PatrolSignal::Continue;
  }

  // Check if the depth camera sees something close ahead
  if (ctx.depth_obstacle_ahead()) {
    RCLCPP_INFO(ctx.get_logger(),
      "[till_obstacle_back] Obstacle detected after %.1fs — reversing to home", elapsed);
    enter_phase(Phase::DriveBackHome, ctx);
    append_buffered_log(ctx, "Obstacle seen; switching to reverse");
    return PatrolSignal::Continue;
  }

  // ROS 2 CONCEPT: RCLCPP_INFO_THROTTLE
  //   Like RCLCPP_INFO but only actually prints once per N milliseconds.
  //   Without throttling, logging at 20 Hz would flood the terminal with
  //   400 lines every 20 seconds — unreadable.  The 2000 means "print
  //   at most once every 2 seconds."
  RCLCPP_INFO_THROTTLE(ctx.get_logger(), *ctx.clock, 2000,
    "[till_obstacle_back] FWD %.1fs", elapsed);

  // Drive forward: linear.x = speed, no strafe (y=0), no turn (z=0)
  ctx.publish_twist(ctx.linear_speed, 0.0, 0.0);
  return PatrolSignal::Continue;
}

// ===========================================================================
// Phase 2: DriveBackHome — reverse to the starting position
// ===========================================================================
// ROBOTICS CONCEPT: ODOMETRY-BASED NAVIGATION
//   The robot knows its position via ODOMETRY: wheel encoders count how
//   many times each wheel has rotated, and from that, the robot calculates
//   how far it has traveled.  This position is published as a TF transform
//   from "odom" frame to "base_footprint" frame.
//
//   Odometry DRIFTS over time (wheel slip, uneven surfaces), so it's only
//   accurate over short distances.  For a patrol that drives a few meters
//   and comes back, it's good enough.  For building-scale navigation,
//   you'd need SLAM (Simultaneous Localization And Mapping) with a map.
//
// MATH CONCEPT: std::hypot(dx, dy)
//   Computes √(dx² + dy²) — the Euclidean (straight-line) distance between
//   two points.  It's more numerically stable than writing sqrt(dx*dx+dy*dy)
//   because it handles very large and very small values without overflow.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick_drive_back_home(PatrolContext & ctx)
{
  const double elapsed = (ctx.now() - phase_start_).seconds();

  if (elapsed > drive_back_timeout_sec) {
    RCLCPP_WARN(ctx.get_logger(),
      "[till_obstacle_back] Drive-back timeout (%.0fs)", elapsed);
    enter_phase(Phase::CaptureImages, ctx);
    return PatrolSignal::Continue;
  }

  // Get current position from odometry
  double x, y, yaw;
  if (!ctx.get_odom_pose(x, y, yaw)) {
    ctx.stop_robot();  // No pose available — stop and retry next tick
    return PatrolSignal::Continue;
  }

  const double dist = std::hypot(ctx.home_x - x, ctx.home_y - y);

  RCLCPP_INFO_THROTTLE(ctx.get_logger(), *ctx.clock, 3000,
    "[till_obstacle_back] BACK dist=%.3f pos=(%.2f,%.2f) yaw=%.2f",
    dist, x, y, yaw);

  if (dist > 50.0) {
    RCLCPP_WARN_THROTTLE(ctx.get_logger(), *ctx.clock, 5000,
      "[till_obstacle_back] Insane odom (dist=%.0f) — ignoring", dist);
    return PatrolSignal::Continue;
  }

  if (dist < ctx.pos_tol) {
    RCLCPP_INFO(ctx.get_logger(),
      "[till_obstacle_back] Reached home — starting capture");
    enter_phase(Phase::CaptureImages, ctx);
    return PatrolSignal::Continue;
  }

  // Reverse with heading correction.  This avoids "random" lateral strafing
  // while still steering back toward home.
  const double angle_to_home = std::atan2(ctx.home_y - y, ctx.home_x - x);
  const double desired_heading_for_reverse = normalize_angle(angle_to_home + M_PI);
  const double heading_err = normalize_angle(desired_heading_for_reverse - yaw);
  const double w = std::clamp(heading_err * 1.6, -ctx.angular_speed, ctx.angular_speed);

  if (std::abs(heading_err) > 0.40) {
    ctx.publish_twist(0.0, 0.0, w);
  } else {
    ctx.publish_twist(-ctx.linear_speed, 0.0, w);
  }
  return PatrolSignal::Continue;
}

// ===========================================================================
// Phase 3: CaptureImages — 360° panoramic photo sweep
// ===========================================================================
// ROBOTICS CONCEPT: CLOSED-LOOP YAW CONTROL
//   Instead of "spin for N seconds" (open-loop, unreliable), we use
//   the odometry heading (yaw) as FEEDBACK.  The algorithm:
//
//   1. Record start_yaw when entering this phase.
//   2. For photo i (0-indexed), compute absolute target:
//        target = normalize_angle(start_yaw + i * (2π / N))
//      With N=12, each step is 2π/12 = 30° = 0.524 radians.
//   3. Use a PROPORTIONAL CONTROLLER to rotate to the target:
//        error = normalize_angle(target - current_yaw)
//        speed = clamp(error * Kp, -max, +max)
//      This means: turn fast when far from target, slow when close.
//   4. After the LAST photo (i = N-1), rotate back to start_yaw to
//      complete a full 360°.
//
// CONTROL THEORY: PROPORTIONAL CONTROLLER (P-controller)
//   The simplest feedback controller: output = Kp * error.
//     - Kp (proportional gain) = 2.0 in our case
//     - error = target_yaw - current_yaw
//     - output = angular velocity (rad/s)
//
//   When error is large (far from target): output is large → fast rotation
//   When error is small (near target): output is small → slow, precise approach
//
//   We add two refinements:
//     - clamp(output, -max, +max): prevent commanding unreasonable speeds
//     - min speed floor (0.15 rad/s): below this, the motor might STALL
//       (not enough torque to overcome friction).  std::copysign preserves
//       the direction while enforcing the minimum magnitude.
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick_capture(PatrolContext & ctx)
{
  const auto tnow = ctx.now();
  const double total_elapsed = (tnow - phase_start_).seconds();
  const double settle_sec = 0.8;
  const double capture_yaw_tol = ctx.capture_yaw_tolerance;

  auto finish_capture = [&]() -> PatrolSignal {
      ctx.stop_robot();
      ctx.set_buzzer(false);
      RCLCPP_INFO(ctx.get_logger(),
        "[till_obstacle_back] Capture complete — %d images, full 360° in %.1fs",
        frames_saved_, total_elapsed);
      ctx.send_mail_request();
      enter_phase(Phase::TurnAround, ctx);
      append_buffered_log(ctx, "Capture complete");
      return PatrolSignal::Continue;
    };

  auto start_three_beeps = [&](bool for_completion) {
      // Drive buzzer from a dedicated sequence state.
      capture_buzzer_on_ = false;
      ctx.set_buzzer(false);
      beep_remaining_ = 3;
      beep_is_on_ = false;
      completion_beep_pending_ = for_completion;
      beep_toggle_deadline_ = tnow;
      capture_step_ = CaptureStep::BeepSequence;
    };

  // Overall safety timeout for the entire capture phase
  if (total_elapsed > 120.0) {
    RCLCPP_WARN(ctx.get_logger(),
      "[till_obstacle_back] Capture timeout after %.0fs", total_elapsed);
    ctx.stop_robot();
    ctx.set_buzzer(false);
    ctx.send_mail_request();    // Email whatever we captured
    enter_phase(Phase::TurnAround, ctx);
    append_buffered_log(ctx, "Capture timeout; continuing to turn phase");
    return PatrolSignal::Continue;
  }

  // Turn off the per-snap beep after 150 ms.
  // The beep was turned ON in the Snap case below; here we turn it OFF
  // after a short delay to produce an audible "blip."
  if (capture_buzzer_on_ && tnow >= buzzer_off_time_) {
    ctx.set_buzzer(false);
    capture_buzzer_on_ = false;
  }

  // MATH: compute the angular spacing between photos.
  // For 12 photos: step_angle = 2π/12 ≈ 0.524 rad ≈ 30°
  const double step_angle = 2.0 * M_PI / ctx.capture_frame_count;

  // -----------------------------------------------------------------------
  // Capture sub-state machine: Snap → Rotate → Settle → Snap → ...
  // -----------------------------------------------------------------------
  switch (capture_step_) {

    // =====================================================================
    // CaptureStep::Snap — take a photo
    // =====================================================================
    case CaptureStep::Snap: {
      // COMPLETION CHECK: all N photos taken AND the final rotation back
      // to start_yaw is done (we arrive back here after Rotate → Settle).
      if (frames_saved_ >= ctx.capture_frame_count) {
        // Requirement: play 3 beeps when full 360° is complete.
        start_three_beeps(true);
        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Full 360 done — playing 3 completion beeps");
        return PatrolSignal::Continue;
      }

      // Try to save the camera's latest color frame as a JPEG
      if (ctx.save_current_image()) {
        frames_saved_++;

        // Audible feedback: short beep so you know a photo was taken
        ctx.set_buzzer(true);
        capture_buzzer_on_ = true;
        buzzer_off_time_ = tnow + rclcpp::Duration::from_seconds(0.15);

        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Captured [%d/%d]", frames_saved_, ctx.capture_frame_count);
        append_buffered_log(ctx, "Captured frame " + std::to_string(frames_saved_));

        // Decide WHERE to rotate to next.
        // Use ABSOLUTE targets anchored to capture_start_yaw_ so each
        // photo slot stays fixed at k*(360/N). This prevents per-step
        // undershoot from accumulating across the full panorama.
        if (frames_saved_ < ctx.capture_frame_count) {
          capture_target_yaw_ = normalize_angle(
            capture_start_yaw_ + frames_saved_ * step_angle);
          if (frames_saved_ == 1) {
            // Requirement: play 3 beeps before rotation starts.
            start_three_beeps(false);
            RCLCPP_INFO(ctx.get_logger(),
              "[till_obstacle_back] Playing 3 beeps before first rotation");
            break;
          }
        } else {
          // Last photo taken — rotate back to starting yaw to complete
          // the full 360° (plus optional compensation). Uses absolute start_yaw because the odom
          // frame is self-consistent: start_yaw was recorded in the same
          // frame, so "return to start_yaw" means "face the same way."
          const double extra_rad = capture_completion_extra_deg * M_PI / 180.0;
          capture_target_yaw_ = normalize_angle(capture_start_yaw_ + extra_rad);
        }

        capture_step_ = CaptureStep::Rotate;
        capture_rotate_start_ = tnow;
        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Rotating to yaw=%.2f (step %.0f°)",
          capture_target_yaw_, step_angle * 180.0 / M_PI);
      } else {
        // save_current_image() returned false — camera not providing data?
        // Log a warning (throttled so we don't spam) to help diagnose.
        RCLCPP_WARN_THROTTLE(ctx.get_logger(), *ctx.clock, 5000,
          "[till_obstacle_back] save_current_image returned false — "
          "no color frame on image topic?");
      }
      break;
    }

    // =====================================================================
    // CaptureStep::BeepSequence — non-blocking triple-beep sequence
    // =====================================================================
    case CaptureStep::BeepSequence: {
      if (tnow < beep_toggle_deadline_) {
        break;
      }

      if (!beep_is_on_) {
        if (beep_remaining_ <= 0) {
          ctx.set_buzzer(false);
          if (completion_beep_pending_) {
            completion_beep_pending_ = false;
            return finish_capture();
          }
          capture_step_ = CaptureStep::Rotate;
          capture_rotate_start_ = tnow;
          RCLCPP_INFO(ctx.get_logger(),
            "[till_obstacle_back] Rotating to yaw=%.2f after pre-rotation beeps",
            capture_target_yaw_);
          break;
        }
        ctx.set_buzzer(true);
        beep_is_on_ = true;
        beep_toggle_deadline_ = tnow + rclcpp::Duration::from_seconds(0.20);
      } else {
        ctx.set_buzzer(false);
        beep_is_on_ = false;
        beep_remaining_--;
        beep_toggle_deadline_ = tnow + rclcpp::Duration::from_seconds(0.20);
      }
      break;
    }

    // =====================================================================
    // CaptureStep::Rotate — spin toward the target yaw
    // =====================================================================
    case CaptureStep::Rotate: {
      double x, y, yaw;
      if (!ctx.get_odom_pose(x, y, yaw)) {
        ctx.stop_robot();
        break;
      }

      // --- Outlier rejection ---
      // The Yahboom serial link can deliver stale then burst-updated
      // encoder data, causing yaw jumps of ~1 rad between ticks.
      // Max physically possible delta per tick ≈ capture_turn_speed * dt.
      // We use 4× margin to allow for measurement noise while still
      // catching the ~1 rad glitches seen in practice.
      double yaw_delta = 0.0;
      if (last_good_yaw_valid_) {
        yaw_delta = std::abs(normalize_angle(yaw - last_good_yaw_));
        const double max_delta = ctx.capture_turn_speed * 0.20;  // 4× a 50ms tick
        if (yaw_delta > max_delta) {
          RCLCPP_WARN_THROTTLE(ctx.get_logger(), *ctx.clock, 2000,
            "[till_obstacle_back] Odom glitch rejected: yaw=%.3f last=%.3f delta=%.3f",
            yaw, last_good_yaw_, yaw_delta);
          append_buffered_log(ctx, "Rejected yaw glitch delta=" + std::to_string(yaw_delta));
          yaw = last_good_yaw_;
          yaw_delta = 0.0;
        }
      }
      last_good_yaw_ = yaw;
      last_good_yaw_valid_ = true;

      const double rotate_elapsed = (tnow - capture_rotate_start_).seconds();
      if (rotate_elapsed > 15.0) {
        RCLCPP_WARN(ctx.get_logger(),
          "[till_obstacle_back] Rotation step timeout (%.0fs) — skipping", rotate_elapsed);
        ctx.stop_robot();
        capture_step_ = CaptureStep::Settle;
        capture_settle_end_ = tnow + rclcpp::Duration::from_seconds(settle_sec);
        break;
      }

      const double err = normalize_angle(capture_target_yaw_ - yaw);

      RCLCPP_INFO_THROTTLE(ctx.get_logger(), *ctx.clock, 3000,
        "[till_obstacle_back] ROTATE step %d: err=%.3f yaw=%.2f stuck=%d",
        frames_saved_, err, yaw, stuck_tick_count_);

      if (std::abs(err) < capture_yaw_tol) {
        ctx.stop_robot();
        capture_step_ = CaptureStep::Settle;
        capture_settle_end_ = tnow + rclcpp::Duration::from_seconds(settle_sec);
        stuck_tick_count_ = 0;
        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Reached target yaw (err=%.3f rad, took %.1fs)",
          err, rotate_elapsed);
      } else {
        double w = std::clamp(err * ctx.capture_kp,
          -ctx.capture_turn_speed, ctx.capture_turn_speed);

        if (std::abs(w) < ctx.capture_min_angular_speed) {
          w = std::copysign(ctx.capture_min_angular_speed, w);
        }

        // --- Unstuck detection ---
        // If odom appears frozen (yaw barely changing despite commanding
        // rotation), boost speed.  The Rosmaster serial link sometimes
        // stops reporting encoder updates; a stronger command often
        // "kicks" the mechanism past the stall/deadband.
        if (yaw_delta < 0.005) {
          stuck_tick_count_++;
        } else {
          stuck_tick_count_ = 0;
        }
        // ~1 second of no movement (20 ticks at 50ms) → double the speed
        if (stuck_tick_count_ > 20) {
          const double boosted = std::copysign(
            std::min(std::abs(w) * 2.0, ctx.capture_turn_speed), w);
          RCLCPP_WARN_THROTTLE(ctx.get_logger(), *ctx.clock, 5000,
            "[till_obstacle_back] Odom stuck (%d ticks) — boosting w %.2f→%.2f",
            stuck_tick_count_, w, boosted);
          w = boosted;
        }

        ctx.publish_twist(0.0, 0.0, w);
      }
      break;
    }

    // =====================================================================
    // CaptureStep::Settle — wait for camera to stabilize
    // =====================================================================
    // After the robot stops rotating, the camera image is still slightly
    // blurry from residual vibration and auto-exposure adjustment.
    // Waiting 0.8 seconds produces a sharper photo.
    case CaptureStep::Settle: {
      if (tnow >= capture_settle_end_) {
        capture_step_ = CaptureStep::Snap;  // Ready for next photo!
      }
      // No motion command needed — the robot is already stopped.
      break;
    }
  }

  return PatrolSignal::Continue;
}

// ===========================================================================
// Phase 4: TurnAround — rotate 180° using the SAME motion profile as capture
// ===========================================================================
// After all photos are taken and emailed, the robot faces roughly the same
// direction as when capture started (because we did a full 360°).  We turn
// 180° so the robot faces the OPPOSITE direction — ready for the next patrol
// cycle to drive forward into new territory.
//
// WHY STEP-BASED (and not one big rotation):
//   Mecanum chassis show much higher wheel slip during SUSTAINED rotation
//   than during repeated short rotations.  The capture sweep does 12 × 30°
//   bursts with a settle between each — that motion profile happens to give
//   odom-to-physical ≈ 1:1 (after calibrating odom_angular_scale).  A single
//   uninterrupted 180° rotation under the SAME controller gave only ~100°
//   physical because the rollers keep skidding once steady-state spin is
//   reached.  The fix: replicate the capture's many-short-bursts profile.
//
// STRATEGY:
//   - step_angle = 2π / capture_frame_count   (e.g. 30° for 12 frames)
//   - turn_steps_total = round(π / step_angle) (e.g. 6 steps for 30° each)
//   - Each step uses the SAME P-controller, min-speed floor, outlier
//     rejection, and stuck detection as tick_capture.
//   - Absolute targets anchored to turn_start_yaw_ prevent accumulated
//     per-step undershoot (same trick the capture loop uses).
//   - Short 0.3s settle between steps (no photo to wait for, but the
//     start/stop cadence is what reduces slip).
// ===========================================================================
PatrolSignal TillObstacleBackPattern::tick_turn_around(PatrolContext & ctx)
{
  const auto tnow = ctx.now();

  double x, y, yaw;
  if (!ctx.get_odom_pose(x, y, yaw)) {
    ctx.stop_robot();
    return PatrolSignal::Continue;
  }

  // step_angle matches the capture sweep so step dynamics are identical
  const double step_angle = 2.0 * M_PI / ctx.capture_frame_count;
  const double settle_sec = 0.3;
  const double per_step_timeout = 15.0;

  // --- First-tick setup ---
  if (!turn_target_set_) {
    turn_start_yaw_ = yaw;
    turn_steps_total_ = std::max(1,
      static_cast<int>(std::round(M_PI / step_angle)));
    turn_steps_done_ = 0;
    turn_step_ = TurnStep::Rotate;
    turn_target_yaw_ = normalize_angle(turn_start_yaw_ + step_angle);
    turn_step_start_ = tnow;
    turn_target_set_ = true;
    last_good_yaw_ = yaw;
    last_good_yaw_valid_ = true;
    stuck_tick_count_ = 0;
    RCLCPP_INFO(ctx.get_logger(),
      "[till_obstacle_back] Turning 180° via %d × %.1f° steps — first target=%.2f",
      turn_steps_total_, step_angle * 180.0 / M_PI, turn_target_yaw_);
    append_buffered_log(ctx, "Turn-around started");
  }

  // Phase-level timeout (raised from 30s — multi-step turn is slower)
  const double elapsed = (tnow - phase_start_).seconds();
  if (elapsed > 60.0) {
    RCLCPP_WARN(ctx.get_logger(),
      "[till_obstacle_back] Turn-around timeout after %.1fs (%d/%d steps done)",
      elapsed, turn_steps_done_, turn_steps_total_);
    ctx.stop_robot();
    enter_phase(Phase::Done, ctx);
    append_buffered_log(ctx, "Turn-around timeout; forcing done");
    return PatrolSignal::Continue;
  }

  switch (turn_step_) {

    // =====================================================================
    // TurnStep::Rotate — spin toward the current step's target yaw
    // =====================================================================
    case TurnStep::Rotate: {
      // Per-step timeout — skip the step rather than blocking forever.
      const double step_elapsed = (tnow - turn_step_start_).seconds();
      if (step_elapsed > per_step_timeout) {
        RCLCPP_WARN(ctx.get_logger(),
          "[till_obstacle_back] Turn step %d/%d timeout (%.0fs) — skipping",
          turn_steps_done_ + 1, turn_steps_total_, step_elapsed);
        ctx.stop_robot();
        turn_step_ = TurnStep::Settle;
        turn_settle_end_ = tnow + rclcpp::Duration::from_seconds(settle_sec);
        break;
      }

      // --- Yaw outlier rejection (identical to tick_capture) ---
      double yaw_delta = 0.0;
      if (last_good_yaw_valid_) {
        yaw_delta = std::abs(normalize_angle(yaw - last_good_yaw_));
        const double max_delta = ctx.capture_turn_speed * 0.20;  // 4× a 50ms tick
        if (yaw_delta > max_delta) {
          RCLCPP_WARN_THROTTLE(ctx.get_logger(), *ctx.clock, 2000,
            "[till_obstacle_back] Turn odom glitch rejected: yaw=%.3f last=%.3f delta=%.3f",
            yaw, last_good_yaw_, yaw_delta);
          append_buffered_log(ctx,
            "Rejected turn yaw glitch delta=" + std::to_string(yaw_delta));
          yaw = last_good_yaw_;
          yaw_delta = 0.0;
        }
      }
      last_good_yaw_ = yaw;
      last_good_yaw_valid_ = true;

      const double err = normalize_angle(turn_target_yaw_ - yaw);

      RCLCPP_INFO_THROTTLE(ctx.get_logger(), *ctx.clock, 3000,
        "[till_obstacle_back] TURN step %d/%d err=%.3f yaw=%.2f tgt=%.2f stuck=%d",
        turn_steps_done_ + 1, turn_steps_total_, err, yaw, turn_target_yaw_,
        stuck_tick_count_);

      if (std::abs(err) < ctx.capture_yaw_tolerance) {
        ctx.stop_robot();
        turn_step_ = TurnStep::Settle;
        turn_settle_end_ = tnow + rclcpp::Duration::from_seconds(settle_sec);
        stuck_tick_count_ = 0;
        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Turn step %d/%d reached (err=%.3f, %.1fs)",
          turn_steps_done_ + 1, turn_steps_total_, err, step_elapsed);
        break;
      }

      // --- P-controller (identical to tick_capture) ---
      double w = std::clamp(err * ctx.capture_kp,
        -ctx.capture_turn_speed, ctx.capture_turn_speed);
      if (std::abs(w) < ctx.capture_min_angular_speed) {
        w = std::copysign(ctx.capture_min_angular_speed, w);
      }

      // --- Stuck detection (identical to tick_capture) ---
      if (yaw_delta < 0.005) {
        stuck_tick_count_++;
      } else {
        stuck_tick_count_ = 0;
      }
      if (stuck_tick_count_ > 20) {  // ~1 second at 20 Hz
        const double boosted = std::copysign(
          std::min(std::abs(w) * 2.0, ctx.capture_turn_speed), w);
        RCLCPP_WARN_THROTTLE(ctx.get_logger(), *ctx.clock, 5000,
          "[till_obstacle_back] Turn step %d stuck (%d ticks) — boosting w %.2f→%.2f",
          turn_steps_done_ + 1, stuck_tick_count_, w, boosted);
        w = boosted;
      }

      ctx.publish_twist(0.0, 0.0, w);
      break;
    }

    // =====================================================================
    // TurnStep::Settle — short pause between step rotations
    // =====================================================================
    // The settle period is what makes this profile match the capture sweep
    // (mecanum slip is much lower in start/stop bursts than in sustained
    // rotation).  We don't need to wait for camera stabilization, just
    // long enough to fully decelerate before the next burst.
    case TurnStep::Settle: {
      if (tnow < turn_settle_end_) {
        break;
      }

      turn_steps_done_++;
      if (turn_steps_done_ >= turn_steps_total_) {
        RCLCPP_INFO(ctx.get_logger(),
          "[till_obstacle_back] Turn complete — %d steps done, pattern done",
          turn_steps_done_);
        // Fire the "patrol finished" sound (e.g. bark.wav) once the final
        // 180° turn is finished.  Non-blocking — see PatrolNode::
        // play_completion_sound().  Safe to call even if the callback was
        // not provided (treat as no-op).
        if (ctx.play_completion_sound) {
          ctx.play_completion_sound();
        }
        enter_phase(Phase::Done, ctx);
        append_buffered_log(ctx, "Turn-around complete");
        maybe_flush_buffered_logs(ctx, true);
        return PatrolSignal::DoneIdle;
      }

      // Set next absolute target (anchored to turn_start_yaw_ to prevent
      // per-step undershoot from accumulating across the 180°).
      turn_target_yaw_ = normalize_angle(
        turn_start_yaw_ + (turn_steps_done_ + 1) * step_angle);
      turn_step_ = TurnStep::Rotate;
      turn_step_start_ = tnow;
      stuck_tick_count_ = 0;
      // Re-prime outlier rejection so the new step starts from current yaw
      last_good_yaw_ = yaw;
      last_good_yaw_valid_ = true;
      RCLCPP_INFO(ctx.get_logger(),
        "[till_obstacle_back] Turn step %d/%d target=%.2f",
        turn_steps_done_ + 1, turn_steps_total_, turn_target_yaw_);
      break;
    }
  }

  return PatrolSignal::Continue;
}

}  // namespace cat_patrol_robot
