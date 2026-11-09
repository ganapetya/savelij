// =============================================================================
// till_obstacle_back_pattern.hpp — "Drive to obstacle, reverse home, snap 360°"
// =============================================================================
//
// PATROL BEHAVIOR (4 phases):
//   Phase 1: DriveForward   — go straight until depth camera sees an obstacle
//   Phase 2: DriveBackHome  — reverse back to the starting position (odometry)
//   Phase 3: CaptureImages  — spin in place taking N photos across a full 360°
//   Phase 4: TurnAround     — rotate 180° to face forward again
//
// C++ CONCEPT: HEADER ORGANIZATION
//   In C++ a class is typically split across TWO files:
//     .hpp (header)  — the class DECLARATION: what methods/members exist
//     .cpp (source)  — the class DEFINITION: the actual code bodies
//
//   Why split?  Compilation speed.  When you change a .cpp file, only that
//   file recompiles.  If you change a .hpp file, EVERY file that includes
//   it recompiles.  So headers should contain the minimum needed:
//   declarations, not implementations (with small exceptions like inline
//   one-liners in the class body).
//
// C++ CONCEPT: INHERITANCE (": public PatrolPattern")
//   TillObstacleBackPattern inherits from PatrolPattern.  This means:
//     - It gets all of PatrolPattern's interface (on_start, tick, etc.)
//     - It MUST override every pure virtual (= 0) method
//     - A pointer/reference to PatrolPattern can point to this class
//       (polymorphism — the node uses "unique_ptr<PatrolPattern>")
//
//   "public" inheritance means: PatrolPattern's public methods stay public
//   in the derived class.  "private" inheritance would hide them.
// =============================================================================
#ifndef CAT_PATROL_ROBOT__PATTERNS__TILL_OBSTACLE_BACK_PATTERN_HPP_
#define CAT_PATROL_ROBOT__PATTERNS__TILL_OBSTACLE_BACK_PATTERN_HPP_

#include <string>
#include <vector>

#include "cat_patrol_robot/patterns/patrol_pattern.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cat_patrol_robot
{

class TillObstacleBackPattern : public PatrolPattern
{
public:
  // -------------------------------------------------------------------------
  // Overridden virtual methods from PatrolPattern
  // -------------------------------------------------------------------------
  // "override" keyword — see patrol_pattern.hpp for explanation.
  void on_start(PatrolContext & ctx) override;
  PatrolSignal tick(PatrolContext & ctx) override;
  void on_stop(PatrolContext & ctx) override;

  // name() is defined right here in the header body.  When a function is
  // this short (one line), writing it inline is fine and saves creating
  // a trivial .cpp entry.  "const" means this method doesn't modify
  // any member variables.
  std::string name() const override { return "till_obstacle_back_images_turn"; }

  // -------------------------------------------------------------------------
  // Public tuning parameters (set by the node after construction)
  // -------------------------------------------------------------------------
  // These use "default member initializers" (C++11): the value after the
  // brace is used unless the caller overwrites it.  The node reads
  // forward_timeout_sec from YAML and writes it here.
  //
  // C++ CONCEPT: PUBLIC vs PRIVATE members
  //   These are public so the node can set them directly (simple structs
  //   don't need setters for plain data).  Internal bookkeeping variables
  //   below are private — outside code shouldn't touch them.
  double forward_timeout_sec{60.0};      // Max seconds to drive forward
  double drive_back_timeout_sec{60.0};   // Max seconds to drive backward
  double capture_completion_extra_deg{0.0};  // Extra final rotation before completion beep
  double log_buffer_period_sec{60.0};    // Keep logs in-memory before flush
  std::string log_file_path{"/tmp/cat_patrol_pattern.log"};  // Buffered log target file

private:
  // =========================================================================
  // Phase enum — the pattern's own state machine
  // =========================================================================
  // C++ CONCEPT: NESTED ENUM CLASS
  //   You can define an enum inside a class.  It's scoped to this class:
  //     Phase::DriveForward  (inside the class)
  //     TillObstacleBackPattern::Phase::DriveForward  (from outside)
  //   This keeps it private — only this class uses it.
  enum class Phase
  {
    DriveForward,    // Phase 1: drive until obstacle
    DriveBackHome,   // Phase 2: reverse to home
    CaptureImages,   // Phase 3: 360° photo sweep
    TurnAround,      // Phase 4: rotate 180° and stop
    Done,            // Finished — signal idle
  };

  Phase phase_{Phase::DriveForward};  // Current phase
  rclcpp::Time phase_start_;          // When current phase began (for timeouts)

  // =========================================================================
  // Capture sub-state machine (runs inside Phase::CaptureImages)
  // =========================================================================
  // The capture phase is itself a mini state machine with three sub-steps
  // that repeat: Snap → Rotate → Settle → Snap → Rotate → Settle → ...
  //
  //   Snap:    save the current camera frame to a JPEG
  //   Rotate:  spin to the next yaw target (closed-loop via odometry)
  //   Settle:  wait 0.8s for camera image to stabilize after rotation
  enum class CaptureStep { Snap, BeepSequence, Rotate, Settle };
  CaptureStep capture_step_{CaptureStep::Snap};
  rclcpp::Time capture_settle_end_;   // When the settle wait finishes
  int frames_saved_{0};               // How many photos saved so far
  std::vector<std::string> saved_paths_;  // File paths of saved JPEGs
  int beep_remaining_{0};             // Pulses left in the current beep sequence
  bool beep_is_on_{false};            // Current beep-sequence output state
  bool completion_beep_pending_{false};  // True when finishing 360° after beeps
  rclcpp::Time beep_toggle_deadline_; // Next beep sequence toggle time

  // =========================================================================
  // Closed-loop yaw tracking for precise 360° panoramic sweep
  // =========================================================================
  // ROBOTICS CONCEPT: OPEN-LOOP vs CLOSED-LOOP CONTROL
  //
  //   Open-loop:  "spin for 0.3 seconds at 3 rad/s" — HOPES that equals 0.9 rad.
  //     Problem: battery, friction, load, and motor response all affect actual
  //     rotation.  On a real robot, you might get 0.7 rad or 1.1 rad.
  //
  //   Closed-loop: "spin until odometry says I've rotated 0.524 rad from start"
  //     — MEASURES actual rotation and adjusts.  Much more reliable.
  //
  //   We use odometry yaw (from TF: odom → base_footprint) as feedback.
  //   For each photo position, we compute an absolute target:
  //     target_yaw = start_yaw + photo_index * (2π / N)
  //   Then a proportional controller rotates toward it.
  //
  //   After the LAST photo, one final rotation returns to start_yaw,
  //   completing a full 360°.
  double capture_start_yaw_{0.0};     // Yaw when capture phase began
  double capture_target_yaw_{0.0};    // Where we're currently rotating TO
  rclcpp::Time capture_rotate_start_; // When this rotation step began (timeout)

  // Odometry noise filtering for yaw during capture rotation.
  // The Yahboom Rosmaster serial link can drop/delay encoder updates,
  // causing sudden yaw jumps of ~1 rad.  We reject readings that exceed
  // the physically possible delta per tick.
  double last_good_yaw_{0.0};         // Last accepted yaw reading
  bool last_good_yaw_valid_{false};   // Have we recorded a reading yet?
  int stuck_tick_count_{0};           // Ticks where yaw hasn't changed

  // Short buzzer beep on each capture — provides audible feedback
  bool capture_buzzer_on_{false};     // Is the beep currently active?
  rclcpp::Time buzzer_off_time_;      // When to turn it off (150ms after snap)

  // =========================================================================
  // Turn-around tracking (Phase 4)
  // =========================================================================
  // The final 180° rotation is broken into N small step rotations using the
  // SAME motion profile as the capture sweep (step_angle = 2π / N_frames).
  // This matters because mecanum chassis slip differs between sustained
  // rotation and brief start/stop bursts — by replicating the capture's
  // many-short-bursts pattern, the two rotations share an odometry-to-
  // physical ratio, so a single odom_angular_scale calibrates both.
  enum class TurnStep { Rotate, Settle };
  double turn_start_yaw_{0.0};           // Yaw when turn-around began
  double turn_target_yaw_{0.0};          // Current step's absolute target yaw
  int    turn_steps_total_{6};           // How many step rotations to cover 180°
  int    turn_steps_done_{0};            // How many step rotations completed
  TurnStep turn_step_{TurnStep::Rotate}; // Current sub-state
  rclcpp::Time turn_settle_end_;         // When the inter-step settle finishes
  rclcpp::Time turn_step_start_;         // When the current step's rotation began (timeout)
  bool turn_target_set_{false};          // Have we initialized the turn-around sequence?

  // Buffered file logging (phase-aware).
  std::vector<std::string> buffered_logs_;
  rclcpp::Time log_window_start_;
  bool log_flush_pending_{false};

  // =========================================================================
  // Private methods — one per phase
  // =========================================================================
  // C++ CONVENTION: "tick_xxx" methods
  //   Each phase has its own method.  The main tick() dispatches to the
  //   right one based on the current phase.  This keeps each method short
  //   and focused (Single Responsibility Principle).
  void enter_phase(Phase p, PatrolContext & ctx);
  PatrolSignal tick_drive_forward(PatrolContext & ctx);
  PatrolSignal tick_drive_back_home(PatrolContext & ctx);
  PatrolSignal tick_capture(PatrolContext & ctx);
  PatrolSignal tick_turn_around(PatrolContext & ctx);
  void append_buffered_log(PatrolContext & ctx, const std::string & message);
  void maybe_flush_buffered_logs(PatrolContext & ctx, bool force);
};

}  // namespace cat_patrol_robot

#endif  // CAT_PATROL_ROBOT__PATTERNS__TILL_OBSTACLE_BACK_PATTERN_HPP_
