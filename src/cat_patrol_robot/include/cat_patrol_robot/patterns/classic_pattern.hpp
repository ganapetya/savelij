// =============================================================================
// classic_pattern.hpp — Simple forward/backward patrol (no obstacle sensing)
// =============================================================================
//
// BEHAVIOR:
//   The robot drives forward for patrol_drive_sec seconds, then backward
//   for the same duration, repeating for 4 total "legs" (2 forward, 2 back).
//   After all legs, it signals DoneNextCapture → the node takes panoramic
//   photos using its own capture_tick().
//
// This is a MINIMAL example of a PatrolPattern.  Compare it with
// TillObstacleBackPattern to see how the Strategy pattern allows very
// different behaviors behind the same interface.
//
// C++ CONCEPT: SIMPLE DERIVED CLASS
//   This class only needs to override 3 methods and store 1 variable.
//   Contrast with TillObstacleBackPattern which has dozens of members.
//   The PatrolPattern interface lets both exist side by side — the node
//   doesn't care which one it's using.
// =============================================================================
#ifndef CAT_PATROL_ROBOT__PATTERNS__CLASSIC_PATTERN_HPP_
#define CAT_PATROL_ROBOT__PATTERNS__CLASSIC_PATTERN_HPP_

#include "cat_patrol_robot/patterns/patrol_pattern.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cat_patrol_robot
{

class ClassicPattern : public PatrolPattern
{
public:
  void on_start(PatrolContext & ctx) override;
  PatrolSignal tick(PatrolContext & ctx) override;

  // No on_stop override needed — default (do nothing) is fine because
  // the base class version handles it.

  std::string name() const override { return "classic"; }

private:
  // The only state this pattern needs: when did the patrol start?
  // All timing is computed relative to this timestamp.
  rclcpp::Time start_time_;
};

}  // namespace cat_patrol_robot

#endif  // CAT_PATROL_ROBOT__PATTERNS__CLASSIC_PATTERN_HPP_
