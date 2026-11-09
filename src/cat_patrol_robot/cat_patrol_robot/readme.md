# Cat Patrol Robot — Quick Summary

## What it does
Autonomously patrols an area, detects cats, takes panoramic photos, and emails them.

## The brain
A single ROS 2 node (`PatrolNode`) running a finite state machine at **20 Hz**.

## States
```
WaitingForTf → Patrol → Capture → ReturnHome → Idle  (loops)
                  ↘ CatApproach ↗   (detour if a cat is spotted)
```

## Key hardware abstractions
- **Wheels** — via `cmd_vel` topic (`Twist` messages)
- **Camera** — color images for snapshots, depth images for obstacle detection
- **LiDAR** — optional front-cone obstacle detection
- **Buzzer** — audible alerts (cat found, photo taken, heartbeat)
- **TF** — odometry tells the robot where it is so it can navigate home

## Patrol patterns (swappable via config)
- **ClassicPattern** — drives forward/backward for a fixed time
- **TillObstacleBackPattern** — drives until hitting an obstacle, reverses home,
  snaps a full 360° panorama, turns around

## End result
JPEG photos saved to disk + a JSON mail request published → a Python `mail_node`
picks that up and sends the email.
