# Lab 1 – ROS 2, Turtlesim, Actions & Services
**Design of Mechatronics Systems – Alexandria University, Faculty of Engineering**

## Team
| Name | ID |
|------|----|
| Mina Naim | 9754 |
| Yassa Emad | 10395 |

## Packages
| Package | Type | Contents |
|---------|------|----------|
| `turtle_interfaces` | ament_cmake | `msg/TargetPose` (x, y, yaw), `action/GoToPose`, `action/DrawSquare`, `srv/SetSpeedLimits` |
| `turtle_control` | ament_python | `go_to_pose_server` (action + speed parameters + `set_speed_limits` service), `reset_service`, `draw_square_server`, pure-Python `motion.py` / `waypoints.py`, unit tests |
| `turtle_bringup` | ament_python | `launch/bringup.launch.py`, `launch/square_demo.launch.py`, `config/turtle_params.yaml` |

### Architecture
```
 draw_square (action) ──► draw_square_server ──► reset_turtle (srv) ──► reset_service ──► /clear, /turtle1/set_pen, /turtle1/teleport_absolute
                                   └──4 goals──► go_to_pose (action) ──► go_to_pose_server ──► /turtle1/cmd_vel  ◄── /turtle1/pose
 set_speed_limits (srv) ─────────────────────────────► go_to_pose_server (params: max_linear_speed, max_angular_speed)
```

### Interfaces
| Name | Fields |
|------|--------|
| `turtle_interfaces/msg/TargetPose` | `float32 x, y, yaw` |
| `turtle_interfaces/action/GoToPose` | goal: `TargetPose target` · result: `success, message, final_distance_error, final_yaw_error` · feedback: `distance_remaining, yaw_remaining, phase` |
| `turtle_interfaces/action/DrawSquare` | goal: `side_length` (0 → default 3.0) · result: `success, message, sides_completed` · feedback: `current_side, status` |
| `turtle_interfaces/srv/SetSpeedLimits` | request: `max_linear_speed, max_angular_speed` · response: `success, message` |
| `std_srvs/srv/Trigger` (`/reset_turtle`) | reset uses a standard type |

## Build
```bash
source /opt/ros/<distro>/setup.bash          # humble / jazzy ...
mkdir -p ~/lab1_ws/src && cd ~/lab1_ws/src
git clone <your-repo-url> .                  # package folders + README only
cd ~/lab1_ws
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash                    # in EVERY new terminal
```

## Run
```bash
ros2 launch turtle_bringup bringup.launch.py                       # whole system
ros2 launch turtle_bringup bringup.launch.py max_linear_speed:=1.0 max_angular_speed:=1.5
ros2 launch turtle_bringup square_demo.launch.py side_length:=3.0  # system + square automatically
```
Nodes started: `turtlesim`, `go_to_pose_server`, `reset_service`, `draw_square_server`.

## Commands to try every requirement
| Req | Command / check |
|-----|-----------------|
| **R1** turtlesim | `ros2 node list` → `/turtlesim`; `ros2 topic list`, `ros2 service list` |
| **R2** message | `ros2 interface show turtle_interfaces/msg/TargetPose` |
| **R3** goToPose | `ros2 action send_goal /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 8.0, y: 3.0, yaw: 1.57}}" --feedback` then `ros2 topic echo /turtle1/pose --once` |
| **R4** reset | `ros2 service call /reset_turtle std_srvs/srv/Trigger` |
| **R5** square | `ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}" --feedback` (max side ≈ 5.1) |
| **R6** launch | `ros2 launch turtle_bringup bringup.launch.py` – nothing started by hand |
| **R7** parameters | `ros2 param list /go_to_pose_server` · `ros2 param get /go_to_pose_server max_linear_speed` · `ros2 param set /go_to_pose_server max_linear_speed 0.8` (use a decimal point) · `ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.0}"` |
| **R7** invalid | `ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: -1.0, max_angular_speed: 1.0}"` → `success: false`, limits unchanged |
| **R8** packages | see table above |

Speed limits are read on **every** control tick (50 Hz), so changing them while the turtle moves takes effect immediately.

### Behaviour details
* **goToPose** state machine: rotate to face target → drive (P control, steering correction) → rotate to final yaw. Tolerances: 0.05 position, 0.03 rad yaw (parameters). Goals outside `[0, 11]` or while another goal runs are rejected; goals can be canceled.
* **reset**: `set_pen(off)` → `teleport_absolute(5.544, 5.544, 0)` → wait 0.15 s → `clear` → `set_pen(on)`. The wait matters: turtlesim applies a teleport in its own 60 Hz loop, so switching the pen on immediately could draw a line. Safe to call repeatedly.
* **square**: calls reset, then sends exactly 4 sequential `go_to_pose` goals (corners counter-clockwise, last corner = start). Rejects squares that leave the window.

## Bonus 1 – Design patterns
| Pattern | Where | Why |
|---------|-------|-----|
| **State** | `turtle_control/motion.py` – `RotateToHeading`, `DriveToPoint`, `RotateToYaw`, `Done`, context `GoToPoseController` | Each phase of goToPose has its own logic and transition rules; no big if/else in the ROS node. |
| **Strategy** | `motion.py` – `SaturatedP` injected into the states | The control law is swappable (e.g. PID) without touching the states. |
| **Factory** | `turtle_control/waypoints.py` – `PathFactory.create('square', ...)` | The square server asks for a path by name; new shapes just register a class. |
| **Observer** | ROS pub/sub (pose subscription, feedback topics) | The node reacts to pose updates instead of polling turtlesim. |

The motion logic has no ROS imports, so it is unit-tested with a simulated turtle:
```bash
cd ~/lab1_ws && colcon test --packages-select turtle_control && colcon test-result --verbose
```
(11 tests: all directions, speed limits never exceeded, live speed change, square geometry, validation.)

## Bonus 2 – Bag file
```bash
# terminal 1
ros2 launch turtle_bringup bringup.launch.py
# terminal 2: record, then run the square
ros2 bag record -o square_bag /turtle1/cmd_vel /turtle1/pose
ros2 action send_goal /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}"
# Ctrl+C to stop recording
ros2 bag info square_bag
# playback: reset first, then replay the velocity commands
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 bag play square_bag --topics /turtle1/cmd_vel
```
![Playback](turtle_bringup/bags/playback.png)

## Troubleshooting
| Problem | Fix |
|---------|-----|
| Package / launch file not found | Source `install/setup.bash` in that terminal; rebuild |
| Goal rejected | Check `ros2 action list`; target must be inside `[0, 11]`; only one goal at a time |
| `param set` refused | Value must be a decimal number in `(0, 10]` |
