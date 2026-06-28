# Tools

Two helpers, both run in WSL.

### `motor_tool.py` - direct-to-motor utilities (not a ROS node)
One CLI for every low-level motor task. U2D2 attached, 12 V on:
```bash
python3 tools/motor_tool.py scan                 # find motors across baud rates
python3 tools/motor_tool.py read                 # pos / goal / torque / error per motor
python3 tools/motor_tool.py zero                 # center all motors (2048) and hold, for assembly
python3 tools/motor_tool.py home-offset 1 2048   # calibrate a joint zero in firmware
python3 tools/motor_tool.py find-limits          # back-drive by hand, capture min/max ticks
python3 tools/motor_tool.py teach                # pose the arm by hand, record demo poses
python3 tools/motor_tool.py set-limits           # burn config lo/hi into motor EEPROM
```
Commissioning order: `zero` -> `home-offset` (if a horn is clocked off) -> `find-limits` ->
copy the numbers into `ros2_ws/src/dyn_arm/config/arm_params.yaml` -> `set-limits`.

### `ros_launch.sh` - the ROS2 launcher
Sources the workspace and sets the RViz display, then launches:
```bash
bash tools/ros_launch.sh arm.launch.py          # sliders + twin + real motors
bash tools/ros_launch.sh demo.launch.py         # hands-free choreography; twin follows
bash tools/ros_launch.sh display.launch.py      # RViz twin only (no motors)
```
