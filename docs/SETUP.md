# Setup

This is the short build path for this arm.

## Parts

- 2x Dynamixel XM430-W210-R.
- 1x Dynamixel XM540-W270-R.
- 1x U2D2 USB serial converter with USB-C cable.
- 1x 12 V 30 A power supply.
- Wooden platform.
- Printed parts from `ros2_ws/src/dyn_arm/meshes/`.
- 8x M3 x 10 mm bolts.
- 16x M3 x 6 mm bolts.
- 8x M2.5 x 10 mm bolts.
- 8x M2.5 x 8 mm bolts.

Print the structural STL files in PLA at 50% infill.

## Assembly

1. Mount the XM540 as the main elbow/base motor.
2. Mount the two XM430 motors as wrist and gripper joints.
3. Bolt the printed structure to the wooden platform.
4. Connect all servos on the Dynamixel TTL bus.
5. Connect the bus to the U2D2.
6. Power the servos from the 12 V supply, not USB.

Before torque is enabled, move the arm by hand and check for binding or cable
snags.

## ROS2

Ubuntu 22.04 with ROS2 Humble:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-dynamixel-sdk \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  python3-colcon-common-extensions

pip install dynamixel-sdk
sudo usermod -aG dialout $USER
```

Log out and back in after adding `dialout`.

## WSL2 notes

If running from Windows/WSL2, attach the U2D2 with `usbipd-win`. This repo
includes:

```powershell
windows\attach_u2d2.ps1
windows\start_vcxsrv.ps1
```

The U2D2 should appear in Linux as `/dev/ttyUSB0`.

## Build

```bash
cd ros2_ws
colcon build
source install/setup.bash
cd ..
```

## First motor setup

```bash
python3 tools/motor_tool.py scan
python3 tools/motor_tool.py zero
python3 tools/motor_tool.py find-limits
```

Copy the measured limits into `ros2_ws/src/dyn_arm/config/arm_params.yaml`, then:

```bash
python3 tools/motor_tool.py set-limits
```

Use small movements first:

```bash
bash tools/ros_launch.sh display.launch.py
bash tools/ros_launch.sh arm.launch.py
```

Run the demo after slider control behaves correctly:

```bash
bash tools/ros_launch.sh demo.launch.py
```
