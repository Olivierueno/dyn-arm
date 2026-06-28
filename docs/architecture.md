# Design notes

## Commanding the arm

Joint targets come from the standard `joint_state_publisher_gui` (one slider per joint),
which publishes `/joint_states` in radians. Two consumers read it:

- `robot_state_publisher` turns it into TF and the RViz **twin**.
- `joint_command_node` converts radians to motor **ticks** and publishes `/goal_positions`,
  which `arm_node` drives the motors from.

So the twin and the real arm are driven from the same source and always agree.
(This project began as a webcam mimic; that input was removed to keep it focused, and it is
recoverable from git history if ever wanted.)

## Coordinates and calibration

Everything is in raw Dynamixel ticks (4096 ticks = one revolution) until the last moment.
`2048` is defined as the **straight-up** pose, where every joint angle is zero. The per-joint
tick<->radian mapping (`CALIB` in `kinematics.py`) is the single place that conversion lives,
used both to drive the motors (radians->ticks) and to draw the twin (ticks->radians).

One servo horn was bolted on ~65 deg off true. Rather than fight the spline, it was corrected
in firmware with the **Homing Offset** register (control-table address 20): the motor adds a
fixed offset to every reported position, so the physical neutral reads as 2048 to all
software above it. It is the digital equivalent of taring a load cell.

## The virtual floor (forward kinematics)

Per-joint limits are a box in joint space; "the gripper must not hit the table" is a diagonal
constraint that couples the elbow and wrist. We compute the gripper-tip height directly:

```
tip_height = elbow_axis_height + L1*cos(elbow) + L2*cos(elbow + wrist)
```

- Angles are measured **from vertical**, so vertical height uses `cos` (a link tilted by
  theta from vertical contributes `L*cos(theta)`). At straight up both cosines are 1, giving
  the maximum tip height (~377 mm); any bend lowers it.
- Both joint signs are **+**, so bending the elbow and wrist the same physical way **adds**
  in `(elbow + wrist)`, matching the real linkage. (A wrong sign here cancels the two angles
  and silently over-estimates the height, a bug found and fixed during bring-up.)

`arm_node` rejects any goal whose predicted tip height is below `floor_margin`, holding the
last safe pose instead. The formula was validated against a tape measure during bring-up by
back-driving the arm (torque off) and comparing the predicted tip height to reality.

## Why current-based holding, not "cut torque on overcurrent"

A tempting failsafe is to watch motor current and disable torque on a spike. That is wrong
for a vertical arm: cutting torque drops it **into** the obstacle. Instead the motors run in
current-based position control (operating mode 5) with a per-joint torque cap, so contact is
a soft current-limited stall that still holds the arm up. This is foldback current limiting,
not an emergency stop.

## Motion smoothing

Motion stays calm via a command **deadband** in the driver (ignore sub-degree goal wobble),
the Dynamixel **profile acceleration/velocity** (trapezoidal ramps instead of instant jerk),
and per-joint **P/D gains**. The light gripper in particular needed high damping (D) and low
stiffness (P) to stop it hunting and chattering at idle.
