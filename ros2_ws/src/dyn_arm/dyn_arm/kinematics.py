"""Shared math: converts between motor ticks, joint angles, and the gripper-tip height.

Single source of truth for both the floor guard (arm_node) and the converters
(joint_command_node, floor_probe_node), so they can never disagree.

Convention: 2048 ticks = the straight-up pose = each joint angle 0 (measured FROM vertical).
Both joint signs are +, so bending the elbow and wrist the same physical way ADDS in
(elbow + wrist), which is what the real linkage does.
"""
import math

RAD_PER_TICK = 2 * math.pi / 4096   # 4096 ticks = one full revolution


# ---------------------------------------------------------------------------
# Forward kinematics: the virtual floor
# ---------------------------------------------------------------------------
def joint_angles(elbow_tick, wrist_tick, zero_tick=2048):
    """Elbow and wrist angles in radians from the straight-up zero."""
    e = (elbow_tick - zero_tick) * RAD_PER_TICK
    w = (wrist_tick - zero_tick) * RAD_PER_TICK
    return e, w


def tip_height(elbow_tick, wrist_tick,
               elbow_axis_h=0.04475, link1=0.140, link2=0.192, zero_tick=2048):
    """Gripper-tip height above the mounting surface (m):

        height = elbow_axis_height + L1*cos(elbow) + L2*cos(elbow + wrist)

    Pure cosine because both joints only ever lower the tip from straight up, so the
    maximum (~377 mm) is at the straight-up pose.
    """
    e, w = joint_angles(elbow_tick, wrist_tick, zero_tick)
    return elbow_axis_h + link1 * math.cos(e) + link2 * math.cos(e + w)


# ---------------------------------------------------------------------------
# Joint calibration: tick <-> radian per joint (for RViz <-> motors)
# ---------------------------------------------------------------------------
# idx = position in the [gripper, wrist, elbow] tick array (motor ids 1, 2, 3).
# lo/hi = real measured travel in radians (so the twin shows the true reachable range).
CALIB = {
    "elbow":   {"idx": 2, "zero": 2048, "sign": +1.0, "scale": RAD_PER_TICK, "lo": -1.730, "hi": 1.727},
    "wrist":   {"idx": 1, "zero": 2048, "sign": +1.0, "scale": RAD_PER_TICK, "lo": -2.442, "hi": 2.441},
    "gripper": {"idx": 0, "zero": 2048, "sign": +1.0, "scale": 0.00158,      "lo": -0.20,  "hi": 0.425},
}


def ticks_to_rad(name, tick):
    """One joint's tick value -> clamped radians (for the RViz twin)."""
    c = CALIB[name]
    ang = (tick - c["zero"]) * c["scale"] * c["sign"]
    return max(c["lo"], min(c["hi"], ang))


def rad_to_ticks(name, rad):
    """One joint's radians -> tick value (for commanding the motor)."""
    c = CALIB[name]
    rad = max(c["lo"], min(c["hi"], rad))
    return int(round(c["zero"] + rad / (c["scale"] * c["sign"])))
