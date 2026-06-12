"""Central configuration for the UR5e dynamic sorting demo."""

import numpy as np

# Webots timing
TIME_STEP = 32
DT = TIME_STEP / 1000.0

# Gripper / flange orientation
# Valid values supported by motion.make_gripper_down_rotation():
# "z", "-z", "x", "-x", "y", "-y".
GRIPPER_DOWN_AXIS = "z"

# Pick geometry, in meters
CUBE_HALF_HEIGHT = 0.025
PICK_CLEARANCE = 0.020
ABOVE_CLEARANCE = 0.160

# TCP/tip offset expressed in FLANGE local frame.
# Tune X/Y if the gripper tip is laterally offset from the flange center.
GRIPPER_TIP_OFFSET_FLANGE = np.array([0.000, 0.000, 0.120])

# Dynamic tracking prediction, in seconds
LEAD_TIME_ABOVE = 0.28
LEAD_TIME_DOWN = 0.14
LEAD_TIME_CLOSE = 0.08

# Tracker filters
MAX_JOINT_STEP = 0.070
CUBE_ALPHA = 0.42
VEL_ALPHA = 0.20
LOST_MAX_STEPS = 12
MAX_CUBE_SPEED = 0.70
MIN_CUBE_AREA = 80

# Pick workspace / lane limits in robot base frame
PICK_X_MIN = -1.35
PICK_X_MAX = -0.25
PICK_X_GOAL = -0.72
PICK_X_BEST = -0.38
PICK_Y_LIMIT = 0.80
PICK_RADIUS_MAX = 0.98
LOCK_DISTANCE = 0.12

TOO_LATE_GRACE_STEPS = 18
IK_FAIL_GRACE_STEPS = 35

# Set to a bin name during single-bin testing, or None for full sorting.
TEST_ONLY_BIN = None

WORKSPACE = {
    "x_min": PICK_X_MIN,
    "x_max": PICK_X_MAX,
    "x_goal": PICK_X_GOAL,
    "y_limit": PICK_Y_LIMIT,
    "r_max": PICK_RADIUS_MAX,
}

# Logging rate control
DYNAMIC_WAIT_LOG_PERIOD = 15
DYNAMIC_TRACK_LOG_PERIOD = 10
