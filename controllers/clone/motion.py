# motion.py
# UR5e joint motor / sensor setup and all arm-motion helpers.

UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

_robot      = None
_time_step  = None
_ur_motors  = []
_ur_sensors = []


def init(robot, time_step):
    """Call once from the main controller after creating the Supervisor."""
    global _robot, _time_step, _ur_motors, _ur_sensors
    _robot     = robot
    _time_step = time_step
    _ur_motors  = []
    _ur_sensors = []

    for name in UR_JOINT_NAMES:
        motor  = robot.getDevice(name)
        sensor = robot.getDevice(name + "_sensor")
        motor.setVelocity(0.8)
        sensor.enable(time_step)
        _ur_motors.append(motor)
        _ur_sensors.append(sensor)

# ------------------------------------------------------------------
# Primitives
# ------------------------------------------------------------------

def move_ur_to(q):
    """Instantly command all UR joints to positions in q."""
    for motor, pos in zip(_ur_motors, q):
        motor.setPosition(pos)


def get_current_joint_positions():
    return [sensor.getValue() for sensor in _ur_sensors]


def wait_steps(n):
    for _ in range(n):
        if _robot.step(_time_step) == -1:
            return False
    return True


# ------------------------------------------------------------------
# Compound moves
# ------------------------------------------------------------------

def smooth_move_ur_to(target_q, steps=100):
    """Linearly interpolate from current position to target_q over `steps` sim steps."""
    start_q = get_current_joint_positions()

    for i in range(steps + 1):
        ratio = i / steps
        q = [start + ratio * (target - start)
             for start, target in zip(start_q, target_q)]
        move_ur_to(q)
        if _robot.step(_time_step) == -1:
            return False

    return True


def wait_until_reached(target_q, tolerance=0.02, max_steps=300):
    """Spin the sim until all joints are within tolerance of target_q."""
    for _ in range(max_steps):
        current_q = get_current_joint_positions()
        errors = [abs(c - t) for c, t in zip(current_q, target_q)]
        if max(errors) < tolerance:
            return True
        if _robot.step(_time_step) == -1:
            return False
    return False


def goto_pose(target_q, steps=100, tolerance=0.02):
    """Smooth move then wait until physically reached."""
    smooth_move_ur_to(target_q, steps=steps)
    wait_until_reached(target_q, tolerance=tolerance)

