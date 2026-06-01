# gripper.py
# Handles Robotiq 3F gripper motor initialisation and open/close commands.

GRIPPER_NAMES = [
    "finger_1_joint_1",
    "finger_2_joint_1",
    "finger_middle_joint_1"
]

_gripper_motors = []


def init(robot, time_step):
    """Call once from the main controller after creating the Supervisor."""
    global _gripper_motors
    _gripper_motors = []
    for name in GRIPPER_NAMES:
        motor = robot.getDevice(name)
        motor.setVelocity(0.5)
        _gripper_motors.append(motor)


def open_gripper():
    for motor in _gripper_motors:
        motor.setPosition(0.05)


def close_gripper():
    for motor in _gripper_motors:
        motor.setPosition(0.8)
