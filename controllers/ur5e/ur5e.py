from controller import Robot, Keyboard
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# UR5e joint motor names
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

motors = []

for name in JOINT_NAMES:
    motor = robot.getDevice(name)
    motors.append(motor)
    motor.setVelocity(0.6)

# A safe initial pose
target = [
    0.0,
    -1.57,
    1.57,
    -1.57,
    -1.57,
    0.0
]

# Joint limit safety
LOWER_LIMIT = [-6.28, -6.28, -6.28, -6.28, -6.28, -6.28]
UPPER_LIMIT = [ 6.28,  6.28,  6.28,  6.28,  6.28,  6.28]

selected_joint = 0
step_angle = 0.02


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def apply_target():
    for i in range(6):
        target[i] = clamp(target[i], LOWER_LIMIT[i], UPPER_LIMIT[i])
        motors[i].setPosition(target[i])


apply_target()

print("UR5e controller started.")
print("Keys:")
print("  1-6 : select joint")
print("  Z   : decrease selected joint angle")
print("  X   : increase selected joint angle")
print("  H   : home pose")
print("  R   : ready pose")

while robot.step(timestep) != -1:
    key = keyboard.getKey()

    while key != -1:
        if key >= ord("1") and key <= ord("6"):
            selected_joint = key - ord("1")
            print(f"Selected UR5e joint: {selected_joint + 1}")

        elif key == ord("Z") or key == ord("z"):
            target[selected_joint] -= step_angle

        elif key == ord("X") or key == ord("x"):
            target[selected_joint] += step_angle

        elif key == ord("H") or key == ord("h"):
            target = [
                0.0,
                -1.57,
                1.57,
                -1.57,
                -1.57,
                0.0
            ]
            print("UR5e home pose")

        elif key == ord("R") or key == ord("r"):
            target = [
                0.0,
                -1.2,
                1.4,
                -1.8,
                -1.57,
                0.0
            ]
            print("UR5e ready pose")

        key = keyboard.getKey()

    apply_target()