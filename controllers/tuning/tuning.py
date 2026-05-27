from controller import Supervisor, Keyboard

TIME_STEP = 32

robot = Supervisor()
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

ur_motors = []
ur_sensors = []

for name in UR_JOINT_NAMES:
    motor = robot.getDevice(name)
    sensor = robot.getDevice(name + "_sensor")

    motor.setVelocity(0.5)
    sensor.enable(TIME_STEP)

    ur_motors.append(motor)
    ur_sensors.append(sensor)

# Pose ban Äáº§u
target_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

selected_joint = 0
step_angle = 0.01


def move_ur_to(q):
    for motor, pos in zip(ur_motors, q):
        motor.setPosition(pos)


def print_current_pose():
    current_pose = []

    for sensor in ur_sensors:
        current_pose.append(sensor.getValue())

    print("Current pose:")
    print("[", end="")
    for i, value in enumerate(current_pose):
        if i < len(current_pose) - 1:
            print(f"{value:.3f}, ", end="")
        else:
            print(f"{value:.3f}", end="")
    print("]")


move_ur_to(target_positions)

print("=== TEACH MODE ===")
print("Press 1-6 to select joint")
print("Press Q to decrease angle")
print("Press E to increase angle")
print("Press P to print current pose")

while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()

    if key == ord('1'):
        selected_joint = 0
        print("Selected joint 1: shoulder_pan_joint")

    elif key == ord('2'):
        selected_joint = 1
        print("Selected joint 2: shoulder_lift_joint")

    elif key == ord('3'):
        selected_joint = 2
        print("Selected joint 3: elbow_joint")

    elif key == ord('4'):
        selected_joint = 3
        print("Selected joint 4: wrist_1_joint")

    elif key == ord('5'):
        selected_joint = 4
        print("Selected joint 5: wrist_2_joint")

    elif key == ord('6'):
        selected_joint = 5
        print("Selected joint 6: wrist_3_joint")

    elif key == ord('Q'):
        target_positions[selected_joint] -= step_angle
        move_ur_to(target_positions)

    elif key == ord('E'):
        target_positions[selected_joint] += step_angle
        move_ur_to(target_positions)

    elif key == ord('P'):
        print_current_pose()