from controller import Supervisor

TIME_STEP = 32
robot = Supervisor()

# =========================
# CAMERA SETUP
# =========================

CAMERA_NAME = "camera"   # Nếu sai tên, mình sẽ đổi sau

camera = robot.getDevice(CAMERA_NAME)
camera.enable(TIME_STEP)

print("Camera enabled:", CAMERA_NAME)
print("Camera width:", camera.getWidth())
print("Camera height:", camera.getHeight())

def detect_white_cube():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    sum_x = 0
    sum_y = 0
    count = 0

    max_brightness = 0
    brightest_pixel = (0, 0, 0)

    sample_step = 4

    for y in range(0, height, sample_step):
        for x in range(0, width, sample_step):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            brightness = (r + g + b) / 3

            if brightness > max_brightness:
                max_brightness = brightness
                brightest_pixel = (r, g, b)

            # Nhận diện vật trắng nhưng nới điều kiện
            if r > 140 and g > 140 and b > 140 and abs(r - g) < 50 and abs(g - b) < 50:
                sum_x += x
                sum_y += y
                count += 1

    if count < 10:
        return None, max_brightness, brightest_pixel

    center_x = sum_x / count
    center_y = sum_y / count

    return (center_x, center_y, count), max_brightness, brightest_pixel
    
def wait_until_cube_ready(max_steps=500):
    print("Waiting for white cube to enter pick zone...")

    image_center_x = camera.getWidth() / 2
    image_center_y = camera.getHeight() / 2

    PICK_ZONE_X = 120
    PICK_ZONE_Y = 120
    MIN_CUBE_AREA = 150

    for i in range(max_steps):
        result, max_brightness, brightest_pixel = detect_white_cube()

        if result is not None:
            cx, cy, area = result

            error_x = abs(cx - image_center_x)
            error_y = abs(cy - image_center_y)

            if i % 10 == 0:
                print(
                    "Cube:",
                    "cx =", round(cx, 1),
                    "cy =", round(cy, 1),
                    "area =", area,
                    "error_x =", round(error_x, 1),
                    "error_y =", round(error_y, 1)
                )

            if error_x < PICK_ZONE_X and error_y < PICK_ZONE_Y and area > MIN_CUBE_AREA:
                print("Cube is ready to pick!")
                print("Final cx:", round(cx, 1), "cy:", round(cy, 1), "area:", area)
                return True

        else:
            if i % 20 == 0:
                print("No white cube detected")

        if robot.step(TIME_STEP) == -1:
            return False

    print("Timeout: cube did not enter pick zone")
    return False
    
def preview_and_detect_cube(n_steps=300):
    print("Preview and detect white cube...")

    for i in range(n_steps):
        result, max_brightness, brightest_pixel = detect_white_cube()

        if result is not None:
            cx, cy, area = result

            if i % 10 == 0:
                print(
                    "White cube detected:",
                    "cx =", round(cx, 1),
                    "cy =", round(cy, 1),
                    "area =", area,
                    "max_brightness =", round(max_brightness, 1),
                    "brightest_rgb =", brightest_pixel
                )
        else:
            if i % 10 == 0:
                print(
                    "No white cube detected",
                    "max_brightness =", round(max_brightness, 1),
                    "brightest_rgb =", brightest_pixel
                )

        if robot.step(TIME_STEP) == -1:
            return False

    return True
    
UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

GRIPPER_NAMES = [
    "finger_1_joint_1",
    "finger_2_joint_1",
    "finger_middle_joint_1"
]

ur_motors = []
ur_sensors = []

for name in UR_JOINT_NAMES:
    motor = robot.getDevice(name)
    sensor = robot.getDevice(name + "_sensor")
    motor.setVelocity(0.8)
    sensor.enable(TIME_STEP)
    ur_motors.append(motor)
    ur_sensors.append(sensor)

gripper_motors = []

for name in GRIPPER_NAMES:
    motor = robot.getDevice(name)
    motor.setVelocity(0.5)
    gripper_motors.append(motor)


def move_ur_to(q):
    for motor, pos in zip(ur_motors, q):
        motor.setPosition(pos)


def wait_steps(n):
    for _ in range(n):
        if robot.step(TIME_STEP) == -1:
            return False
    return True


def open_gripper():
    for motor in gripper_motors:
        motor.setPosition(0.05)


def close_gripper():
    for motor in gripper_motors:
        motor.setPosition(0.8)
        
def get_current_joint_positions():
    return [sensor.getValue() for sensor in ur_sensors]


def smooth_move_ur_to(target_q, steps=100):
    start_q = get_current_joint_positions()

    for i in range(steps + 1):
        ratio = i / steps

        q = []
        for start, target in zip(start_q, target_q):
            value = start + ratio * (target - start)
            q.append(value)

        move_ur_to(q)

        if robot.step(TIME_STEP) == -1:
            return False

    return True
    
def wait_until_reached(target_q, tolerance=0.02, max_steps=300):
    for _ in range(max_steps):
        current_q = [sensor.getValue() for sensor in ur_sensors]

        errors = []
        for current, target in zip(current_q, target_q):
            errors.append(abs(current - target))

        if max(errors) < tolerance:
            return True

        if robot.step(TIME_STEP) == -1:
            return False

    return False
    
def goto_pose(target_q, steps=100, tolerance=0.02):
    smooth_move_ur_to(target_q, steps=steps)
    wait_until_reached(target_q, tolerance=tolerance)
    
def preview_camera(n_steps=200):
    print("Preview camera before picking...")

    for _ in range(n_steps):
        if robot.step(TIME_STEP) == -1:
            return False

    return True
    
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0] 

PICK_ABOVE = [-0.000, -1.210, 1.370, -1.770, -1.590, -0.000] 

PICK_DOWN = [-0.000, -1.010, 1.590, -2.170, -1.553, -0.020]

BIN_ABOVE = [3.060, -1.550, 1.050, -1.050, -1.490, -0.080]  

BIN_DOWN = [3.060, -1.550, 1.450, -1.450, -1.490, -0.080] 

SAFE_MID = [2.410, -1.570, 1.150, -1.210, -1.570, 0.000]

while robot.step(TIME_STEP) != -1:
    # 1. Về vị trí HOME
    goto_pose(HOME, steps=100)

    # 2. Mở gripper
    open_gripper()
    wait_steps(50)

    # 3. Di chuyển tới phía trên vật
    goto_pose(PICK_ABOVE, steps=100)
    
    # 3.1 Chờ cube trắng đi vào giữa camera
    if not wait_until_cube_ready(max_steps=500):
        continue
    
    # 4. Hạ xuống vị trí gắp
    goto_pose(PICK_DOWN, steps=100)

    # 5. Đóng gripper để gắp vật
    close_gripper()
    wait_steps(80)

    # 6. Nâng vật lên
    goto_pose(PICK_ABOVE, steps=100)
    goto_pose(SAFE_MID, steps=100)

    # 7. Di chuyển tới phía trên thùng đỏ
    goto_pose(BIN_ABOVE, steps=120)

    # 8. Hạ xuống vị trí thả
    goto_pose(BIN_DOWN, steps=100)

    # 9. Mở gripper để thả vật
    open_gripper()
    wait_steps(80)

    # 10. Nâng tay lên khỏi thùng
    goto_pose(BIN_ABOVE, steps=100)
    goto_pose(SAFE_MID, steps=100)

    # 11. Quay về HOME
    goto_pose(HOME, steps=120)