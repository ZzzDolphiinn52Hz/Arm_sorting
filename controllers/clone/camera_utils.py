# camera_utils.py
# Camera setup and all cube-detection / tracking helpers.

import math

_robot      = None
_time_step  = None
_camera     = None


def init(robot, time_step, camera_name="camera"):
    """Call once from the main controller after creating the Supervisor."""
    global _robot, _time_step, _camera
    _robot     = robot
    _time_step = time_step

    _camera = robot.getDevice(camera_name)
    _camera.enable(time_step)
    _camera.recognitionEnable(time_step)

    print("Camera enabled:", camera_name)
    print("Camera width:",  _camera.getWidth())
    print("Camera height:", _camera.getHeight())


# ------------------------------------------------------------------
# Detection
# ------------------------------------------------------------------

def detect_cube():
    """
    Returns a dict with pixel coords, area, 3-D position and model name
    for the first recognised object, or None if nothing is in view.
    """
    objects = _camera.getRecognitionObjects()
    if not objects:
        return None

    obj             = objects[0]
    pos_on_image    = obj.getPositionOnImage()   # pixel x, y
    size_on_image   = obj.getSizeOnImage()        # pixel w, h
    position_3d     = obj.getPosition()           # x, y, z relative to robot
    model_name      = obj.getModel()

    return {
        "cx":          pos_on_image[0],
        "cy":          pos_on_image[1],
        "area":        size_on_image[0] * size_on_image[1],
        "position_3d": position_3d,
        "model":       model_name
    }


# ------------------------------------------------------------------
# Tracking / waiting
# ------------------------------------------------------------------

def wait_until_cube_ready(move_ur_to_fn, get_joints_fn, max_steps=500):
    """
    Track the cube across the conveyor and return the locked base angle
    once the cube is centred in the pick zone, or None on timeout.

    move_ur_to_fn   -- motion.move_ur_to
    get_joints_fn   -- motion.get_current_joint_positions
    """
    print("Tracking and following the cube along the conveyor...")

    image_center_x = _camera.getWidth()  / 2
    image_center_y = _camera.getHeight() / 2

    PICK_ZONE_X  = 15
    PICK_ZONE_Y  = 250
    MIN_CUBE_AREA = 150
    Kp = -0.0015

    for i in range(max_steps):
        result = detect_cube()

        if result is not None:
            cx, cy, area, pos_3d, model = result["cx"], result["cy"], result["area"], result["position_3d"], result["model"]
            error_x = cx - image_center_x
            error_y = abs(cy - image_center_y)

            current_q    = get_joints_fn()
            current_q[0] += Kp * error_x
            
            # Làm tròn từng phần tử bên trong danh sách
            readable_pos_3d = [round(val, 3) for val in list(pos_3d)]
            readable_q = [round(val, 6) for val in list(current_q)]
            display_model = model if model != "" else "Unknown_Cube"

            
            # Di chuyển lệnh in này vào trong block i % 10 để tránh làm tràn màn hình Console
            
            print(f"[DEBUG] Vật: {display_model} | Tọa độ 3D: {readable_pos_3d}")
            print(f"q: {readable_q}")

                            
            move_ur_to_fn(current_q)

            if i % 10 == 0:
                print(f"Tracking -> cx: {round(cx,1)}, cy: {round(cy,1)}, "
                      f"error_y: {round(error_y,1)}, Base Angle: {round(current_q[0],3)}")

            if abs(error_x) < PICK_ZONE_X and error_y < PICK_ZONE_Y and area > MIN_CUBE_AREA:
                print(f"-> TARGET LOCKED! Base angle: {round(current_q[0],3)}. Proceeding to pick!")
                return current_q[0]
        else:
            if i % 20 == 0:
                print("Searching for cube...")

        if _robot.step(_time_step) == -1:
            return None

    print("Timeout: Cube moved past or was missed.")
    return None


def dynamic_descend_and_pick(move_ur_to_fn, get_joints_fn, pick_down_pose, steps=25):
    """
    Lower the arm towards pick_down_pose while continuously correcting the
    base joint to follow the cube in real time.
    """
    print("Descending to pick while tracking cube...")
    start_q  = get_joints_fn()
    target_q = list(pick_down_pose)
    image_center_x = _camera.getWidth() / 2
    Kp = -0.0015

    for i in range(steps + 1):
        ratio     = i / steps
        current_q = get_joints_fn()
        base_angle = current_q[0]

        result = detect_cube()
        if result is not None:
            error_x     = result["cx"] - image_center_x
            base_angle += Kp * error_x

        next_q = [base_angle] + [
            start_q[j] + ratio * (target_q[j] - start_q[j])
            for j in range(1, 6)
        ]
        move_ur_to_fn(next_q)

        if _robot.step(_time_step) == -1:
            return False

    return True


def check_if_picked_successfully(wait_steps_fn):
    """
    After lifting, verify the cube is still close to the camera (i.e. in the gripper).
    wait_steps_fn -- motion.wait_steps
    """
    wait_steps_fn(10)

    result = detect_cube()
    if result is None:
        print("-> [CONFIRM]: Object NOT seen in gripper. Pick missed completely!")
        return False

    pos_3d   = result["position_3d"]
    distance = math.sqrt(pos_3d[0]**2 + pos_3d[1]**2 + pos_3d[2]**2)
    print(f"Distance from camera to object after lift: {round(distance, 3)} m")

    if distance < 0.25:
        print("-> [CONFIRM]: Pick SUCCESSFUL! Carrying to bin.")
        return True
    else:
        print("-> [CONFIRM]: Pick MISSED! Object still on conveyor.")
        return False


def preview_camera(n_steps=200):
    print("Preview camera before picking...")
    for _ in range(n_steps):
        if _robot.step(_time_step) == -1:
            return False
    return True

def count_detected_cubes():
    """
    Đếm và trả về số lượng vật thể đang xuất hiện trong tầm nhìn của camera.
    """
    objects = _camera.getRecognitionObjects()
    if not objects:
        return 0  # Không có khối nào trong tầm nhìn
        
    total_cubes = len(objects)
    print(f"[VISION]: Phát hiện {total_cubes} khối hộp trong khung hình!")
    return total_cubes

def count_cubes_by_model(target_model_name="cube"):
    """
    Chỉ đếm các vật thể có tên model trùng với mục tiêu chỉ định
    """
    objects = _camera.getRecognitionObjects()
    if not objects:
        return 0
        
    count = 0
    for obj in objects:
        if target_model_name in obj.getModel().lower():
            count += 1
            
    print(f"[FILTER]: Tìm thấy {count} vật thể thuộc nhóm '{target_model_name}'")
    return count