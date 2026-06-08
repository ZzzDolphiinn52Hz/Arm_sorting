# camera_utils.py
# Camera setup and all cube-detection / tracking helpers.

import math
import numpy as np

_robot      = None
_time_step  = None
_camera     = None

def get_cube_in_base_frame(pos_3d_camera, T_base_flange):
    """
    Convert cube position from Camera frame -> Flange frame -> Base frame.

    pos_3d_camera: [x, y, z] from Webots Camera Recognition
    T_base_flange: FK matrix, Base <- Flange
    """

    x_c, y_c, z_c = pos_3d_camera

    # TODO: cần calibrate đúng ma trận này theo camera thật trong Webots.
    T_FLANGE_CAMERA = np.array([
    [ 7.0000e-06, -1.0000e+00,  1.7000e-05,  9.2760e-03 ],
    [ 1.2000e-05, -1.7000e-05, -1.0000e+00, -2.0070e-02 ],
    [ 1.0000e+00,  7.0000e-06,  1.2000e-05, 6.0071e-02 ],
    [ 0.0 , 0.0 , 0.0 , 1.0 ],
    ])

    p_camera = np.array([x_c, y_c, z_c, 1.0])

    p_base = T_base_flange @ T_FLANGE_CAMERA @ p_camera

    return p_base[0:3]

def get_cube_base_position(T_base_flange):
    """
    Trả về vị trí cube trong hệ Base robot.
    """
    cube_data = detect_cube()
    if cube_data is None:
        return None

    pos_camera = cube_data["position_3d"]
    cube_base = get_cube_in_base_frame(pos_camera, T_base_flange)

    return cube_base


def get_cube_world_position(T_base_flange, T_world_base):
    """
    Trả về vị trí cube trong hệ World Webots.
    """
    cube_base = get_cube_base_position(T_base_flange)
    if cube_base is None:
        return None

    p_base = np.array([cube_base[0], cube_base[1], cube_base[2], 1.0])
    p_world = T_world_base @ p_base

    return p_world[0:3]

def debug_check_camera_transform(pos_3d_camera, T_base_flange, T_world_base, supervisor_cube_node):
    """
    HÀM KIỂM TRA BƯỚC 2: Đối chiếu tọa độ vật tính từ Ma trận Camera sang World với tọa độ thực tế.
    """
    # Tính vị trí vật đối với hệ trục Base Robot
    cube_in_base = get_cube_in_base_frame(pos_3d_camera, T_base_flange)
    
    # Chuyển tiếp tọa độ vật từ hệ Base sang hệ World của Webots để đối chứng với Supervisor
    T_base_cube = np.identity(4)
    T_base_cube[0:3, 3] = cube_in_base
    T_world_cube = T_world_base @ T_base_cube
    calc_cube_world = T_world_cube[0:3, 3]
    
    # Lấy tọa độ thực tế của khối hộp trực tiếp từ Webots Supervisor
    actual_cube_world = supervisor_cube_node.getPosition()
    
    print("\n=== [CHECK STEP 2] CAMERA TO BASE MATRIX VERIFICATION ===")
    print(f"Toán ma trận tính ra (World): {[round(p, 4) for p in calc_cube_world]}")
    print(f"Supervisor Webots đo (World): {[round(a, 4) for a in actual_cube_world]}")
    
    # Tính sai số khoảng cách đường thẳng
    error = np.linalg.norm(np.array(actual_cube_world) - calc_cube_world)
    print(f"--> Sai số tuyệt đối phép dịch ma trận camera: {round(error, 6)} mét")
    
    if error < 0.015:  # Ngưỡng chấp nhận sai số cho camera quét động là dưới 1.5 cm
        print("[STATUS]: BƯỚC 2 ĐẠT CHUẨN! Hệ tọa độ thị giác đã đồng bộ hoàn toàn.")
        return True
    else:
        print("[WARNING]: Sai lệch hệ trục camera! Kiểm tra lại định hướng camera hoặc offset.")
        return False

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