# camera_utils.py
# Camera setup and cube-detection / tracking helpers.
# Optimized version: stable multi-cube selection + target locking.

import math
import numpy as np

_robot      = None
_time_step  = None
_camera     = None
_locked_cube_id = None

# Calibrated transform: Flange <- Camera.
# If camera is moved in the Webots scene, recalibrate this matrix.
T_FLANGE_CAMERA = np.array([
    [ 7.0000e-06, -1.0000e+00,  1.7000e-05,  9.2760e-03 ],
    [ 1.2000e-05, -1.7000e-05, -1.0000e+00, -2.0070e-02 ],
    [ 1.0000e+00,  7.0000e-06,  1.2000e-05,  6.0071e-02 ],
    [ 0.0,         0.0,         0.0,          1.0        ],
])


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


def reset_target_lock():
    """Forget the currently tracked recognition object."""
    global _locked_cube_id
    _locked_cube_id = None


def set_target_lock(cube_id):
    """Lock future detections to one Webots recognition object id when available."""
    global _locked_cube_id
    _locked_cube_id = cube_id


def get_target_lock():
    return _locked_cube_id


def _safe_get_id(obj):
    """Webots Python wrappers differ by version; support both method spellings."""
    for name in ("getId", "get_id"):
        if hasattr(obj, name):
            try:
                return getattr(obj, name)()
            except Exception:
                pass
    return None


def _model_matches(model, target_model_name="cube"):
    if target_model_name is None or target_model_name == "":
        return True

    # Some Webots recognition objects return an empty model string even when
    # recognitionColors are configured. Keep empty model accepted so cubes are
    # not accidentally ignored.
    if model is None or model == "":
        return True

    return target_model_name.lower() in model.lower()


def _object_to_detection(obj):
    pos_on_image  = obj.getPositionOnImage()   # pixel x, y
    size_on_image = obj.getSizeOnImage()       # pixel w, h
    position_3d   = obj.getPosition()          # camera frame
    model_name    = obj.getModel()

    return {
        "id":          _safe_get_id(obj),
        "cx":          float(pos_on_image[0]),
        "cy":          float(pos_on_image[1]),
        "w":           float(size_on_image[0]),
        "h":           float(size_on_image[1]),
        "area":        float(size_on_image[0] * size_on_image[1]),
        "position_3d": np.array(position_3d, dtype=float),
        "model":       model_name,
        "object":      obj,
    }


def detect_cubes(target_model_name="cube", min_area=0):
    """
    Return all recognized cube-like objects as dictionaries.
    This is the safe replacement for choosing objects[0].
    """
    objects = _camera.getRecognitionObjects()
    if not objects:
        return []

    detections = []
    for obj in objects:
        det = _object_to_detection(obj)
        if det["area"] < min_area:
            continue
        if not _model_matches(det["model"], target_model_name):
            continue
        detections.append(det)

    return detections


def detect_cube(target_model_name="cube", min_area=0):
    """
    Backward-compatible API.
    Return one stable cube detection, not simply the first recognition object.
    """
    detections = detect_cubes(target_model_name=target_model_name, min_area=min_area)
    if not detections:
        return None

    # Keep the same object while a pick is in progress.
    if _locked_cube_id is not None:
        for det in detections:
            if det["id"] == _locked_cube_id:
                return det

    # Fallback: choose the largest visible cube because it is usually the most
    # reliable 3-D measurement. The dynamic tracker will override this with
    # base-frame scoring when multiple cubes are present.
    return max(detections, key=lambda d: d["area"])

# ------------------------------------------------------------------
# Multi-object detection: shape + color debug only
# ------------------------------------------------------------------

TARGET_SHAPES = ["cube", "box", "sphere", "ball", "cylinder"]

COLOR_TABLE = {
    "red":    np.array([1.0, 0.0, 0.0]),
    "green":  np.array([0.0, 1.0, 0.0]),
    "blue":   np.array([0.0, 0.0, 1.0]),
    "yellow": np.array([1.0, 1.0, 0.0]),
    "white":  np.array([1.0, 1.0, 1.0]),
    "black":  np.array([0.0, 0.0, 0.0]),
}


def classify_shape_from_model(model_name):
    model = (model_name or "").lower()

    if "cube" in model or "box" in model:
        return "cube"

    if "sphere" in model or "ball" in model:
        return "sphere"

    if "cylinder" in model:
        return "cylinder"

    return "unknown"


def classify_color_from_model(model_name):
    model = (model_name or "").lower()

    for color_name in COLOR_TABLE.keys():
        if color_name in model:
            return color_name

    return "unknown"


def read_recognition_color_rgb(obj):
    """
    Đọc recognitionColors từ Webots CameraRecognitionObject.

    Lưu ý:
    obj.getColors() có thể trả về ctypes pointer LP_c_double,
    nên không được dùng len(colors).
    """
    try:
        colors = obj.getColors()
    except Exception:
        return None

    if colors is None:
        return None

    # Nếu Webots có hàm getNumberOfColors(), dùng để check có màu hay không
    try:
        n_colors = obj.getNumberOfColors()
        if n_colors <= 0:
            return None
    except Exception:
        pass

    # getColors() thường là flat array/pointer: [r, g, b, r, g, b, ...]
    try:
        return np.array([
            float(colors[0]),
            float(colors[1]),
            float(colors[2])
        ], dtype=float)
    except Exception:
        return None


def classify_color_from_rgb(rgb):
    if rgb is None:
        return "unknown"

    rgb = np.array(rgb, dtype=float)

    best_color = "unknown"
    best_dist = 999.0

    for color_name, ref_rgb in COLOR_TABLE.items():
        dist = np.linalg.norm(rgb - ref_rgb)

        if dist < best_dist:
            best_dist = dist
            best_color = color_name

    return best_color


def detect_objects_shape_color():
    """
    Debug vision only:
    - Không gắp
    - Không tracking
    - Chỉ trả về danh sách object camera đang thấy
    """
    objects = _camera.getRecognitionObjects()
    results = []

    if not objects:
        return results

    for obj in objects:
        model_name = obj.getModel() or ""

        shape_name = classify_shape_from_model(model_name)

        # Bỏ qua các object không thuộc nhóm cần test
        if shape_name == "unknown":
            continue

        pos_on_image = obj.getPositionOnImage()
        size_on_image = obj.getSizeOnImage()
        position_3d = obj.getPosition()

        rgb = read_recognition_color_rgb(obj)
        color_name = classify_color_from_rgb(rgb)

        # Fallback: nếu không đọc được recognitionColors thì lấy màu từ model name
        if color_name == "unknown":
            color_name = classify_color_from_model(model_name)

        try:
            obj_id = obj.getId()
        except Exception:
            obj_id = None

        results.append({
            "id": obj_id,
            "model": model_name,
            "shape": shape_name,
            "color": color_name,
            "rgb": rgb,
            "cx": pos_on_image[0],
            "cy": pos_on_image[1],
            "area": size_on_image[0] * size_on_image[1],
            "position_3d": position_3d,
        })

    return results

# ------------------------------------------------------------------
# Object -> Bin classification debug
# ------------------------------------------------------------------

SORT_RULES = {
    ("cube", "blue"): "BLUE_CUBE_BIN",
    ("cube", "red"): "RED_CUBE_BIN",
    ("cylinder", "yellow"): "YELLOW_CYLINDER_BIN",
    ("sphere", "green"): "GREEN_SPHERE_BIN",
}


def classify_bin_from_shape_color(shape, color):
    """
    Nhận shape + color, trả về tên thùng phân loại.
    """
    return SORT_RULES.get((shape, color), "UNKNOWN_BIN")


def detect_objects_with_bin():
    """
    Detect object như cũ, nhưng thêm field 'bin'.
    Chưa gắp, chưa tracking, chỉ phân loại.
    """
    objects = detect_objects_shape_color()

    for obj in objects:
        obj["bin"] = classify_bin_from_shape_color(
            obj["shape"],
            obj["color"]
        )

    return objects

def get_objects_base_with_bin(T_base_flange):
    """
    Lấy tất cả object camera thấy được,
    có sẵn shape/color/bin,
    rồi đổi position_3d từ camera frame sang base frame.
    """
    objects = detect_objects_with_bin()

    for obj in objects:
        obj["base_position"] = get_cube_in_base_frame(
            obj["position_3d"],
            T_base_flange
        )

    return objects


def debug_print_objects_base_with_bin(T_base_flange):
    objects = get_objects_base_with_bin(T_base_flange)

    print(f"\n[BASE SORT DEBUG] objects = {len(objects)}")

    for i, obj in enumerate(objects):
        p = obj.get("base_position", None)

        if p is None:
            p_text = None
        else:
            p_text = [round(float(v), 4) for v in p]

        print(
            f"[{i}] "
            f"id={obj.get('id')} | "
            f"shape={obj.get('shape')} | "
            f"color={obj.get('color')} | "
            f"bin={obj.get('bin')} | "
            f"base={p_text}"
        )
        
def debug_print_detected_objects_with_bin():
    objects = detect_objects_with_bin()

    print(f"\n[SORT DEBUG] detected_objects = {len(objects)}")

    for i, obj in enumerate(objects):
        pos_text = [round(float(v), 3) for v in obj["position_3d"]]

        print(
            f"[{i}] "
            f"id={obj['id']} | "
            f"model={obj['model']} | "
            f"shape={obj['shape']} | "
            f"color={obj['color']} | "
            f"bin={obj['bin']} | "
            f"pixel=({round(obj['cx'], 1)}, {round(obj['cy'], 1)}) | "
            f"area={round(obj['area'], 1)} | "
            f"pos3d={pos_text}"
        )

def debug_print_detected_objects():
    objects = detect_objects_shape_color()

    print(f"\n[VISION DEBUG] detected_objects = {len(objects)}")

    for i, obj in enumerate(objects):
        rgb_text = None
        if obj["rgb"] is not None:
            rgb_text = [round(float(v), 3) for v in obj["rgb"]]

        pos_text = [round(float(v), 3) for v in obj["position_3d"]]

        print(
            f"[{i}] "
            f"id={obj['id']} | "
            f"model={obj['model']} | "
            f"shape={obj['shape']} | "
            f"color={obj['color']} | "
            f"rgb={rgb_text} | "
            f"pixel=({round(obj['cx'], 1)}, {round(obj['cy'], 1)}) | "
            f"area={round(obj['area'], 1)} | "
            f"pos3d={pos_text}"
        )

def get_cube_in_base_frame(pos_3d_camera, T_base_flange):
    """
    Convert cube position from Camera frame -> Flange frame -> Base frame.

    pos_3d_camera: [x, y, z] from Webots Camera Recognition.
    T_base_flange: FK matrix, Base <- Flange.
    """
    p_camera = np.array([pos_3d_camera[0], pos_3d_camera[1], pos_3d_camera[2], 1.0])
    p_base = T_base_flange @ T_FLANGE_CAMERA @ p_camera
    return p_base[0:3]


def get_cube_candidates_base(T_base_flange, target_model_name="cube", min_area=0):
    """Return every visible cube with camera and base-frame coordinates."""
    detections = detect_cubes(target_model_name=target_model_name, min_area=min_area)
    candidates = []

    for det in detections:
        base_pos = get_cube_in_base_frame(det["position_3d"], T_base_flange)
        if not np.all(np.isfinite(base_pos)):
            continue
        item = dict(det)
        item["position_base"] = np.array(base_pos, dtype=float)
        candidates.append(item)

    return candidates


def choose_cube_candidate(candidates, last_base=None, workspace=None):
    """
    Select one cube deterministically.

    Priority:
    1. locked Webots recognition id;
    2. nearest to last tracked base position;
    3. best score for the pick lane/workspace.
    """
    if not candidates:
        return None

    if _locked_cube_id is not None:
        for c in candidates:
            if c["id"] == _locked_cube_id:
                return c

    if last_base is not None:
        last_base = np.array(last_base, dtype=float)
        nearest = min(candidates, key=lambda c: np.linalg.norm(c["position_base"] - last_base))
        # Avoid jumping to another cube if the previous target is momentarily
        # lost. 25 cm is much larger than per-step conveyor motion but smaller
        # than normal spacing between cubes.
        if np.linalg.norm(nearest["position_base"] - last_base) < 0.25:
            return nearest

    if workspace is None:
        return max(candidates, key=lambda c: c["area"])

    x_goal = workspace.get("x_goal", -0.75)
    x_min  = workspace.get("x_min", -1.25)
    x_max  = workspace.get("x_max", -0.25)
    y_lim  = workspace.get("y_limit", 0.60)
    r_max  = workspace.get("r_max", 0.95)

    def score(c):
        x, y, _ = c["position_base"]
        r = math.sqrt(x * x + y * y)

        # Prefer cubes near the planned pick zone and lane center.
        s = abs(x - x_goal) + 0.8 * abs(y)

        # Penalize, do not immediately reject. This prevents the controller
        # from discarding a cube just because it is still near the camera edge.
        if x < x_min:
            s += 2.0 + 3.0 * (x_min - x)
        if x > x_max:
            s += 1.0 + 2.0 * (x - x_max)
        if abs(y) > y_lim:
            s += 2.0 + 3.0 * (abs(y) - y_lim)
        if r > r_max:
            s += 2.0 + 3.0 * (r - r_max)
        return s

    return min(candidates, key=score)


def get_selected_cube_base_position(
    T_base_flange,
    last_base=None,
    target_model_name="cube",
    min_area=0,
    workspace=None,
    return_candidate=False,
):
    candidates = get_cube_candidates_base(
        T_base_flange,
        target_model_name=target_model_name,
        min_area=min_area,
    )
    selected = choose_cube_candidate(candidates, last_base=last_base, workspace=workspace)
    if selected is None:
        return (None, None) if return_candidate else None

    if selected["id"] is not None:
        set_target_lock(selected["id"])

    if return_candidate:
        return selected["position_base"], selected
    return selected["position_base"]


def get_cube_base_position(T_base_flange):
    """Backward-compatible single-cube helper."""
    cube_data = detect_cube()
    if cube_data is None:
        return None

    return get_cube_in_base_frame(cube_data["position_3d"], T_base_flange)


def get_cube_world_position(T_base_flange, T_world_base):
    cube_base = get_cube_base_position(T_base_flange)
    if cube_base is None:
        return None

    p_base = np.array([cube_base[0], cube_base[1], cube_base[2], 1.0])
    p_world = T_world_base @ p_base
    return p_world[0:3]


def debug_check_camera_transform(pos_3d_camera, T_base_flange, T_world_base, supervisor_cube_node):
    cube_in_base = get_cube_in_base_frame(pos_3d_camera, T_base_flange)

    T_base_cube = np.identity(4)
    T_base_cube[0:3, 3] = cube_in_base
    T_world_cube = T_world_base @ T_base_cube
    calc_cube_world = T_world_cube[0:3, 3]

    actual_cube_world = supervisor_cube_node.getPosition()

    print("\n=== [CHECK STEP 2] CAMERA TO BASE MATRIX VERIFICATION ===")
    print(f"Toán ma trận tính ra (World): {[round(p, 4) for p in calc_cube_world]}")
    print(f"Supervisor Webots đo (World): {[round(a, 4) for a in actual_cube_world]}")

    error = np.linalg.norm(np.array(actual_cube_world) - calc_cube_world)
    print(f"--> Sai số tuyệt đối phép dịch ma trận camera: {round(error, 6)} mét")

    if error < 0.015:
        print("[STATUS]: BƯỚC 2 ĐẠT CHUẨN! Hệ tọa độ thị giác đã đồng bộ hoàn toàn.")
        return True
    else:
        print("[WARNING]: Sai lệch hệ trục camera! Kiểm tra lại định hướng camera hoặc offset.")
        return False


# ------------------------------------------------------------------
# Legacy tracking / waiting helpers kept for compatibility
# ------------------------------------------------------------------

def wait_until_cube_ready(move_ur_to_fn, get_joints_fn, max_steps=500):
    print("Tracking and following the cube along the conveyor...")

    image_center_x = _camera.getWidth()  / 2
    image_center_y = _camera.getHeight() / 2

    PICK_ZONE_X  = 15
    PICK_ZONE_Y  = 250
    MIN_CUBE_AREA = 150
    Kp = -0.0015

    for i in range(max_steps):
        result = detect_cube(min_area=MIN_CUBE_AREA)

        if result is not None:
            cx, cy, area = result["cx"], result["cy"], result["area"]
            error_x = cx - image_center_x
            error_y = abs(cy - image_center_y)

            current_q = get_joints_fn()
            current_q[0] += Kp * error_x
            move_ur_to_fn(current_q)

            if i % 10 == 0:
                model = result["model"] if result["model"] != "" else "Unknown_Cube"
                pos_3d = [round(val, 3) for val in list(result["position_3d"])]
                print(f"Tracking -> id={result['id']}, model={model}, pos3d={pos_3d}, "
                      f"cx={round(cx,1)}, cy={round(cy,1)}, error_y={round(error_y,1)}, "
                      f"base={round(current_q[0],3)}")

            if abs(error_x) < PICK_ZONE_X and error_y < PICK_ZONE_Y and area > MIN_CUBE_AREA:
                if result["id"] is not None:
                    set_target_lock(result["id"])
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
            error_x = result["cx"] - image_center_x
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
    After lifting, verify that at least one cube is very close to the camera.
    This avoids a false miss when several other cubes remain visible on the belt.
    """
    wait_steps_fn(5)

    detections = detect_cubes(target_model_name="cube")
    if not detections:
        print("-> [CONFIRM]: Object NOT seen in gripper. Pick missed completely!")
        return False

    distances = []
    for det in detections:
        p = det["position_3d"]
        distances.append(math.sqrt(p[0]**2 + p[1]**2 + p[2]**2))

    min_distance = min(distances)
    print(f"Closest distance from camera to object after lift: {round(min_distance, 3)} m")

    if min_distance < 0.25:
        print("-> [CONFIRM]: Pick SUCCESSFUL! Carrying to bin.")
        return True

    print("-> [CONFIRM]: Pick MISSED! Object still on conveyor.")
    return False


def preview_camera(n_steps=200):
    print("Preview camera before picking...")
    for _ in range(n_steps):
        if _robot.step(_time_step) == -1:
            return False
    return True


def count_detected_cubes():
    total_cubes = len(detect_cubes(target_model_name=None))
    print(f"[VISION]: Phát hiện {total_cubes} vật thể trong khung hình!")
    return total_cubes


def count_cubes_by_model(target_model_name="cube"):
    count = len(detect_cubes(target_model_name=target_model_name))
    print(f"[FILTER]: Tìm thấy {count} vật thể thuộc nhóm '{target_model_name}'")
    return count
