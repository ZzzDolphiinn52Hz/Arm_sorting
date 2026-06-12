from controller import Supervisor

import math
import numpy as np

import poses
import gripper
import motion
import camera_utils

# =========================
# INIT
# =========================

TIME_STEP = 32
robot = Supervisor()

motion.init(robot, TIME_STEP)
gripper.init(robot, TIME_STEP)
camera_utils.init(robot, TIME_STEP, camera_name="camera")

end_effector_node = robot.getFromDef("tool_slot")
if end_effector_node is None:
    end_effector_node = robot.getSelf().getFromProtoDef("wrist_3_link")

# Trục của flange/gripper cần ép vuông góc mặt conveyor.
# Bản motion_optimized hỗ trợ đủ: "z", "-z", "x", "-x", "y", "-y".
# Nếu gripper vẫn nghiêng, đổi biến này trước, không cần sửa IK.
GRIPPER_DOWN_AXIS = "z"
# =========================
# SORTING BIN MAP
# =========================

BIN_POSES = {
    "BLUE_CUBE_BIN": (
        poses.BLUE_CUBE_BIN_ABOVE,
        poses.BLUE_CUBE_BIN_DOWN,
    ),

    "RED_CUBE_BIN": (
        poses.RED_CUBE_BIN_ABOVE,
        poses.RED_CUBE_BIN_DOWN,
    ),

    "YELLOW_CYLINDER_BIN": (
        poses.YELLOW_CYLINDER_BIN_ABOVE,
        poses.YELLOW_CYLINDER_BIN_DOWN,
    ),

    "GREEN_SPHERE_BIN": (
        poses.GREEN_SPHERE_BIN_ABOVE,
        poses.GREEN_SPHERE_BIN_DOWN,
    ),

    # fallback nếu phân loại lỗi
    "UNKNOWN_BIN": (
        poses.BIN_ABOVE,
        poses.BIN_DOWN,
    ),
}

# =========================
# PICK GEOMETRY CONFIG
# =========================

CUBE_HALF_HEIGHT = 0.025
PICK_CLEARANCE = 0.02
ABOVE_CLEARANCE = 0.16

# TCP/tip offset expressed in FLANGE local frame, not base frame.
# Đây là chỗ quan trọng để sửa lỗi "flange đúng nhưng gripper lệch".
# Nếu đầu kẹp thật không nằm ngay dưới tâm flange, chỉnh X/Y ở đây.
# Ví dụ bị lệch vào trong conveyor 2 cm thì thử [0.02, 0.0, 0.120]
# hoặc [-0.02, 0.0, 0.120] tùy chiều thực tế.
GRIPPER_TIP_OFFSET_FLANGE = np.array([0.000, 0.000, 0.120])


# =========================
# DYNAMIC TRACKING CONFIG
# =========================

DT = TIME_STEP / 1000.0

LEAD_TIME_ABOVE = 0.28
LEAD_TIME_DOWN = 0.14
LEAD_TIME_CLOSE = 0.08

MAX_JOINT_STEP = 0.070
CUBE_ALPHA = 0.42
VEL_ALPHA = 0.20
LOST_MAX_STEPS = 12
MAX_CUBE_SPEED = 0.70      # m/s, clamp noise when detector switches object
MIN_CUBE_AREA = 80

# Use a broad camera/pick lane first, then let IK decide reachability.
# These numbers are intentionally less aggressive than the old hard skip.
PICK_X_MIN = -1.35
PICK_X_MAX = -0.25
PICK_X_GOAL = -0.72
PICK_Y_LIMIT = 0.80
PICK_RADIUS_MAX = 0.98
TOO_LATE_GRACE_STEPS = 18
IK_FAIL_GRACE_STEPS = 35

workspace = {
    "x_min": PICK_X_MIN,
    "x_max": PICK_X_MAX,
    "x_goal": PICK_X_GOAL,
    "y_limit": PICK_Y_LIMIT,
    "r_max": PICK_RADIUS_MAX,
}

tracker = {
    "pos": None,
    "vel": np.zeros(3),
    "last_meas": None,
    "lost": 0,

    "id": None,
    "model": None,
    "shape": None,
    "color": None,
    "bin": None,

    "too_late_count": 0,
    "ik_fail_count": 0,
}

def reset_object_tracker():
    tracker["pos"] = None
    tracker["vel"] = np.zeros(3)
    tracker["last_meas"] = None
    tracker["lost"] = 0

    tracker["id"] = None
    tracker["model"] = None
    tracker["shape"] = None
    tracker["color"] = None
    tracker["bin"] = None

    tracker["too_late_count"] = 0
    tracker["ik_fail_count"] = 0

    if hasattr(camera_utils, "reset_target_lock"):
        camera_utils.reset_target_lock()

# =========================
# UTILS
# =========================

def is_valid_vector(v, size=None):
    if v is None:
        return False
    arr = np.array(v, dtype=float)
    if size is not None and arr.size != size:
        return False
    return np.all(np.isfinite(arr))


def reset_object_tracker():
    tracker["id"] = None
    tracker["pos"] = None
    tracker["vel"] = np.zeros(3)
    tracker["last_meas"] = None
    tracker["lost"] = 0
    tracker["too_late_count"] = 0
    tracker["ik_fail_count"] = 0
    if hasattr(camera_utils, "reset_target_lock"):
        camera_utils.reset_target_lock()


def limit_joint_step(current_q, target_q, max_step=MAX_JOINT_STEP):
    current_q = np.array(current_q, dtype=float)
    target_q = np.array(target_q, dtype=float)
    dq = target_q - current_q
    dq = np.clip(dq, -max_step, max_step)
    return (current_q + dq).tolist()

PICK_X_BEST = -0.38
LOCK_DISTANCE = 0.12

TEST_ONLY_BIN = None

def select_target_object(objects_base):
    """
    Chọn 1 object tốt nhất để gắp.
    Bước này chỉ chọn object đã phân loại được bin.
    """
    candidates = []

    for obj in objects_base:
        obj_bin = obj.get("bin")

        if obj_bin in [None, "UNKNOWN_BIN"]:
            continue

        # Khi đang test riêng một loại, bỏ qua object thuộc bin khác
        if TEST_ONLY_BIN is not None and obj_bin != TEST_ONLY_BIN:
            continue

        p = obj.get("base_position", None)

        if not is_valid_vector(p, size=3):
            continue

        p = np.array(p, dtype=float)

        # Lọc ngang nhẹ, tránh chọn vật quá lệch khỏi vùng tay máy.
        if abs(p[1]) > PICK_Y_LIMIT:
            continue

        score = (
            abs(p[0] - PICK_X_BEST) * 2.0
            + abs(p[1]) * 1.0
        )

        candidates.append((score, obj))

    if not candidates:
        return None

    candidates.sort(key=lambda x: x[0])
    return candidates[0][1]


def find_locked_object(objects_base):
    """
    Nếu tracker đã khóa object thì cố bám đúng object đó.
    Ưu tiên id, nếu không tìm được id thì bám object gần vị trí tracker nhất.
    """
    if not objects_base:
        return None

    # Ưu tiên 1: bám theo recognition id
    if tracker["id"] is not None:
        for obj in objects_base:
            if obj.get("id") == tracker["id"]:
                return obj

    # Ưu tiên 2: nếu mất id, bám object gần vị trí cũ nhất
    if tracker["pos"] is not None:
        best_obj = None
        best_dist = 999.0

        for obj in objects_base:
            p = obj.get("base_position", None)

            if not is_valid_vector(p, size=3):
                continue

            dist = np.linalg.norm(np.array(p, dtype=float) - tracker["pos"])

            if dist < best_dist:
                best_dist = dist
                best_obj = obj

        if best_obj is not None and best_dist < LOCK_DISTANCE:
            return best_obj

    # Nếu chưa khóa gì thì chọn target mới
    return select_target_object(objects_base)

def clamp_velocity(v):
    v = np.array(v, dtype=float)
    speed = np.linalg.norm(v)
    if speed > MAX_CUBE_SPEED:
        v = v / speed * MAX_CUBE_SPEED
    return v


def make_pick_targets_from_cube_base(cube_base, seed_q=None):
    """
    Create FLANGE targets in Base frame from cube center in Base frame.

    Correct formula:
        p_tip_base = cube top + clearance
        p_flange_base = p_tip_base - R_base_flange @ tip_offset_flange

    This prevents side error when the gripper/TCP is not exactly at the flange
    origin or when the flange is not perfectly vertical yet.
    """
    if seed_q is None:
        seed_q = motion.get_current_joint_positions()

    target_R = motion.make_gripper_down_rotation(
        seed_q=seed_q,
        flange_axis=GRIPPER_DOWN_AXIS,
    )

    cube_base = np.array(cube_base, dtype=float)
    p_tip_down_base = np.array([
        cube_base[0],
        cube_base[1],
        cube_base[2] + CUBE_HALF_HEIGHT + PICK_CLEARANCE,
    ])

    p_down_base = p_tip_down_base - target_R @ GRIPPER_TIP_OFFSET_FLANGE
    p_above_base = p_down_base + np.array([0.0, 0.0, ABOVE_CLEARANCE])

    return p_above_base, p_down_base, target_R


def is_inside_broad_workspace(p):
    x, y, _ = p
    r = math.sqrt(x * x + y * y)
    if abs(y) > PICK_Y_LIMIT:
        return False, "out_y"
    if r > PICK_RADIUS_MAX:
        return False, "out_radius"
    if x > PICK_X_MAX:
        return False, "too_early"
    if x < PICK_X_MIN:
        return False, "too_late"
    return True, "inside"


# =========================
# VISION TRACKER
# =========================

def update_cube_tracker():
    """
    Update one locked cube in base frame.
    Multiple visible cubes are handled by choosing a target once, then keeping
    its recognition id or nearest-neighbor track.
    """
    current_q = motion.get_current_joint_positions()
    if not is_valid_vector(current_q, size=6):
        return None, False, None

    T_base_flange = motion.forward_kinematics(current_q)

    meas, candidate = camera_utils.get_selected_cube_base_position(
        T_base_flange,
        last_base=tracker["pos"],
        target_model_name="cube",
        min_area=MIN_CUBE_AREA,
        workspace=workspace,
        return_candidate=True,
    )

    if is_valid_vector(meas, size=3):
        meas = np.array(meas, dtype=float)
        candidate_id = candidate.get("id") if candidate is not None else None

        if tracker["id"] is None and candidate_id is not None:
            tracker["id"] = candidate_id

        if tracker["pos"] is None or tracker["last_meas"] is None:
            tracker["pos"] = meas
            tracker["vel"] = np.zeros(3)
        else:
            jump = np.linalg.norm(meas - tracker["last_meas"])
            if jump > 0.25:
                # Detector probably switched to another cube. Reset velocity so
                # prediction does not instantly mark it as too_late.
                tracker["pos"] = meas
                tracker["vel"] = np.zeros(3)
            else:
                raw_vel = clamp_velocity((meas - tracker["last_meas"]) / DT)
                tracker["vel"] = (1.0 - VEL_ALPHA) * tracker["vel"] + VEL_ALPHA * raw_vel
                tracker["pos"] = (1.0 - CUBE_ALPHA) * tracker["pos"] + CUBE_ALPHA * meas

        tracker["last_meas"] = meas
        tracker["lost"] = 0
        return tracker["pos"], True, candidate

    if tracker["pos"] is not None and tracker["lost"] < LOST_MAX_STEPS:
        tracker["pos"] = tracker["pos"] + tracker["vel"] * DT
        tracker["lost"] += 1
        return tracker["pos"], False, None

    return None, False, None

def update_object_tracker():
    """
    Tracker mới:
    - Lấy nhiều object từ camera
    - Chọn 1 object
    - Lưu shape/color/bin vào tracker
    - Trả về vị trí base để IK dùng như trước
    """
    current_q = motion.get_current_joint_positions()

    if not is_valid_vector(current_q, size=6):
        return None, False

    T_base_flange = motion.forward_kinematics(current_q)
    objects_base = camera_utils.get_objects_base_with_bin(T_base_flange)

    obj = find_locked_object(objects_base)

    if obj is not None:
        meas = obj.get("base_position", None)

        if is_valid_vector(meas, size=3):
            meas = np.array(meas, dtype=float)

            if tracker["pos"] is None or tracker["last_meas"] is None:
                tracker["pos"] = meas
                tracker["vel"] = np.zeros(3)
            else:
                raw_vel = (meas - tracker["last_meas"]) / DT
                tracker["vel"] = (1.0 - VEL_ALPHA) * tracker["vel"] + VEL_ALPHA * raw_vel
                tracker["pos"] = (1.0 - CUBE_ALPHA) * tracker["pos"] + CUBE_ALPHA * meas

            tracker["last_meas"] = meas
            tracker["lost"] = 0

            # Lưu thông tin phân loại vào tracker
            was_new_target = tracker["id"] is None

            tracker["id"] = obj.get("id")
            tracker["model"] = obj.get("model")
            tracker["shape"] = obj.get("shape")
            tracker["color"] = obj.get("color")
            tracker["bin"] = obj.get("bin")

            if was_new_target:
                print(
                    f"[TARGET LOCKED] "
                    f"id={tracker['id']} | "
                    f"shape={tracker['shape']} | "
                    f"color={tracker['color']} | "
                    f"bin={tracker['bin']}"
                )

            return tracker["pos"], True

    # Nếu tạm mất object thì dự đoán tiếp
    if tracker["pos"] is not None and tracker["lost"] < LOST_MAX_STEPS:
        tracker["pos"] = tracker["pos"] + tracker["vel"] * DT
        tracker["lost"] += 1
        return tracker["pos"], False

    return None, False

# =========================
# DYNAMIC IK TRACKING
# =========================

def dynamic_ik_track_step(target_mode, lead_time, reach_tolerance, ori_tolerance_deg=3.0):
    object_base, seen = update_object_tracker()

    if object_base is None:
        return False, None, None, None, seen, "no_object"

    object_pred = object_base + tracker["vel"] * lead_time

    inside, workspace_status = is_inside_broad_workspace(object_pred)

    # Do not immediately fail when the cube is still early or slightly outside.
    # Keep the target locked so the robot waits for the same cube.
    if workspace_status == "too_late":
        tracker["too_late_count"] += 1
        if tracker["too_late_count"] >= TOO_LATE_GRACE_STEPS:
            return False, None, None, None, seen, "too_late"
        return False, None, None, None, seen, "waiting_too_late_confirm"

    tracker["too_late_count"] = 0

    if workspace_status in ("too_early", "out_y", "out_radius"):
        return False, None, None, None, seen, "waiting_" + workspace_status

    current_q = motion.get_current_joint_positions()
    if not is_valid_vector(current_q, size=6):
        return False, None, None, None, seen, "bad_q"

    p_above_base, p_down_base, target_R = make_pick_targets_from_cube_base(object_pred, seed_q=current_q)
    target_base = p_above_base if target_mode == "above" else p_down_base

    flange_pos = motion.fk_position(current_q)
    pos_error = float(np.linalg.norm(np.array(target_base) - np.array(flange_pos)))
    ori_error_deg = float(motion.gripper_down_angle_deg(current_q, flange_axis=GRIPPER_DOWN_AXIS))

    # Critical fix: do not stop only because position is close. The gripper must
    # also be vertical; otherwise it closes sideways at the conveyor edge.
    if pos_error < reach_tolerance and ori_error_deg < ori_tolerance_deg:
        tracker["ik_fail_count"] = 0
        return True, target_base, pos_error, ori_error_deg, seen, "reached"

    q_target = motion.inverse_kinematics_pose(
        target_pos_base=target_base,
        target_R_base_flange=target_R,
        seed_q=current_q,
        max_iters=180,
        pos_tolerance=0.005,
        ori_tolerance=np.radians(1.8),
        damping=0.055,
        max_step=0.060,
        position_weight=1.0,
        orientation_weight=1.8,
        stay_near_seed=0.0015,
        verbose=False,
    )

    if not is_valid_vector(q_target, size=6):
        tracker["ik_fail_count"] += 1
        if tracker["ik_fail_count"] >= IK_FAIL_GRACE_STEPS:
            return False, target_base, pos_error, ori_error_deg, seen, "ik_fail_final"
        return False, target_base, pos_error, ori_error_deg, seen, "ik_fail_wait"

    tracker["ik_fail_count"] = 0
    q_cmd = limit_joint_step(current_q, q_target)
    motion.move_ur_to(q_cmd)

    return False, target_base, pos_error, ori_error_deg, seen, "tracking"


def dynamic_track_to_cube(
    target_mode,
    max_steps,
    lead_time,
    reach_tolerance,
    stable_required=4,
    stop_when_reached=True,
    ori_tolerance_deg=3.0,
):
    stable_count = 0

    for i in range(max_steps):
        reached, target_base, pos_error, ori_error_deg, seen, status = dynamic_ik_track_step(
            target_mode=target_mode,
            lead_time=lead_time,
            reach_tolerance=reach_tolerance,
            ori_tolerance_deg=ori_tolerance_deg,
        )

        if status == "too_late":
            print("[DYNAMIC] Cube đã ra khỏi vùng gắp thật sự. Bỏ cube này.")
            return False

        if status == "ik_fail_final":
            print("[DYNAMIC] IK fail quá lâu với cube hiện tại. Bỏ cube này để tránh loạn.")
            return False

        if target_base is None:
            if i % 15 == 0:
                print(f"[DYNAMIC] mode={target_mode}, status={status}, seen={seen}, waiting same cube...")
        else:
            if i % 10 == 0:
                print(
                    f"[DYNAMIC] mode={target_mode}, status={status}, seen={seen}, "
                    f"target={[round(float(v), 4) for v in target_base]}, "
                    f"pos_err={round(float(pos_error), 4) if pos_error is not None else None}, "
                    f"ori_err={round(float(ori_error_deg), 2) if ori_error_deg is not None else None}deg"
                )

        if reached:
            stable_count += 1
        else:
            stable_count = 0

        if stop_when_reached and stable_count >= stable_required:
            return True

        if robot.step(TIME_STEP) == -1:
            return False

    return not stop_when_reached


def dynamic_lift_after_pick(lift_height=0.18):
    current_q = motion.get_current_joint_positions()
    current_pos = motion.fk_position(current_q)

    lift_target = np.array([current_pos[0], current_pos[1], current_pos[2] + lift_height])

    target_R = motion.make_gripper_down_rotation(seed_q=current_q, flange_axis=GRIPPER_DOWN_AXIS)
    q_lift = motion.inverse_kinematics_pose(
        target_pos_base=lift_target,
        target_R_base_flange=target_R,
        seed_q=current_q,
        max_iters=180,
        orientation_weight=1.5,
        verbose=False,
    )

    if not is_valid_vector(q_lift, size=6):
        print("[DYNAMIC] Không tìm được q_lift")
        return False

    return motion.goto_pose(q_lift, steps=30, tolerance=0.04)

def place_object_to_bin(bin_name):
    """
    Nhận tên bin, tự lấy pose above/down tương ứng rồi thả vật.
    """

    if bin_name not in BIN_POSES:
        print(f"[SORT WARNING] Không có bin '{bin_name}', dùng UNKNOWN_BIN")
        bin_name = "UNKNOWN_BIN"

    bin_above, bin_down = BIN_POSES[bin_name]

    print(f"[SORT] Đưa object tới {bin_name}")

    motion.goto_pose(poses.SAFE_MID, steps=5)

    motion.goto_pose(bin_above, steps=10)
    motion.goto_pose(bin_down, steps=5)

    gripper.open_gripper()
    # motion.wait_steps(6)

    motion.goto_pose(bin_above, steps=10)
    motion.goto_pose(poses.SAFE_MID, steps=5)
    motion.goto_pose(poses.PICK_ABOVE, steps=5)

    return True

def dynamic_pick_cube():
    print("\n=== DYNAMIC PICK START ===")
    # reset_object_tracker()
    reset_object_tracker()

    ok = dynamic_track_to_cube(
        target_mode="above",
        max_steps=300,
        lead_time=LEAD_TIME_ABOVE,
        reach_tolerance=0.019,
        stable_required=3,
        stop_when_reached=True,
        ori_tolerance_deg=4.0,
    )

    if not ok:
        print("[DYNAMIC] Không bám được q_above động")
        reset_object_tracker()
        return False

    print("[DYNAMIC] Đã bám tới vùng above.")

    ok = dynamic_track_to_cube(
        target_mode="down",
        max_steps=190,
        lead_time=LEAD_TIME_DOWN,
        reach_tolerance=0.019,
        stable_required=2,
        stop_when_reached=True,
        ori_tolerance_deg=2.5,
    )

    if not ok:
        print("[DYNAMIC] Không xuống được vùng pick động")
        reset_object_tracker()
        return False

    picked_bin = tracker["bin"] if tracker["bin"] is not None else "UNKNOWN_BIN"

    print(
        f"[DYNAMIC] Đã tới vùng down. Đóng gripper..."
        f" Picked target: shape={tracker['shape']}, "
        f"color={tracker['color']}, "
        f"bin={picked_bin}"
    )

    gripper.close_gripper()

    dynamic_track_to_cube(
        target_mode="down",
        max_steps=35,
        lead_time=LEAD_TIME_CLOSE,
        reach_tolerance=999.0,
        stable_required=2,
        stop_when_reached=False,
        ori_tolerance_deg=4.0,
    )

    # motion.wait_steps(10)

    ok = dynamic_lift_after_pick(lift_height=0.18)
    if not ok:
        print("[DYNAMIC] Lift failed")
        reset_object_tracker()
        return False

    print("[DYNAMIC] Pick động xong và đã nâng cube.")

    # Tạm thời bỏ check để test sorting theo bin trước
    print(f"[SORT TEST] Picked bin = {picked_bin}")
    place_object_to_bin(picked_bin)

    """
    if camera_utils.check_if_picked_successfully(wait_steps_fn=motion.wait_steps):
        print(f"[SORT] Picked bin = {picked_bin}")
        place_object_to_bin(picked_bin)
    else:
        print("Pick missed. Returning HOME for next target...")

    gripper.open_gripper()
    reset_object_tracker()
    motion.goto_pose(poses.PICK_ABOVE, steps=50)
    return True
    """


# =========================
# STARTUP / WARM-UP
# =========================

for _ in range(10):
    if robot.step(TIME_STEP) == -1:
        quit()

motion.move_ur_to(poses.HOME)
#motion.wait_steps(50)

gripper.open_gripper()
#motion.wait_steps(20)

motion.goto_pose(poses.PICK_ABOVE, steps=80)
#motion.wait_steps(20)

print("=== DYNAMIC SORT PICK LOOP START ===")

while robot.step(TIME_STEP) != -1:
    ok = dynamic_pick_cube()

    if ok:
        print("[SORT] Pick cycle finished. Thử object tiếp theo...")
    else:
        print("[SORT] Pick failed. Thử object tiếp theo...")