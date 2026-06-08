from controller import Supervisor

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

# Nếu cần debug trực tiếp TCP/flange bằng Supervisor
end_effector_node = robot.getFromDef("tool_slot")
if end_effector_node is None:
    end_effector_node = robot.getSelf().getFromProtoDef("wrist_3_link")

cube_node = robot.getFromDef("cube2")

# Trục của flange/gripper cần ép vuông góc mặt conveyor.
# Nếu chạy thử thấy gripper bị lật ngược hoặc vẫn nghiêng sai chiều,
# đổi lần lượt: "-z", "x", "-x", "y", "-y" tùy cách bạn gắn gripper vào flange.
GRIPPER_DOWN_AXIS = "z"


def is_valid_vector(v, size=None):
    if v is None:
        return False

    arr = np.array(v, dtype=float)

    if size is not None and arr.size != size:
        return False

    return np.all(np.isfinite(arr))


def make_pick_targets_from_cube_base(cube_base):
    """
    Tạo target cho FLANGE trong hệ Base.
    cube_base là tọa độ cube trong hệ Base robot.

    Lưu ý:
    - IK điều khiển flange, không điều khiển đầu gripper.
    - Vì gripper dài xuống dưới flange, phải cộng thêm FLANGE_TO_GRIPPER_TIP.
    """

    # Cần chỉnh theo gripper thật của bạn
    CUBE_HALF_HEIGHT = 0.025          # nếu cube cao 5 cm
    FLANGE_TO_GRIPPER_TIP = 0.12       # khoảng cách flange -> đầu gripper, cần tune
    PICK_CLEARANCE = 0.01             # khoảng hở khi xuống gắp
    ABOVE_CLEARANCE = 0.16           # khoảng cách q_above so với q_down

    cube_top_z = cube_base[2] + CUBE_HALF_HEIGHT

    p_down_base = np.array([
        cube_base[0],
        cube_base[1],
        cube_top_z + FLANGE_TO_GRIPPER_TIP + PICK_CLEARANCE
    ])

    p_above_base = np.array([
        cube_base[0],
        cube_base[1],
        p_down_base[2] + ABOVE_CLEARANCE
    ])

    return p_above_base, p_down_base
# =========================
# DYNAMIC TRACKING CONFIG
# =========================

DT = TIME_STEP / 1000.0

# Tùy tốc độ băng tải và tốc độ robot mà chỉnh.
# Conveyor càng nhanh thì tăng LEAD_TIME.
LEAD_TIME_ABOVE = 0.25
LEAD_TIME_DOWN = 0.12
LEAD_TIME_CLOSE = 0.08

MAX_JOINT_STEP = 0.075      # rad/step, tránh q nhảy quá gắt
CUBE_ALPHA = 0.35           # lọc vị trí cube
VEL_ALPHA = 0.25            # lọc vận tốc cube
LOST_MAX_STEPS = 8

tracker = {
    "pos": None,
    "vel": np.zeros(3),
    "last_meas": None,
    "lost": 0,
}


def limit_joint_step(current_q, target_q, max_step=MAX_JOINT_STEP):
    current_q = np.array(current_q, dtype=float)
    target_q = np.array(target_q, dtype=float)

    dq = target_q - current_q
    dq = np.clip(dq, -max_step, max_step)

    return (current_q + dq).tolist()


def update_cube_tracker():
    """
    Cập nhật cube_base liên tục từ camera + FK.
    Nếu camera mất cube vài step thì dùng vận tốc ước lượng để dự đoán tiếp.
    """
    current_q = motion.get_current_joint_positions()

    if not is_valid_vector(current_q, size=6):
        return None, False

    T_base_flange = motion.forward_kinematics(current_q)
    meas = camera_utils.get_cube_base_position(T_base_flange)

    if is_valid_vector(meas, size=3):
        meas = np.array(meas, dtype=float)

        if tracker["pos"] is None:
            tracker["pos"] = meas
            tracker["vel"] = np.zeros(3)
        else:
            raw_vel = (meas - tracker["last_meas"]) / DT
            tracker["vel"] = (1.0 - VEL_ALPHA) * tracker["vel"] + VEL_ALPHA * raw_vel
            tracker["pos"] = (1.0 - CUBE_ALPHA) * tracker["pos"] + CUBE_ALPHA * meas

        tracker["last_meas"] = meas
        tracker["lost"] = 0
        return tracker["pos"], True

    # Nếu tạm mất cube, dự đoán theo vận tốc cũ
    if tracker["pos"] is not None and tracker["lost"] < LOST_MAX_STEPS:
        tracker["pos"] = tracker["pos"] + tracker["vel"] * DT
        tracker["lost"] += 1
        return tracker["pos"], False

    return None, False


PICK_X_MIN = -1.5     # nếu cube_base[0] nhỏ hơn giá trị này thì coi như đã quá xa
PICK_X_MAX = -0.40     # vùng bắt đầu pick tốt
PICK_Y_LIMIT = 0.40    # giới hạn lệch ngang, tùy scene của bạn


def dynamic_ik_track_step(target_mode, lead_time, reach_tolerance):
    cube_base, seen = update_cube_tracker()

    if cube_base is None:
        return False, None, None, seen, "no_cube"

    cube_pred = cube_base + tracker["vel"] * lead_time

    # Nếu cube đã trôi quá xa vùng làm việc thì bỏ cube này
    if cube_pred[0] < PICK_X_MIN:
        return False, None, None, seen, "too_late"

    if abs(cube_pred[1]) > PICK_Y_LIMIT:
        return False, None, None, seen, "out_y"

    p_above_base, p_down_base = make_pick_targets_from_cube_base(cube_pred)

    if target_mode == "above":
        target_base = p_above_base
    else:
        target_base = p_down_base

    current_q = motion.get_current_joint_positions()

    if not is_valid_vector(current_q, size=6):
        return False, target_base, None, seen, "bad_q"

    # Tính lỗi hiện tại trước khi gọi IK
    flange_pos = motion.fk_position(current_q)
    pos_error = np.linalg.norm(np.array(target_base) - np.array(flange_pos))

    # Với mode down, vẫn tiếp tục gọi IK để sửa hướng gripper,
    # không được chỉ dựa vào lỗi vị trí.
    if pos_error < reach_tolerance:
        return True, target_base, pos_error, seen, "reached"

    q_target = motion.inverse_kinematics_downward(
        target_base,
        seed_q=current_q,
        flange_axis=GRIPPER_DOWN_AXIS,
        max_iters=120,
        pos_tolerance=0.006,
        ori_tolerance=0.01,
        damping=0.08,
        max_step=0.08,
        stay_near_seed=0.002,
    )

    if not is_valid_vector(q_target, size=6):
        # Không trả err=None nữa. Vẫn trả pos_error thật để debug.
        return False, target_base, pos_error, seen, "ik_fail"

    q_cmd = limit_joint_step(current_q, q_target)
    motion.move_ur_to(q_cmd)

    return False, target_base, pos_error, seen, "tracking"


def dynamic_track_to_cube(
    target_mode,
    max_steps,
    lead_time,
    reach_tolerance,
    stable_required=4,
    stop_when_reached=True
):
    stable_count = 0

    for i in range(max_steps):
        reached, target_base, pos_error, seen, status = dynamic_ik_track_step(
            target_mode=target_mode,
            lead_time=lead_time,
            reach_tolerance=reach_tolerance
        )
    
        if status == "too_late":
            print("[DYNAMIC] Cube đã chạy quá xa vùng pick. Bỏ cube này.")
            return False

        if status == "out_y":
            print("[DYNAMIC] Cube lệch ngang quá nhiều. Bỏ cube này.")
            return False

        if target_base is None:
            if i % 20 == 0:
                print(f"[DYNAMIC] status={status}, camera chưa có target hợp lệ")
        else:
            if i % 10 == 0:
                print(
                    f"[DYNAMIC] mode={target_mode}, "
                    f"status={status}, "
                    f"seen={seen}, "
                    f"target={[round(float(v), 4) for v in target_base]}, "
                    f"err={round(float(pos_error), 4) if pos_error is not None else None}"
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
    """
    Sau khi đã kẹp cube, không cần bám conveyor nữa.
    Nâng flange thẳng lên theo hệ Base.
    """
    current_q = motion.get_current_joint_positions()
    current_pos = motion.fk_position(current_q)

    lift_target = np.array([
        current_pos[0],
        current_pos[1],
        current_pos[2] + lift_height
    ])

    q_lift = motion.inverse_kinematics_downward(
        lift_target,
        seed_q=current_q,
        flange_axis=GRIPPER_DOWN_AXIS,
    )

    if not is_valid_vector(q_lift, size=6):
        print("[DYNAMIC] Không tìm được q_lift")
        return False

    return motion.goto_pose(q_lift, steps=80, tolerance=0.04)


def dynamic_pick_cube():
    """
    Pick động:
    1. bám cube ở vị trí above
    2. hạ xuống nhưng vẫn tính lại IK liên tục
    3. đóng gripper trong khi vẫn bám cube vài step
    4. nâng lên
    """

    print("\n=== DYNAMIC PICK START ===")

    # Reset tracker cho mỗi cube mới
    tracker["pos"] = None
    tracker["vel"] = np.zeros(3)
    tracker["last_meas"] = None
    tracker["lost"] = 0

    ok = dynamic_track_to_cube(
        target_mode="above",
        max_steps=250,
        lead_time=LEAD_TIME_ABOVE,
        reach_tolerance=0.035,
        stable_required=5,
        stop_when_reached=True
    )

    if not ok:
        print("[DYNAMIC] Không bám được q_above động")
        return False

    print("[DYNAMIC] Đã bám tới vùng above.")

    ok = dynamic_track_to_cube(
        target_mode="down",
        max_steps=160,
        lead_time=LEAD_TIME_DOWN,
        reach_tolerance=0.022,
        stable_required=3,
        stop_when_reached=True
    )

    if not ok:
        print("[DYNAMIC] Không xuống được vùng pick động")
        return False

    print("[DYNAMIC] Đã tới vùng down. Đóng gripper...")

    gripper.close_gripper()

    # Trong lúc gripper đang đóng, vẫn tiếp tục bám cube một đoạn ngắn
    dynamic_track_to_cube(
        target_mode="down",
        max_steps=35,
        lead_time=LEAD_TIME_CLOSE,
        reach_tolerance=999.0,
        stable_required=2,
        stop_when_reached=False
    )

    motion.wait_steps(10)

    ok = dynamic_lift_after_pick(lift_height=0.18)

    if not ok:
        print("[DYNAMIC] Lift failed")
        return False

    print("[DYNAMIC] Pick động xong và đã nâng cube.")

    # 7. Check whether pick succeeded
    if camera_utils.check_if_picked_successfully(wait_steps_fn=motion.wait_steps):
        # Success path: carry to bin and drop
        motion.goto_pose(poses.SAFE_MID,   steps=20)
        motion.goto_pose(poses.BIN_ABOVE,  steps=30)
        motion.goto_pose(poses.BIN_DOWN,   steps=20)

        gripper.open_gripper()

        motion.goto_pose(poses.BIN_ABOVE,  steps=20)
        motion.goto_pose(poses.SAFE_MID,   steps=20)
    else:
        # Miss path: skip bin run, go straight back home
        print("Pick missed. Returning HOME for next target...")

    # Back to home for next cycle
    gripper.open_gripper()
    motion.goto_pose(poses.PICK_ABOVE, steps=100)

    return True


# =========================
# STARTUP / WARM-UP
# =========================

# Cho sensor/motor/camera cập nhật vài bước đầu để tránh NaN
for _ in range(10):
    if robot.step(TIME_STEP) == -1:
        quit()

# Command HOME bằng lệnh trực tiếp trước, không nội suy
motion.move_ur_to(poses.HOME)
motion.wait_steps(50)

# Mở gripper
gripper.open_gripper()
motion.wait_steps(20)

# Đưa robot về vùng camera nhìn thấy băng chuyền/cube
motion.goto_pose(poses.PICK_ABOVE, steps=80)
motion.wait_steps(20)

print("=== IK TEST START ===")

step_count = 0


while robot.step(TIME_STEP) != -1:
    ok = dynamic_pick_cube()

    print("[DYNAMIC] Thử lại cube tiếp theo...")
"""
# =========================
# IK TEST LOOP
# =========================

while robot.step(TIME_STEP) != -1:

    current_q = motion.get_current_joint_positions()

    if not is_valid_vector(current_q, size=6):
        print(f"[IK TEST] current_q invalid: {current_q}")
        continue

    # FK hiện tại: Base <- Flange
    T_base_flange = motion.forward_kinematics(current_q)

    # Transform cố định: World <- Base
    T_world_base = motion.get_world_base_transform()

    cube_base = camera_utils.get_cube_base_position(T_base_flange)

    if cube_base is None:
        if step_count % 20 == 0:
            print("[IK TEST] Camera chưa thấy cube...")
        step_count += 1
        continue

    if not is_valid_vector(cube_base, size=3):
        print(f"[IK TEST] cube_base invalid: {cube_base}")
        continue

    cube_world = motion.base_point_to_world(cube_base)

    print(f"\n[IK TEST] Cube World: {[round(float(v), 4) for v in cube_world]}")
    print(f"[IK TEST] Cube Base : {[round(float(v), 4) for v in cube_base]}")

    p_above_base, p_down_base = make_pick_targets_from_cube_base(cube_base)

    print(f"[IK TEST] Target above base: {[round(float(v), 4) for v in p_above_base]}")
    print(f"[IK TEST] Target down base : {[round(float(v), 4) for v in p_down_base]}")

    if not is_valid_vector(p_above_base, size=3):
        print(f"[IK TEST] p_above_base invalid: {p_above_base}")
        continue

    if not is_valid_vector(p_down_base, size=3):
        print(f"[IK TEST] p_down_base invalid: {p_down_base}")
        continue

    print(f"[IK TEST] Target above base: {[round(float(v), 4) for v in p_above_base]}")
    print(f"[IK TEST] Target down base : {[round(float(v), 4) for v in p_down_base]}")

    # 3. IK có ràng buộc hướng: gripper/flange phải vuông góc mặt conveyor
    # Lỗi trong bản bạn gửi: dùng target_above_base / target_down_base / PICK_ABOVE
    # nhưng các biến đúng trong file này là p_above_base / p_down_base / poses.PICK_ABOVE.
    q_above = motion.inverse_kinematics_downward(
        p_above_base,
        seed_q=poses.PICK_ABOVE,
        flange_axis=GRIPPER_DOWN_AXIS,
    )

    if not is_valid_vector(q_above, size=6):
        print(f"[IK TEST] Không tìm được q_above hoặc q_above invalid: {q_above}")
        continue

    q_down = motion.inverse_kinematics_downward(
        p_down_base,
        seed_q=q_above,
        flange_axis=GRIPPER_DOWN_AXIS,
    )

    if not is_valid_vector(q_down, size=6):
        print(f"[IK TEST] Không tìm được q_down hoặc q_down invalid: {q_down}")
        continue

    print(f"[IK TEST] q_above: {[round(float(v), 4) for v in q_above]}")
    print(f"[IK TEST] q_down : {[round(float(v), 4) for v in q_down]}")

    # 4. Debug hướng gripper sau IK
    if hasattr(motion, "debug_check_gripper_down"):
        motion.debug_check_gripper_down(q_above, flange_axis=GRIPPER_DOWN_AXIS)
        motion.debug_check_gripper_down(q_down, flange_axis=GRIPPER_DOWN_AXIS)

    # 5. Đi tới vị trí phía trên cube
    ok = motion.goto_pose(q_above, steps=80, tolerance=0.04)
    if not ok:
        print("[IK TEST] goto_pose(q_above) failed")
        continue

    print("[IK TEST] Đã tới q_above.")
    motion.wait_steps(20)

    # 6. Hạ xuống vị trí gắp
    ok = motion.goto_pose(q_down, steps=60)
    if not ok:
        print("[IK TEST] goto_pose(q_down) failed")
        continue
    print("[IK TEST] Đã tới q_down.")
    motion.wait_steps(20)

    gripper.close_gripper()
    motion.wait_steps(80)

    motion.goto_pose(q_above, steps=80)

    print("[IK TEST] Đã gắp thử và nâng lên.")
    break
"""
