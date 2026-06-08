from controller import Supervisor

import numpy as np
import poses
import gripper
import motion
import camera_utils

TIME_STEP = 32
robot = Supervisor()

motion.init(robot, TIME_STEP)
gripper.init(robot, TIME_STEP)
camera_utils.init(robot, TIME_STEP, camera_name="camera")

end_effector_node = robot.getFromDef("tool_slot")
if end_effector_node is None:
    end_effector_node = robot.getSelf().getFromProtoDef("wrist_3_link")

cube_node = robot.getFromDef("cube2")


def is_valid_vector(v, size=None):
    if v is None:
        return False

    arr = np.array(v, dtype=float)

    if size is not None and arr.size != size:
        return False

    return np.all(np.isfinite(arr))


def make_pick_targets_from_cube(cube_world):
    """
    Tạo 2 target cho flange/TCP trong hệ World:
    - p_above_world: điểm đứng trên cube
    - p_down_world : điểm thấp hơn để chuẩn bị gắp

    Các offset này sẽ cần chỉnh theo gripper thật của bạn.
    """
    PICK_DOWN_Z_OFFSET = 0.12
    PICK_ABOVE_EXTRA_Z = 0.10

    p_down_world = np.array([
        cube_world[0],
        cube_world[1],
        cube_world[2] + PICK_DOWN_Z_OFFSET
    ])

    p_above_world = np.array([
        cube_world[0],
        cube_world[1],
        cube_world[2] + PICK_DOWN_Z_OFFSET + PICK_ABOVE_EXTRA_Z
    ])

    return p_above_world, p_down_world


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

    # Lấy cube trong World bằng camera + calibration đã sửa đúng
    cube_world = camera_utils.get_cube_world_position(
        T_base_flange,
        T_world_base
    )

    if cube_world is None:
        if step_count % 20 == 0:
            print("[IK TEST] Camera chưa thấy cube...")
        step_count += 1
        continue

    if not is_valid_vector(cube_world, size=3):
        print(f"[IK TEST] cube_world invalid: {cube_world}")
        continue

    print(f"\n[IK TEST] Cube World: {[round(float(v), 4) for v in cube_world]}")

    # 1. Tạo target trong World
    p_above_world, p_down_world = make_pick_targets_from_cube(cube_world)

    # 2. Đổi target từ World -> Base
    p_above_base = motion.world_point_to_base(p_above_world)
    p_down_base = motion.world_point_to_base(p_down_world)

    if not is_valid_vector(p_above_base, size=3):
        print(f"[IK TEST] p_above_base invalid: {p_above_base}")
        continue

    if not is_valid_vector(p_down_base, size=3):
        print(f"[IK TEST] p_down_base invalid: {p_down_base}")
        continue

    print(f"[IK TEST] Target above base: {[round(float(v), 4) for v in p_above_base]}")
    print(f"[IK TEST] Target down base : {[round(float(v), 4) for v in p_down_base]}")

    # 3. IK cho điểm above
    q_above = motion.inverse_kinematics_position(
        target_pos_base=p_above_base,
        seed_q=poses.PICK_ABOVE
    )

    if not is_valid_vector(q_above, size=6):
        print(f"[IK TEST] Không tìm được q_above hoặc q_above invalid: {q_above}")
        continue

    # 4. IK cho điểm down, lấy q_above làm seed
    q_down = motion.inverse_kinematics_position(
        target_pos_base=p_down_base,
        seed_q=q_above
    )

    if not is_valid_vector(q_down, size=6):
        print(f"[IK TEST] Không tìm được q_down hoặc q_down invalid: {q_down}")
        continue

    print(f"[IK TEST] q_above: {[round(float(v), 4) for v in q_above]}")
    print(f"[IK TEST] q_down : {[round(float(v), 4) for v in q_down]}")

    # 5. Đi tới vị trí phía trên cube
    ok = motion.goto_pose(q_above, steps=100)

    if not ok:
        print("[IK TEST] goto_pose(q_above) failed")
        continue

    print("[IK TEST] Đã tới q_above.")

    motion.wait_steps(20)

    # 6. Hạ xuống vị trí gắp thử, chưa đóng gripper
    ok = motion.goto_pose(q_down, steps=60)

    if not ok:
        print("[IK TEST] goto_pose(q_down) failed")
        continue

    motion.wait_steps(20)

    gripper.close_gripper()
    motion.wait_steps(80)

    motion.goto_pose(q_above, steps=80)

    print("[IK TEST] Đã gắp thử và nâng lên.")
    break


