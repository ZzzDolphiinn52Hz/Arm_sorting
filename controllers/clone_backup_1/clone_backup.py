from controller import Supervisor

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
    # Phương án dự phòng nếu bạn chưa đặt DEF name: Lấy khâu cuối cùng trên mô hình cơ học
    end_effector_node = robot.getSelf().getFromProtoDef("wrist_3_link")

cube_node = robot.getFromDef("cube2")

# =========================
# MAIN LOOP
# =========================
while robot.step(TIME_STEP) != -1:

    # 1. Go home
    motion.goto_pose(poses.HOME, steps=50)

    # 2. Lấy các ma trận Động học thuận đã chuẩn hóa ở Bước 1
    current_q = motion.get_current_joint_positions()
    T_flange_base = motion.forward_kinematics(current_q)       # Hệ Base
    T_world_base = motion.forward_kinematics_world(current_q)            # Lấy ma trận Rz(pi/2) tịnh tiến Z đã sửa ở Bước 1
    
    # 3. Đọc dữ liệu camera thời gian thực
    cube_data = camera_utils.detect_cube()
    if cube_data is not None:
        pos_camera = cube_data["position_3d"]  # Mảng [X, Y, Z] thô từ camera
        
        # Thực hiện kiểm tra Bước 2
        camera_utils.debug_check_camera_transform(pos_camera, T_flange_base, T_world_base, cube_node)


    
    # 2. Open gripper
    gripper.open_gripper()
    motion.wait_steps(10)

    # 3. Move to waiting position above conveyor
    motion.goto_pose(poses.PICK_ABOVE, steps=50)

    # Lấy tọa độ thực tế từ Webots Node của End-Effector để làm mốc đối chứng
    webots_pos = end_effector_node.getPosition() 
    webots_ori = end_effector_node.getOrientation() 
    
    # Chạy hàm kiểm tra liên tục
    fk_is_ok = motion.debug_check_fk(webots_pos, webots_ori)
    
    if not fk_is_ok:
        print("Dừng lại! Hãy điều chỉnh mô hình toán học khâu nối trong motion.py")
        # robot.simulationQuit(0) # Có thể dừng mô phỏng để sửa code

    
    # 3.1 Track cube until aligned
    tracked_pan = camera_utils.wait_until_cube_ready(
        move_ur_to_fn=motion.move_ur_to,
        get_joints_fn=motion.get_current_joint_positions,
        max_steps=500
    )
    
    # 3.1 Chạy vòng lặp quét camera nhưng KHÔNG cho robot di chuyển
    tracked_pan = camera_utils.wait_until_cube_ready(
        move_ur_to_fn=lambda q: None,  # MẸO: Hàm rỗng này sẽ "nuốt" lệnh di chuyển, giữ robot đứng yên!
        get_joints_fn=motion.get_current_joint_positions,
        max_steps=500
    )
    

    if tracked_pan is None:
        continue  # Nothing found — restart loop


    # 4. Descend and pick while tracking in real time
    camera_utils.dynamic_descend_and_pick(
        move_ur_to_fn=motion.move_ur_to,
        get_joints_fn=motion.get_current_joint_positions,
        pick_down_pose=poses.PICK_DOWN,
        steps=25
    )
    
    # 5. Close gripper
    gripper.close_gripper()
    motion.wait_steps(80)

    # 6. Lift straight up using the actual pan angle at the moment of grasp
    current_pan = motion.get_current_joint_positions()[0]
    dynamic_pick_above = list(poses.PICK_ABOVE)
    dynamic_pick_above[0] = current_pan
    motion.goto_pose(dynamic_pick_above, steps=30)

    # 7. Check whether pick succeeded
    if camera_utils.check_if_picked_successfully(wait_steps_fn=motion.wait_steps):
        # Success path: carry to bin and drop
        motion.goto_pose(poses.SAFE_MID,   steps=80)
        motion.goto_pose(poses.BIN_ABOVE,  steps=100)
        motion.goto_pose(poses.BIN_DOWN,   steps=80)

        gripper.open_gripper()
        motion.wait_steps(80)

        motion.goto_pose(poses.BIN_ABOVE,  steps=80)
        motion.goto_pose(poses.SAFE_MID,   steps=80)
    else:
        # Miss path: skip bin run, go straight back home
        print("Pick missed. Returning HOME for next target...")

    # Back to home for next cycle
    motion.goto_pose(poses.HOME, steps=100)

