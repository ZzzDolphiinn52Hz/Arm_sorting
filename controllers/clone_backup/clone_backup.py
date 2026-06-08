from controller import Supervisor

import poses
import gripper
import motion
import camera_utils
import numpy as np

# =========================
# INIT
# =========================

TIME_STEP = 32
robot = Supervisor()

motion.init(robot, TIME_STEP)
gripper.init(robot, TIME_STEP)
camera_utils.init(robot, TIME_STEP, camera_name="camera")

"""
# calibration hệ tọa độ camera
camera_node = robot.getFromDef("CAMERA_NODE")

def node_to_transform(node):
    T = np.identity(4)
    T[0:3, 0:3] = np.array(node.getOrientation()).reshape((3, 3))
    T[0:3, 3] = np.array(node.getPosition())
    return T

if camera_node is None:
    print("[ERROR] Không tìm thấy DEF CAMERA_NODE. Hãy đặt DEF cho Camera trong .wbt")
    robot.simulationQuit(1)
"""

end_effector_node = robot.getFromDef("tool_slot")
if end_effector_node is None:
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
    T_base_flange = motion.forward_kinematics(current_q)       # Hệ Base
    T_world_base = motion.get_world_base_transform()           # Hệ world
    
    """
    # calibration hệ tọa độ camera
    
    T_world_flange = T_world_base @ T_base_flange
    T_world_camera = node_to_transform(camera_node)

    T_flange_camera = np.linalg.inv(T_world_flange) @ T_world_camera

    print("\n=== T_FLANGE_CAMERA CALIBRATION ===")
    print(np.round(T_flange_camera, 6))
    """

    # 3. Đọc dữ liệu camera thời gian thực
    cube_data = camera_utils.detect_cube()
    if cube_data is not None:
        pos_camera = cube_data["position_3d"]  # Mảng [X, Y, Z] thô từ camera
        
        # Thực hiện kiểm tra Bước 2
        camera_utils.debug_check_camera_transform(
            pos_camera, 
            T_base_flange,
            T_world_base, 
            cube_node
        )
    
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
        motion.goto_pose(poses.SAFE_MID,   steps=40)
        motion.goto_pose(poses.BIN_ABOVE,  steps=40)
        motion.goto_pose(poses.BIN_DOWN,   steps=40)

        gripper.open_gripper()
        motion.wait_steps(80)

        motion.goto_pose(poses.BIN_ABOVE,  steps=40)
        motion.goto_pose(poses.SAFE_MID,   steps=40)
    else:
        # Miss path: skip bin run, go straight back home
        print("Pick missed. Returning HOME for next target...")

    # Back to home for next cycle
    motion.goto_pose(poses.HOME, steps=40)
    
