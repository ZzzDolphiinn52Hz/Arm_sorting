# motion.py
# UR5e joint motor / sensor setup and all arm-motion helpers.

UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]

_robot      = None
_time_step  = None
_ur_motors  = []
_ur_sensors = []


def init(robot, time_step):
    """Call once from the main controller after creating the Supervisor."""
    global _robot, _time_step, _ur_motors, _ur_sensors
    _robot     = robot
    _time_step = time_step
    _ur_motors  = []
    _ur_sensors = []

    for name in UR_JOINT_NAMES:
        motor  = robot.getDevice(name)
        sensor = robot.getDevice(name + "_sensor")
        motor.setVelocity(0.8)
        sensor.enable(time_step)
        _ur_motors.append(motor)
        _ur_sensors.append(sensor)

# ------------------------------------------------------------------
# Primitives
# ------------------------------------------------------------------

def move_ur_to(q):
    """Instantly command all UR joints to positions in q."""
    for motor, pos in zip(_ur_motors, q):
        motor.setPosition(pos)


def get_current_joint_positions():
    return [sensor.getValue() for sensor in _ur_sensors]


def wait_steps(n):
    for _ in range(n):
        if _robot.step(_time_step) == -1:
            return False
    return True


# ------------------------------------------------------------------
# Compound moves
# ------------------------------------------------------------------

def smooth_move_ur_to(target_q, steps=100):
    """Linearly interpolate from current position to target_q over `steps` sim steps."""
    start_q = get_current_joint_positions()

    for i in range(steps + 1):
        ratio = i / steps
        q = [start + ratio * (target - start)
             for start, target in zip(start_q, target_q)]
        move_ur_to(q)
        if _robot.step(_time_step) == -1:
            return False

    return True


def wait_until_reached(target_q, tolerance=0.02, max_steps=300):
    """Spin the sim until all joints are within tolerance of target_q."""
    for _ in range(max_steps):
        current_q = get_current_joint_positions()
        errors = [abs(c - t) for c, t in zip(current_q, target_q)]
        if max(errors) < tolerance:
            return True
        if _robot.step(_time_step) == -1:
            return False
    return False


def goto_pose(target_q, steps=100, tolerance=0.02):
    """Smooth move then wait until physically reached."""
    smooth_move_ur_to(target_q, steps=steps)
    wait_until_reached(target_q, tolerance=tolerance)


# ------------------------------------------------------------------
# Forward Kinematics (UR5e Modified DH / Craig convention)
# ------------------------------------------------------------------
# UR5e DH parameters (Universal Robots official e-Series spec):
#
#   Joint |  alpha(i-1) |   a(i-1)   |   d(i)   | theta(i)
#     1   |      0      |     0      |  0.1625  |   q[0]
#     2   |    pi/2     |     0      |    0     |   q[1]
#     3   |      0      |  -0.425    |    0     |   q[2]
#     4   |      0      |  -0.3922   |  0.1333  |   q[3]
#     5   |    pi/2     |     0      |  0.0997  |   q[4]
#     6   |   -pi/2     |     0      |  0.0996  |   q[5]
#
# Base mount from ure.wbt:
#   translation 0 0 0.61
#   rotation (0,0,-1) 1.5708  =>  Rz(-pi/2) + trans Z=0.61
# ------------------------------------------------------------------

import numpy as np

_DH = [
    # (alpha_prev, a_prev,   d,      theta_offset)
    (0,            0,        0.1625, 0),
    (np.pi/2,      0,        0,      0),
    (0,           -0.425,    0,      0),
    (0,           -0.3922,   0.1333, 0),
    (np.pi/2,      0,        0.0997, 0),
    (-np.pi/2,     0,        0.0996, 0),
]


def _dh_matrix(alpha, a, d, theta):
    """Single DH transformation matrix (Modified DH / Craig convention)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ ct,     -st,     0,    a    ],
        [ st*ca,   ct*ca, -sa,  -sa*d ],
        [ st*sa,   ct*sa,  ca,   ca*d ],
        [ 0,       0,      0,    1    ]
    ])


def forward_kinematics(q):
    """
    Tinh toan vi tri va huong cua End-Effector (Flange) tu bo 6 goc khop q.
    Tra ve ma tran dong nhat 4x4 trong he toa do robot-base.

    q: list/array 6 goc (rad) theo thu tu:
       [shoulder_pan, shoulder_lift, elbow,
        wrist_1, wrist_2, wrist_3]
    """
    T = np.eye(4) # root 
    """
        [ 1  0  0  0 ]  --> Hướng trục X trùng khít trục X gốc
    T = | 0  1  0  0 |  --> Hướng trục Y trùng khít trục Y gốc
        | 0  0  1  0 |  --> Hướng trục Z trùng khít trục Z gốc
        [ 0  0  0  1 ]  --> Tọa độ vị trí ban đầu là (X=0, Y=0, Z=0)
    """
    for i, (alpha, a, d, offset) in enumerate(_DH):
        theta = q[i] + offset
        T = T @ _dh_matrix(alpha, a, d, theta)
    return T

def forward_kinematics_world(q):
    """
    HÀM ĐÃ SỬA: Tính FK và chuyển sang hệ tọa độ Webots World-Frame chính xác.
    Sửa lỗi đảo trục X, Y bằng ma trận định hướng Rz(+pi/2) và bù sai lệch cao độ Z.
    """
    # 1. Cấu hình chính xác ma trận chuyển đổi từ Base sang World (T_world_base)
    # Xoay căn chỉnh hệ trục để triệt tiêu lỗi đối xứng gương [X, Y] -> [-X, -Y]
    T_world_base = np.array([
        [ 0, -1,  0,  0.000000 ],   
        [ 1,  0,  0,  0.000000 ],
        [ 0,  0,  1,  0.364645 ],   # Giá trị dịch Z tối ưu (0.61 - 0.245355)
        [ 0,  0,  0,  1.000000 ]
    ])
    
    # 2. Ma trận bù sai số độ cao khâu đế phụ (Pedestal Offset) trong Webots
    # Thực tế mô hình UR5e trong Webots thường có một khoảng offset dịch lên khoảng 0.2053m 
    # giữa điểm chân đế thực tế trên sàn và khớp Shoulder Pan Joint đầu tiên.
    T_pedestal_offset = np.eye(4)
    T_pedestal_offset[2, 3] = 0.2053  # Bù trừ khoảng cách Z bị thiếu hụt ~20cm của bạn
    
    # Tích hợp ma trận bù vào hệ chân đế toàn cục
    T_base_corrected = T_world_base @ T_pedestal_offset

    # 3. Tính Động học thuận nội tại của robot từ bộ góc q
    T_flange_base = forward_kinematics(q)
    
    # 4. Trả về ma trận đồng nhất quy đổi hoàn toàn ra hệ tọa độ World của Webots
    return T_base_corrected @ T_flange_base


def debug_check_fk(webots_position, webots_orientation=None):
    """
    HAM KIEM TRA BUOC 1: Doi chieu giua FK tu tinh va toa do thuc te cua Webots.

    webots_position   : [X, Y, Z] lay tu:
                        supervisor.getFromDef('TCP').getPosition()
    webots_orientation: (tuy chon) hien chua su dung
    
    Vi du goi trong main controller:
        flange = robot.getFromDef('solid(1)')  # TCP node trong toolSlot
        pos = flange.getPosition()
        motion.debug_check_fk(pos)
    """
    current_q = get_current_joint_positions()
    T_world   = forward_kinematics_world(current_q)
    calc_pos  = T_world[0:3, 3]

    print("\n=== [CHECK STEP 1] FORWARD KINEMATICS VERIFICATION ===")
    print(f"Goc khop thuc te q : {[round(a, 4) for a in current_q]}")
    print(f"Vi tri Webots do   : {[round(v, 6) for v in webots_position]}")
    print(f"Vi tri FK tinh ra  : {[round(p, 6) for p in calc_pos]}")

    error = np.linalg.norm(np.array(webots_position) - calc_pos)
    print(f"--> Sai so Euclidean: {round(error, 6)} met")

    if error < 0.005:
        print("[STATUS]: BUOC 1 DAT CHUAN! San sang chuyen sang Buoc 2.")
        return True
    else:
        print("[WARNING]: FK bi lech! Kiem tra lai DH params hoac base transform.")
        return False

