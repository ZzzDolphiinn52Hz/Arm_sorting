# motion.py
# UR5e joint motor / sensor setup and all arm-motion helpers.
# Optimized version: supports any flange approach axis and stricter orientation checks.

import numpy as np

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

# Slightly faster than 0.8 so the arm can use more of the camera-visible belt
# before the cube leaves the reachable window. Keep below motor maxVelocity.
DEFAULT_MOTOR_VELOCITY = 1.2


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
        motor.setVelocity(DEFAULT_MOTOR_VELOCITY)
        sensor.enable(time_step)
        _ur_motors.append(motor)
        _ur_sensors.append(sensor)


# ------------------------------------------------------------------
# Primitives
# ------------------------------------------------------------------

def move_ur_to(q):
    """Command all UR joints to positions in q."""
    for motor, pos in zip(_ur_motors, q):
        motor.setPosition(float(pos))


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
    start_q = get_current_joint_positions()

    for i in range(steps + 1):
        ratio = i / steps
        q = [start + ratio * (target - start) for start, target in zip(start_q, target_q)]
        move_ur_to(q)

        if _robot.step(_time_step) == -1:
            return False

    return True


def wait_until_reached(target_q, tolerance=0.04, max_steps=500):
    errors = [999.0]
    for _ in range(max_steps):
        current_q = get_current_joint_positions()
        errors = [abs(c - t) for c, t in zip(current_q, target_q)]

        if max(errors) < tolerance:
            return True

        if _robot.step(_time_step) == -1:
            return False

    print(f"[MOTION WARNING] wait_until_reached timeout. Max joint error = {round(max(errors), 5)}")
    return False


def goto_pose(target_q, steps=100, tolerance=0.02):
    ok = smooth_move_ur_to(target_q, steps=steps)
    if not ok:
        return False
    return wait_until_reached(target_q, tolerance=tolerance)


# ------------------------------------------------------------------
# Forward Kinematics (UR5e Modified DH / Craig convention)
# ------------------------------------------------------------------

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
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ ct,      -st,     0,    a    ],
        [ st*ca,    ct*ca, -sa,  -sa*d ],
        [ st*sa,    ct*sa,  ca,   ca*d ],
        [ 0,        0,      0,    1    ]
    ])


def forward_kinematics(q):
    """Return Base <- Flange homogeneous transform from six UR5e joint angles."""
    T = np.eye(4)
    for i, (alpha, a, d, offset) in enumerate(_DH):
        theta = q[i] + offset
        T = T @ _dh_matrix(alpha, a, d, theta)
    return T


def get_world_base_transform():
    T_world_base = np.array([
        [ 0, -1,  0,  0.000000 ],
        [ 1,  0,  0,  0.000000 ],
        [ 0,  0,  1,  0.364645 ],
        [ 0,  0,  0,  1.000000 ]
    ])

    T_pedestal_offset = np.eye(4)
    T_pedestal_offset[2, 3] = 0.2053

    return T_world_base @ T_pedestal_offset


def forward_kinematics_world(q):
    return get_world_base_transform() @ forward_kinematics(q)


def world_point_to_base(p_world):
    T_base_world = np.linalg.inv(get_world_base_transform())
    p_world_h = np.array([p_world[0], p_world[1], p_world[2], 1.0])
    return (T_base_world @ p_world_h)[0:3]


def base_point_to_world(p_base):
    p_base_h = np.array([p_base[0], p_base[1], p_base[2], 1.0])
    return (get_world_base_transform() @ p_base_h)[0:3]


LOWER_LIMIT = np.array([-6.28, -6.28, -6.28, -6.28, -6.28, -6.28])
UPPER_LIMIT = np.array([ 6.28,  6.28,  6.28,  6.28,  6.28,  6.28])


def clamp_q(q):
    return np.minimum(np.maximum(q, LOWER_LIMIT), UPPER_LIMIT)


def fk_position(q):
    return forward_kinematics(q)[0:3, 3]


def fk_rotation(q):
    return forward_kinematics(q)[0:3, 0:3]


def _normalize(v, eps=1e-9):
    v = np.array(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        return v
    return v / n


def numerical_jacobian_position(q, eps=1e-4):
    q = np.array(q, dtype=float)
    p0 = fk_position(q)
    J = np.zeros((3, 6))

    for i in range(6):
        q_eps = q.copy()
        q_eps[i] += eps
        p_eps = fk_position(q_eps)
        J[:, i] = (p_eps - p0) / eps

    return J


def rotation_vector_from_matrix(R):
    cos_angle = (np.trace(R) - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)

    vee = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])

    if angle < 1e-6:
        return 0.5 * vee

    return (angle / (2.0 * np.sin(angle))) * vee


def orientation_error(R_current, R_target):
    R_err = R_target @ R_current.T
    return rotation_vector_from_matrix(R_err)


def numerical_jacobian_pose(q, eps=1e-4):
    q = np.array(q, dtype=float)
    T0 = forward_kinematics(q)
    p0 = T0[0:3, 3]
    R0 = T0[0:3, 0:3]

    J = np.zeros((6, 6))

    for i in range(6):
        q_eps = q.copy()
        q_eps[i] += eps

        T_eps = forward_kinematics(q_eps)
        p_eps = T_eps[0:3, 3]
        R_eps = T_eps[0:3, 0:3]

        J[0:3, i] = (p_eps - p0) / eps
        dR = R_eps @ R0.T
        J[3:6, i] = rotation_vector_from_matrix(dR) / eps

    return J


def _parse_flange_axis(flange_axis):
    axis = str(flange_axis).strip().lower()
    sign = -1 if axis.startswith("-") else 1
    name = axis[1:] if axis.startswith("-") else axis
    if name not in ("x", "y", "z"):
        raise ValueError("flange_axis phải là một trong: 'x', '-x', 'y', '-y', 'z', '-z'.")
    idx = {"x": 0, "y": 1, "z": 2}[name]
    return idx, sign, name


def make_gripper_down_rotation(seed_q=None, flange_axis="z", yaw_ref_base=None):
    """
    Create target flange orientation so the selected local flange/gripper axis
    points vertically down to the conveyor.

    flange_axis can be: 'x', '-x', 'y', '-y', 'z', '-z'.
    Meaning: the chosen local axis of the flange is the physical approach axis
    of the gripper.
    """
    if seed_q is None:
        seed_q = get_current_joint_positions()

    R_seed = fk_rotation(seed_q)
    axis_idx, sign, _ = _parse_flange_axis(flange_axis)

    down_base = np.array([0.0, 0.0, -1.0])
    # If approach_axis = sign * R[:, axis_idx] must equal down,
    # then R[:, axis_idx] = sign * down.
    approach_col = sign * down_base

    cols = [None, None, None]
    cols[axis_idx] = approach_col

    # Preserve yaw as much as possible by projecting one seed axis onto the
    # horizontal plane perpendicular to the approach column.
    if yaw_ref_base is not None:
        ref = np.array(yaw_ref_base, dtype=float)
        ref_idx = 0 if axis_idx != 0 else 1
    else:
        ref_idx = 0 if axis_idx != 0 else 1
        ref = R_seed[:, ref_idx]

    ref = ref - np.dot(ref, approach_col) * approach_col
    if np.linalg.norm(ref) < 1e-6:
        ref = np.array([1.0, 0.0, 0.0])
        ref = ref - np.dot(ref, approach_col) * approach_col
    if np.linalg.norm(ref) < 1e-6:
        ref = np.array([0.0, 1.0, 0.0])
        ref = ref - np.dot(ref, approach_col) * approach_col

    cols[ref_idx] = _normalize(ref)

    missing_idx = list({0, 1, 2} - {axis_idx, ref_idx})[0]

    def fill_missing():
        if missing_idx == 0:
            cols[0] = _normalize(np.cross(cols[1], cols[2]))
        elif missing_idx == 1:
            cols[1] = _normalize(np.cross(cols[2], cols[0]))
        else:
            cols[2] = _normalize(np.cross(cols[0], cols[1]))

    fill_missing()

    # Recompute the reference column so x cross y = z exactly while preserving
    # the requested approach column.
    if ref_idx == 0:
        cols[0] = _normalize(np.cross(cols[1], cols[2]))
    elif ref_idx == 1:
        cols[1] = _normalize(np.cross(cols[2], cols[0]))
    else:
        cols[2] = _normalize(np.cross(cols[0], cols[1]))

    R_target = np.column_stack(cols)
    return R_target


def get_gripper_approach_axis(q, flange_axis="z"):
    R = fk_rotation(q)
    axis_idx, sign, _ = _parse_flange_axis(flange_axis)
    return sign * R[:, axis_idx]


def gripper_down_angle_deg(q, flange_axis="z"):
    approach_axis = get_gripper_approach_axis(q, flange_axis=flange_axis)
    desired_down = np.array([0.0, 0.0, -1.0])
    dot_val = np.clip(np.dot(_normalize(approach_axis), desired_down), -1.0, 1.0)
    return float(np.degrees(np.arccos(dot_val)))


def inverse_kinematics_pose(
    target_pos_base,
    target_R_base_flange,
    seed_q=None,
    max_iters=180,
    pos_tolerance=0.003,
    ori_tolerance=0.025,
    damping=0.05,
    max_step=0.06,
    position_weight=1.0,
    orientation_weight=1.4,
    stay_near_seed=0.002,
    verbose=False,
):
    """Numerical damped-least-squares IK for flange position + orientation."""
    if seed_q is None:
        q = np.array(get_current_joint_positions(), dtype=float)
    else:
        q = np.array(seed_q, dtype=float)

    seed = q.copy()
    target_pos = np.array(target_pos_base, dtype=float)
    target_R = np.array(target_R_base_flange, dtype=float)

    for it in range(max_iters):
        T = forward_kinematics(q)
        current_pos = T[0:3, 3]
        current_R = T[0:3, 0:3]

        pos_error = target_pos - current_pos
        ori_error = orientation_error(current_R, target_R)

        pos_norm = np.linalg.norm(pos_error)
        ori_norm = np.linalg.norm(ori_error)

        if pos_norm < pos_tolerance and ori_norm < ori_tolerance:
            if verbose:
                print(f"[IK-POSE] Success at iter {it}, pos_error={round(pos_norm, 6)} m, ori_error={round(np.degrees(ori_norm), 3)} deg")
            return q.tolist()

        J = numerical_jacobian_pose(q)
        J_weighted = J.copy()
        J_weighted[0:3, :] *= position_weight
        J_weighted[3:6, :] *= orientation_weight

        error = np.concatenate((position_weight * pos_error, orientation_weight * ori_error))

        A = J_weighted @ J_weighted.T + (damping ** 2) * np.eye(6)
        dq = J_weighted.T @ np.linalg.solve(A, error)

        dq += stay_near_seed * (seed - q)

        dq_norm = np.linalg.norm(dq)
        if dq_norm > max_step:
            dq = dq / dq_norm * max_step

        q = clamp_q(q + dq)

    if verbose:
        T = forward_kinematics(q)
        final_pos_error = np.linalg.norm(target_pos - T[0:3, 3])
        final_ori_error = np.linalg.norm(orientation_error(T[0:3, 0:3], target_R))
        print(f"[IK-POSE] Failed. pos_error={round(final_pos_error, 6)} m, ori_error={round(np.degrees(final_ori_error), 3)} deg")
    return None


def inverse_kinematics_downward(target_pos_base, seed_q=None, flange_axis="z", **kwargs):
    if seed_q is None:
        seed_q = get_current_joint_positions()

    target_R = make_gripper_down_rotation(seed_q=seed_q, flange_axis=flange_axis)
    return inverse_kinematics_pose(
        target_pos_base=target_pos_base,
        target_R_base_flange=target_R,
        seed_q=seed_q,
        **kwargs
    )


def debug_check_gripper_down(q, flange_axis="z"):
    approach_axis = get_gripper_approach_axis(q, flange_axis=flange_axis)
    angle_deg = gripper_down_angle_deg(q, flange_axis=flange_axis)

    print("\n=== [CHECK GRIPPER ORIENTATION] ===")
    print(f"flange_axis       : {flange_axis}")
    print(f"Approach axis base: {[round(float(v), 4) for v in approach_axis]}")
    print(f"Desired down axis : {[0.0, 0.0, -1.0]}")
    print(f"Angle to vertical : {round(angle_deg, 3)} deg")

    return angle_deg


def inverse_kinematics_position(
    target_pos_base,
    seed_q=None,
    max_iters=120,
    tolerance=0.002,
    damping=0.04,
    max_step=0.08,
    stay_near_seed=0.01,
    verbose=False,
):
    if seed_q is None:
        q = np.array(get_current_joint_positions(), dtype=float)
    else:
        q = np.array(seed_q, dtype=float)

    seed = q.copy()
    target = np.array(target_pos_base, dtype=float)

    for it in range(max_iters):
        current_pos = fk_position(q)
        error = target - current_pos
        error_norm = np.linalg.norm(error)

        if error_norm < tolerance:
            if verbose:
                print(f"[IK] Success at iter {it}, error={round(error_norm, 6)} m")
            return q.tolist()

        J = numerical_jacobian_position(q)
        A = J @ J.T + (damping ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(A, error)
        dq += stay_near_seed * (seed - q)

        dq_norm = np.linalg.norm(dq)
        if dq_norm > max_step:
            dq = dq / dq_norm * max_step

        q = clamp_q(q + dq)

    if verbose:
        final_error = np.linalg.norm(target - fk_position(q))
        print(f"[IK] Failed. Final error={round(final_error, 6)} m")
    return None


def debug_check_fk(webots_position, webots_orientation=None):
    current_q = get_current_joint_positions()
    T_world = forward_kinematics_world(current_q)
    calc_pos = T_world[0:3, 3]

    print("\n=== [CHECK STEP 1] FORWARD KINEMATICS VERIFICATION ===")
    print(f"Goc khop thuc te q : {[round(a, 4) for a in current_q]}")
    print(f"Vi tri Webots do   : {[round(v, 6) for v in webots_position]}")
    print(f"Vi tri FK tinh ra  : {[round(p, 6) for p in calc_pos]}")

    error = np.linalg.norm(np.array(webots_position) - calc_pos)
    print(f"--> Sai so Euclidean: {round(error, 6)} met")

    if error < 0.005:
        print("[STATUS]: BUOC 1 DAT CHUAN! San sang chuyen sang Buoc 2.")
        return True

    print("[WARNING]: FK bi lech! Kiem tra lai DH params hoac base transform.")
    return False
