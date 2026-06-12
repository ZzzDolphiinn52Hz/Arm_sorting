"""Dynamic pick-and-place sequence for UR5e sorting."""

import math
import numpy as np

import config
import gripper
import motion
import poses
import sorting
import tracker as object_tracker


def limit_joint_step(current_q, target_q, max_step=config.MAX_JOINT_STEP):
    current_q = np.array(current_q, dtype=float)
    target_q = np.array(target_q, dtype=float)
    dq = target_q - current_q
    dq = np.clip(dq, -max_step, max_step)
    return (current_q + dq).tolist()


def make_pick_targets_from_cube_base(cube_base, seed_q=None):
    """
    Create FLANGE targets in base frame from object center in base frame.

    p_tip_base = object top + clearance
    p_flange_base = p_tip_base - R_base_flange @ tip_offset_flange
    """
    if seed_q is None:
        seed_q = motion.get_current_joint_positions()

    target_R = motion.make_gripper_down_rotation(
        seed_q=seed_q,
        flange_axis=config.GRIPPER_DOWN_AXIS,
    )

    cube_base = np.array(cube_base, dtype=float)
    p_tip_down_base = np.array([
        cube_base[0],
        cube_base[1],
        cube_base[2] + config.CUBE_HALF_HEIGHT + config.PICK_CLEARANCE,
    ])

    p_down_base = p_tip_down_base - target_R @ config.GRIPPER_TIP_OFFSET_FLANGE
    p_above_base = p_down_base + np.array([0.0, 0.0, config.ABOVE_CLEARANCE])

    return p_above_base, p_down_base, target_R


def is_inside_broad_workspace(p):
    x, y, _ = p
    r = math.sqrt(x * x + y * y)

    if abs(y) > config.PICK_Y_LIMIT:
        return False, "out_y"
    if r > config.PICK_RADIUS_MAX:
        return False, "out_radius"
    if x > config.PICK_X_MAX:
        return False, "too_early"
    if x < config.PICK_X_MIN:
        return False, "too_late"

    return True, "inside"


def dynamic_ik_track_step(target_mode, lead_time, reach_tolerance, ori_tolerance_deg=3.0):
    object_base, seen = object_tracker.update_object_tracker()

    if object_base is None:
        return False, None, None, None, seen, "no_object"

    object_pred = object_base + object_tracker.tracker["vel"] * lead_time
    _, workspace_status = is_inside_broad_workspace(object_pred)

    if workspace_status == "too_late":
        object_tracker.tracker["too_late_count"] += 1
        if object_tracker.tracker["too_late_count"] >= config.TOO_LATE_GRACE_STEPS:
            return False, None, None, None, seen, "too_late"
        return False, None, None, None, seen, "waiting_too_late_confirm"

    object_tracker.tracker["too_late_count"] = 0

    if workspace_status in ("too_early", "out_y", "out_radius"):
        return False, None, None, None, seen, "waiting_" + workspace_status

    current_q = motion.get_current_joint_positions()
    if not object_tracker.is_valid_vector(current_q, size=6):
        return False, None, None, None, seen, "bad_q"

    p_above_base, p_down_base, target_R = make_pick_targets_from_cube_base(
        object_pred,
        seed_q=current_q,
    )
    target_base = p_above_base if target_mode == "above" else p_down_base

    flange_pos = motion.fk_position(current_q)
    pos_error = float(np.linalg.norm(np.array(target_base) - np.array(flange_pos)))
    ori_error_deg = float(
        motion.gripper_down_angle_deg(
            current_q,
            flange_axis=config.GRIPPER_DOWN_AXIS,
        )
    )

    if pos_error < reach_tolerance and ori_error_deg < ori_tolerance_deg:
        object_tracker.tracker["ik_fail_count"] = 0
        return True, target_base, pos_error, ori_error_deg, seen, "reached"

    q_target = motion.inverse_kinematics_pose(
        target_pos_base=target_base,
        target_R_base_flange=target_R,
        seed_q=current_q,
        max_iters=180,
        pos_tolerance=0.005,
        ori_tolerance=np.radians(1.2),
        damping=0.055,
        max_step=0.060,
        position_weight=1.0,
        orientation_weight=2.4,
        stay_near_seed=0.0015,
        verbose=False,
    )

    if not object_tracker.is_valid_vector(q_target, size=6):
        object_tracker.tracker["ik_fail_count"] += 1
        if object_tracker.tracker["ik_fail_count"] >= config.IK_FAIL_GRACE_STEPS:
            return False, target_base, pos_error, ori_error_deg, seen, "ik_fail_final"
        return False, target_base, pos_error, ori_error_deg, seen, "ik_fail_wait"

    object_tracker.tracker["ik_fail_count"] = 0
    q_cmd = limit_joint_step(current_q, q_target)
    motion.move_ur_to(q_cmd)

    return False, target_base, pos_error, ori_error_deg, seen, "tracking"


def dynamic_track_to_object(
    robot,
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
            print("[DYNAMIC] Object left the real pick zone. Skipping this object.")
            return False

        if status == "ik_fail_final":
            print("[DYNAMIC] IK failed for too long on current object. Skipping it.")
            return False

        if target_base is None:
            if i % config.DYNAMIC_WAIT_LOG_PERIOD == 0:
                print(f"[DYNAMIC] mode={target_mode}, status={status}, seen={seen}, waiting same object...")
        else:
            if i % config.DYNAMIC_TRACK_LOG_PERIOD == 0:
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

        if robot.step(config.TIME_STEP) == -1:
            return False

    return not stop_when_reached


def dynamic_lift_after_pick(lift_height=0.18):
    current_q = motion.get_current_joint_positions()
    current_pos = motion.fk_position(current_q)

    lift_target = np.array([
        current_pos[0],
        current_pos[1],
        current_pos[2] + lift_height,
    ])

    target_R = motion.make_gripper_down_rotation(
        seed_q=current_q,
        flange_axis=config.GRIPPER_DOWN_AXIS,
    )
    q_lift = motion.inverse_kinematics_pose(
        target_pos_base=lift_target,
        target_R_base_flange=target_R,
        seed_q=current_q,
        max_iters=180,
        orientation_weight=1.5,
        verbose=False,
    )

    if not object_tracker.is_valid_vector(q_lift, size=6):
        print("[DYNAMIC] Cannot solve q_lift")
        return False

    return motion.goto_pose(q_lift, steps=30, tolerance=0.04)


def place_object_to_bin(bin_name):
    bin_name, (bin_above, bin_down) = sorting.get_bin_pose(bin_name)
    print(f"[SORT] Moving object to {bin_name}")

    if not motion.goto_pose(poses.SAFE_MID, steps=20):
        return False

    if not motion.goto_pose(bin_above, steps=30):
        return False

    if not motion.goto_pose(bin_down, steps=20):
        return False

    gripper.open_gripper()
    motion.wait_steps(8)

    if not motion.goto_pose(bin_above, steps=25):
        return False

    motion.goto_pose(poses.SAFE_MID, steps=20)
    motion.goto_pose(poses.PICK_ABOVE, steps=30)

    return True


def dynamic_pick_object(robot):
    print("DYNAMIC PICK START ===")
    object_tracker.reset_object_tracker()

    ok = dynamic_track_to_object(
        robot=robot,
        target_mode="above",
        max_steps=300,
        lead_time=config.LEAD_TIME_ABOVE,
        reach_tolerance=0.019,
        stable_required=3,
        stop_when_reached=True,
        ori_tolerance_deg=4.0,
    )

    if not ok:
        print("[DYNAMIC] Cannot track to dynamic above pose")
        object_tracker.reset_object_tracker()
        return False

    print("[DYNAMIC] Reached dynamic above zone.")

    ok = dynamic_track_to_object(
        robot=robot,
        target_mode="down",
        max_steps=190,
        lead_time=config.LEAD_TIME_DOWN,
        reach_tolerance=0.019,
        stable_required=2,
        stop_when_reached=True,
        ori_tolerance_deg=2.5,
    )

    if not ok:
        print("[DYNAMIC] Cannot descend to dynamic pick zone")
        object_tracker.reset_object_tracker()
        return False

    picked_bin = object_tracker.tracker["bin"] if object_tracker.tracker["bin"] is not None else "UNKNOWN_BIN"

    print(
        f"[DYNAMIC] Reached down zone. Closing gripper... "
        f"Picked target: shape={object_tracker.tracker['shape']}, "
        f"color={object_tracker.tracker['color']}, "
        f"bin={picked_bin}"
    )

    gripper.close_gripper()

    dynamic_track_to_object(
        robot=robot,
        target_mode="down",
        max_steps=35,
        lead_time=config.LEAD_TIME_CLOSE,
        reach_tolerance=999.0,
        stable_required=2,
        stop_when_reached=False,
        ori_tolerance_deg=4.0,
    )

    ok = dynamic_lift_after_pick(lift_height=0.18)
    if not ok:
        print("[DYNAMIC] Lift failed")
        object_tracker.reset_object_tracker()
        return False

    print("[DYNAMIC] Dynamic pick finished and object was lifted.")
    print(f"[SORT TEST] Picked bin = {picked_bin}")

    place_ok = place_object_to_bin(picked_bin)
    object_tracker.reset_object_tracker()

    return bool(place_ok)
