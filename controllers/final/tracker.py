"""Vision target selection and dynamic object tracking."""

import numpy as np

import camera_utils
import config
import motion
import sorting

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


def is_valid_vector(v, size=None):
    if v is None:
        return False
    arr = np.array(v, dtype=float)
    if size is not None and arr.size != size:
        return False
    return np.all(np.isfinite(arr))


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


def clamp_velocity(v):
    v = np.array(v, dtype=float)
    speed = np.linalg.norm(v)
    if speed > config.MAX_CUBE_SPEED:
        v = v / speed * config.MAX_CUBE_SPEED
    return v


def select_target_object(objects_base):
    """Choose the best classified object to pick."""
    candidates = []

    for obj in objects_base:
        obj_bin = obj.get("bin")
        if obj_bin in [None, "UNKNOWN_BIN"]:
            continue

        if not sorting.is_allowed_bin(obj_bin):
            continue

        p = obj.get("base_position", None)
        if not is_valid_vector(p, size=3):
            continue

        p = np.array(p, dtype=float)
        if abs(p[1]) > config.PICK_Y_LIMIT:
            continue

        score = abs(p[0] - config.PICK_X_BEST) * 2.0 + abs(p[1]) * 1.0
        candidates.append((score, obj))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]


def find_locked_object(objects_base):
    """Keep tracking the same object by recognition id or nearest neighbor."""
    if not objects_base:
        return None

    if tracker["id"] is not None:
        for obj in objects_base:
            if obj.get("id") == tracker["id"]:
                return obj

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

        if best_obj is not None and best_dist < config.LOCK_DISTANCE:
            return best_obj

    return select_target_object(objects_base)


def update_cube_tracker():
    """
    Legacy cube-only tracker.
    Kept for compatibility; full sorting uses update_object_tracker().
    """
    current_q = motion.get_current_joint_positions()
    if not is_valid_vector(current_q, size=6):
        return None, False, None

    T_base_flange = motion.forward_kinematics(current_q)

    meas, candidate = camera_utils.get_selected_cube_base_position(
        T_base_flange,
        last_base=tracker["pos"],
        target_model_name="cube",
        min_area=config.MIN_CUBE_AREA,
        workspace=config.WORKSPACE,
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
                tracker["pos"] = meas
                tracker["vel"] = np.zeros(3)
            else:
                raw_vel = clamp_velocity((meas - tracker["last_meas"]) / config.DT)
                tracker["vel"] = (1.0 - config.VEL_ALPHA) * tracker["vel"] + config.VEL_ALPHA * raw_vel
                tracker["pos"] = (1.0 - config.CUBE_ALPHA) * tracker["pos"] + config.CUBE_ALPHA * meas

        tracker["last_meas"] = meas
        tracker["lost"] = 0
        return tracker["pos"], True, candidate

    if tracker["pos"] is not None and tracker["lost"] < config.LOST_MAX_STEPS:
        tracker["pos"] = tracker["pos"] + tracker["vel"] * config.DT
        tracker["lost"] += 1
        return tracker["pos"], False, None

    return None, False, None


def update_object_tracker():
    """
    Full sorting tracker:
    - reads all camera-recognized objects;
    - chooses or keeps one target;
    - stores shape/color/bin metadata;
    - returns the filtered base-frame position for IK.
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
                raw_vel = clamp_velocity((meas - tracker["last_meas"]) / config.DT)
                tracker["vel"] = (1.0 - config.VEL_ALPHA) * tracker["vel"] + config.VEL_ALPHA * raw_vel
                tracker["pos"] = (1.0 - config.CUBE_ALPHA) * tracker["pos"] + config.CUBE_ALPHA * meas

            tracker["last_meas"] = meas
            tracker["lost"] = 0

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

    if tracker["pos"] is not None and tracker["lost"] < config.LOST_MAX_STEPS:
        tracker["pos"] = tracker["pos"] + tracker["vel"] * config.DT
        tracker["lost"] += 1
        return tracker["pos"], False

    return None, False
