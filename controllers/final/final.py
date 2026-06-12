"""Main entry point for the UR5e dynamic sorting demo."""

from controller import Supervisor

import camera_utils
import config
import gripper
import motion
import pick_place
import poses


def init_system():
    robot = Supervisor()

    motion.init(robot, config.TIME_STEP)
    gripper.init(robot, config.TIME_STEP)
    camera_utils.init(robot, config.TIME_STEP, camera_name="camera")

    # Optional sanity lookup. Kept here to catch scene-name changes early.
    end_effector_node = robot.getFromDef("tool_slot")
    if end_effector_node is None:
        end_effector_node = robot.getSelf().getFromProtoDef("wrist_3_link")

    return robot


def warmup(robot):
    for _ in range(10):
        if robot.step(config.TIME_STEP) == -1:
            return False

    motion.move_ur_to(poses.HOME)
    gripper.open_gripper()
    return motion.goto_pose(poses.PICK_ABOVE, steps=80)


def main():
    robot = init_system()

    if not warmup(robot):
        print("[STARTUP] Warm-up failed.")
        return

    print("=== UR5E DYNAMIC SORTING LOOP START ===")

    while robot.step(config.TIME_STEP) != -1:
        ok = pick_place.dynamic_pick_object(robot)

        if ok:
            print("[SORT] Pick cycle finished. Trying next object...")
        else:
            print("[SORT] Pick failed. Trying next object...")


if __name__ == "__main__":
    main()
