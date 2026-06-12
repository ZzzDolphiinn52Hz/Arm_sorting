# UR5e Dynamic Sorting Controller - Clean Version

## Purpose
This folder contains a cleaned and modular version of the previous `clone_backup_1.py` controller.
The runtime logic is intentionally kept close to the stable demo version, but responsibilities are split into small files.

## File map

- `main.py`: Webots entry point, system initialization, warm-up, main loop.
- `config.py`: all tunable constants for timing, pick geometry, tracking, workspace, and test mode.
- `sorting.py`: bin-to-pose map and sorting helper functions.
- `tracker.py`: target selection, target locking, position filtering, velocity estimation.
- `pick_place.py`: dynamic IK tracking, pick sequence, lift, and place-to-bin sequence.
- `motion.py`: UR5e motor setup, FK, IK, and pose motion utilities.
- `gripper.py`: gripper setup and open/close commands.
- `camera_utils.py`: Webots recognition, shape/color/bin detection, camera-to-base conversion.
- `poses.py`: predefined joint-space poses.

## How to use in Webots

1. Create a controller folder, for example:

   `controllers/ur5e_sorting_clean/`

2. Copy all `.py` files from this package into that folder.

3. In the Webots Scene Tree, set the UR5e/Supervisor controller name to:

   `ur5e_sorting_clean`

4. Start the simulation.

## Main tuning points

Edit `config.py` for normal tuning:

- `GRIPPER_DOWN_AXIS`: change if the gripper is tilted or upside down.
- `GRIPPER_TIP_OFFSET_FLANGE`: tune if the flange reaches correctly but the gripper tip is shifted.
- `LEAD_TIME_ABOVE`, `LEAD_TIME_DOWN`, `LEAD_TIME_CLOSE`: tune dynamic conveyor prediction.
- `PICK_X_MIN`, `PICK_X_MAX`, `PICK_Y_LIMIT`, `PICK_RADIUS_MAX`: tune pick workspace.
- `TEST_ONLY_BIN`: set to a bin name for single-category testing, or `None` for full sorting.

## Important customer notes

- If the camera position or orientation changes in the Webots scene, recalibrate `T_FLANGE_CAMERA` in `camera_utils.py`.
- If the robot base or pedestal changes, recalibrate `get_world_base_transform()` in `motion.py`.
- Sorting depends on Webots recognition data. Objects should have clear `model` values and `recognitionColors` in the world/PROTO files.
- Do not transfer `.pyc` files to the customer. They are temporary Python cache files.
