# UR5e Dynamic Sorting Controller

## Overview

<video src="./demo.mp4" width="100%" controls></video>

[[Video]](./demo.mp4)

This repository contains a modular UR5e robotic arm controller for **dynamic object sorting** in Webots simulation. The system uses computer vision to identify and track moving objects on a conveyor belt, dynamically predicts their trajectories, and performs real-time pick-and-place operations to sort objects into category-specific bins.

## Key Features

- **Dynamic Object Tracking**: Real-time vision-based tracking with velocity estimation for moving objects
- **Predictive IK**: Inverse kinematics with lead-time prediction to intercept moving targets
- **Multi-Category Sorting**: Sorts objects by shape, color, and model into labeled bins
- **Modular Architecture**: Clean separation of concerns across specialized modules
- **Gripper Control**: Synchronized gripper operations with pick-place sequences
- **Workspace Management**: Configurable pick zones and collision avoidance

## System Architecture

The controller is organized into the following modules in `controllers/final/`:

### Core Modules

| Module | Responsibility |
|--------|-----------------|
| **`final.py`** | Webots entry point; system initialization, warm-up sequence, and main control loop |
| **`config.py`** | Centralized tuning parameters: timing, geometry, workspace, and test mode settings |
| **`pick_place.py`** | Dynamic pick-and-place sequences: IK tracking, gripper control, and bin placement |
| **`tracker.py`** | Vision-based object selection, target locking, position filtering, and velocity estimation |
| **`motion.py`** | UR5e kinematics (FK/IK), pose motion utilities, and joint control |
| **`gripper.py`** | Gripper initialization and open/close commands |
| **`camera_utils.py`** | Webots vision recognition, object detection, and camera-to-robot frame transforms |
| **`sorting.py`** | Bin-to-pose mapping and bin-selection logic for different object categories |
| **`poses.py`** | Predefined joint-space poses (home, pick above, bin approaches, etc.) |

## How It Works

### Workflow

1. **Initialization** (`final.py`):
   - Initialize robot, motion system, gripper, and camera
   - Perform warm-up sequence to verify connectivity

2. **Main Loop** (`final.py`):
   - Repeatedly call `pick_place.dynamic_pick_object()`
   - Handle success/failure of each pick cycle

3. **Object Detection & Tracking** (`tracker.py`):
   - Read camera-recognized objects in base frame
   - Select the best candidate based on position and test filter
   - Lock onto target using recognition ID or nearest-neighbor matching
   - Estimate velocity with exponential filtering

4. **Dynamic Pick** (`pick_place.py`):
   - **Phase 1 (Above)**: Track object above ground with lead-time prediction
   - **Phase 2 (Down)**: Descend to pick height while tracking
   - **Gripper Close**: Synchronize grasp with final object motion
   - **Lift**: Raise object vertically

5. **Object Placement** (`pick_place.py`):
   - Move to safe intermediate pose
   - Navigate to bin "above" pose
   - Lower into bin "down" pose
   - Release gripper and return to pick position

## Installation & Usage

### Setup in Webots

1. Copy the `controllers/final/` folder to your Webots project:
   ```
   controllers/ur5e_sorting_final/
   ```

2. Copy all `.py` files from `controllers/final/` into that folder

3. In the Webots Scene Tree:
   - Select the UR5e robot node
   - Set the `controller` field to: `ur5e_sorting_final`

4. Start the simulation:
   - Objects should be recognized by the camera with `model` and `recognitionColors` defined
   - The robot will begin picking and sorting objects

### Configuration & Tuning

Edit `config.py` to adjust:

#### Timing
- `LEAD_TIME_ABOVE` (0.28s): Prediction ahead of time when tracking to above pose
- `LEAD_TIME_DOWN` (0.14s): Prediction when descending to pick height
- `LEAD_TIME_CLOSE` (0.08s): Prediction during final gripper closure

#### Pick Geometry
- `CUBE_HALF_HEIGHT` (0.025m): Half-size of objects being picked
- `PICK_CLEARANCE` (0.020m): Clearance above object before gripper closes
- `ABOVE_CLEARANCE` (0.160m): Safe clearance above objects during approach
- `GRIPPER_TIP_OFFSET_FLANGE` (0.0, 0.0, 0.120m): TCP offset in flange frame

#### Gripper Orientation
- `GRIPPER_DOWN_AXIS`: Set to `"z"`, `"-z"`, `"x"`, `"-x"`, `"y"`, or `"-y"`
  - Defines the direction the gripper points when over objects

#### Pick Workspace (in robot base frame)
- `PICK_X_MIN` (-1.35m): Minimum X (furthest back on conveyor)
- `PICK_X_MAX` (-0.25m): Maximum X (nearest to base)
- `PICK_X_GOAL` (-0.72m): Target X for best pick position
- `PICK_X_BEST` (-0.38m): Optimal X for scoring candidate objects
- `PICK_Y_LIMIT` (0.80m): Maximum Y (lateral distance)
- `PICK_RADIUS_MAX` (0.98m): Maximum radial distance
- `LOCK_DISTANCE` (0.12m): Distance threshold for re-acquiring same object

#### Tracking Filters
- `CUBE_ALPHA` (0.42): Low-pass filter coefficient for position (0–1)
- `VEL_ALPHA` (0.20): Low-pass filter coefficient for velocity (0–1)
- `LOST_MAX_STEPS` (12): Max steps to extrapolate when object lost
- `MAX_CUBE_SPEED` (0.70 m/s): Clamp maximum velocity
- `MIN_CUBE_AREA` (80 px²): Minimum object size to track

#### Failure Tolerance
- `TOO_LATE_GRACE_STEPS` (18): Steps to tolerate before giving up if object exits pick zone
- `IK_FAIL_GRACE_STEPS` (35): Steps to tolerate IK failures before skipping

#### Testing
- `TEST_ONLY_BIN`: Set to a bin name (e.g., `"RED_CUBE_BIN"`) to pick only that category
  - Set to `None` for full sorting

## Calibration & Customization

### Camera Transform
If the camera position or orientation changes in your Webots scene:
- Recalibrate `T_FLANGE_CAMERA` in `camera_utils.py`
- This matrix defines the transform from gripper flange to camera frame

### Robot Base Transform
If the robot base or pedestal moves in the scene:
- Recalibrate `get_world_base_transform()` in `motion.py`
- This ensures correct mapping from world coordinates to robot base frame

### Bin Poses
Define new bin locations in `poses.py`:
- Add `BIN_NAME_ABOVE` and `BIN_NAME_DOWN` joint-space poses
- Register them in `sorting.py`'s `BIN_POSES` dictionary

### Object Recognition
Ensure your Webots world file correctly defines:
- `model` field: unique string identifier per object type
- `recognitionColors`: array of RGB values for color-based identification
- PROTO files should include these in recognition data

## Important Notes

⚠️ **Do NOT transfer `.pyc` files** to production environments. These are temporary Python cache files generated at runtime.

⚠️ **Sorting depends on recognition data** from Webots. Objects must have proper `model` values and `recognitionColors` in the simulation.

⚠️ **Tuning is scene-dependent**. Lead times, workspace limits, and filter coefficients should be adjusted for your specific conveyor speed, object size, and robot mounting.

## Troubleshooting

| Issue | Diagnosis | Solution |
|-------|-----------|----------|
| Objects not detected | Camera transform misconfigured | Recalibrate `T_FLANGE_CAMERA` |
| Picking wrong bin | Object classification issue | Verify `model` and `recognitionColors` in Webots |
| Gripper misaligned | Orientation incorrect | Adjust `GRIPPER_DOWN_AXIS` in `config.py` |
| Missing moving objects | Lead time too small | Increase `LEAD_TIME_ABOVE` / `LEAD_TIME_DOWN` |
| IK fails frequently | Workspace too tight | Increase `MAX_JOINT_STEP` or adjust workspace bounds |
