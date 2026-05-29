# Webots Camera Recognition Integration Guide
**Project:** Arm Sorting (UR5e + Conveyor Belt)  
**Date:** 2026-05-29  

---

## 1. Project Structure Overview

```
Arm_sorting/
├── worlds/
│   └── ure.wbt                        (main world file)
├── controllers/
│   ├── test/
│   │   └── test.py                    (main UR5e controller — active)
│   ├── ur5e/
│   │   └── ur5e.py                    (manual keyboard controller)
│   ├── tuning/
│   │   └── tuning.py
│   └── conveyor_belt/
│       ├── conveyor_belt.c            (C controller for belt speed/timer)
│       ├── conveyor_belt              (compiled binary)
│       └── Makefile
└── guide.md                           (this file)
```

**Active controller assigned in ure.wbt:** `test` (test.py)

---

## 2. Reference Sample Project

Sample project location: `/home/devops/Downloads/test/`

```
test/
├── worlds/
│   └── camera_recognition.wbt
└── controllers/
    └── camera_recognition/
        ├── camera_recognition.py      (Python implementation)
        ├── camera_recognition.c       (C implementation — same logic as .py)
        └── Makefile
```

`camera_recognition.py` and `camera_recognition.c` implement the exact same logic,
just in different languages. Both do:
- Enable camera + recognition
- Each step: get detected objects, print model, id, position, orientation, size, colors

This sample demonstrates Webots' built-in camera recognition API using:
- `webots/camera.h`
- `webots/camera_recognition_object.h`

---

## 3. camera_recognition.py — Detailed Explanation

### Overview

The script is a class-based Webots controller that:
1. Sets up a camera with recognition enabled
2. Drives the robot's wheels to spin in place
3. Every simulation step, scans for objects in view and prints all their data

### Class Structure

```python
from controller import Robot

class Controller(Robot):
    def __init__(self):
        ...
    def run(self):
        ...

controller = Controller()
controller.run()
```

Subclasses `Robot` from Webots' `controller` library. The bottom two lines are the
entry point — instantiate and run. This is the standard Webots Python pattern.

---

### __init__ — Setup Phase

```python
self.timeStep = 64
```
Simulation time step in milliseconds. Every `step()` call advances the sim by 64ms.
Must be a multiple of the world's `basicTimeStep` (your project uses 8, so 64 is
valid — 8x multiple).

```python
self.camera = self.getDevice('camera')
self.camera.enable(self.timeStep)
self.camera.recognitionEnable(self.timeStep)
```
- `getDevice('camera')` — fetches the camera device by name, must match the name in the .wbt file
- `enable(timeStep)` — activates the camera, starts capturing images every 64ms
- `recognitionEnable(timeStep)` — activates the recognition engine on top of the camera

Both `enable()` and `recognitionEnable()` are required. Calling `enable()` alone does
NOT activate recognition.

```python
self.left_motor.setPosition(float('inf'))
self.right_motor.setPosition(float('inf'))
self.left_motor.setVelocity(-1.5)
self.right_motor.setVelocity(1.5)
```
- `setPosition(float('inf'))` — switches motor from position control to velocity
  control mode. Required before calling `setVelocity()`
- Left wheel -1.5, right wheel +1.5 — opposite directions so the robot spins in place
  to scan surroundings

---

### run() — Main Loop

```python
while self.step(self.timeStep) != -1:
```
Standard Webots main loop. `step()` advances the simulation by one timeStep.
Returns -1 when the simulation is stopped or reset — that is the exit condition.

```python
number_of_objects = self.camera.getRecognitionNumberOfObjects()
```
Returns an `int` — how many objects the camera currently sees. Objects must have
`recognitionColors` set in the .wbt to be counted.

```python
objects = self.camera.getRecognitionObjects()
```
Returns a list of `CameraRecognitionObject` instances — one per detected object.
If nothing is detected, returns an empty list.

---

### Per-Object API — All Methods You Can Call

```python
obj.getModel()
```
Returns a `string` — the model name of the detected object. Comes from the `model`
field of the Solid node in the .wbt. Useful for identifying object types
(e.g. `"CUBE_1"`, `"can"`, `"box"`).

```python
obj.getId()
```
Returns an `int` — a unique ID assigned by Webots to each solid in the scene.
Stays consistent per object across steps, so you can track the same object over time.

```python
obj.getPosition()
```
Returns `[x, y, z]` as floats — the 3D position of the object relative to the camera.
Units are meters. Most useful for robotics — you can use these coordinates directly
to calculate where the arm needs to move.

```python
obj.getOrientation()
```
Returns `[x, y, z, angle]` — axis-angle representation of the object's orientation
relative to the camera. Useful if you need to know which way the object is rotated
(e.g. for precise grasping alignment).

```python
obj.getSize()
```
Returns `[width, height]` in meters — the real-world physical size of the object
in 3D space. Not pixel size — actual meters.

```python
obj.getPositionOnImage()
```
Returns `[x, y]` in pixels — where the center of the object appears on the camera
image. Useful for image-space logic like checking if the object is centered in the
frame (which is exactly what `wait_until_cube_ready()` does in test.py).

```python
obj.getSizeOnImage()
```
Returns `[width, height]` in pixels — how large the object appears on the camera
image. Larger value means object is closer to the camera. Useful as a rough distance
estimate or for area-based detection thresholds.

```python
obj.getNumberOfColors()
```
Returns an `int` — how many colors are registered for this object from
`recognitionColors` in the .wbt. An object can have multiple colors listed.

```python
obj.getColors()
```
Returns a flat list of floats — RGB values packed together. For N colors the list
has `3*N` elements. Read like this:

```python
colors = obj.getColors()
n = obj.getNumberOfColors()
for j in range(n):
    r = colors[3 * j]
    g = colors[3 * j + 1]
    b = colors[3 * j + 2]
```
Each R, G, B value is between 0.0 and 1.0.

---

### Full API Summary Table

| Method                      | Return type          | Description                                         |
|-----------------------------|----------------------|-----------------------------------------------------|
| `getModel()`                | string               | Object model name from .wbt                         |
| `getId()`                   | int                  | Unique object ID, stable across steps               |
| `getPosition()`             | [x, y, z] floats     | 3D position relative to camera (meters)             |
| `getOrientation()`          | [x, y, z, angle]     | Axis-angle orientation relative to camera           |
| `getSize()`                 | [w, h] floats        | Real-world size in meters                           |
| `getPositionOnImage()`      | [x, y] ints          | Pixel position on camera image                      |
| `getSizeOnImage()`          | [w, h] ints          | Pixel size on camera image                          |
| `getNumberOfColors()`       | int                  | Number of registered colors                         |
| `getColors()`               | flat float list      | RGB values (0.0-1.0), packed as [r,g,b,r,g,b,...]   |

---

### Bug in Original Sample (line 54)

```python
# Wrong — inconsistent attribute access
print(f' Orientation: {object.orientation[0]} ...')

# Correct — use the variable returned by getOrientation()
orientation = obj.getOrientation()
print(f' Orientation: {orientation[0]} {orientation[1]} {orientation[2]} {orientation[3]}')
```
The original uses `object.orientation[0]` (direct attribute) instead of the
`orientation` variable defined two lines above. This may throw an `AttributeError`
depending on Webots version. Always use the getter methods.

---

## 4. Webots Camera Recognition API

### Python API (used in this project)

```python
from controller import Robot

# Setup
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)   # must call this too

# In main loop
objects = camera.getRecognitionObjects()
count   = camera.getRecognitionNumberOfObjects()

for obj in objects:
    obj.getModel()              # object model name (string)
    obj.getId()                 # unique object ID (int)
    obj.getPosition()           # [x, y, z] relative to robot (3D)
    obj.getOrientation()        # [x, y, z, angle] quaternion
    obj.getSize()               # [width, height] in meters
    obj.getPositionOnImage()    # [x, y] in pixels on camera image
    obj.getSizeOnImage()        # [w, h] in pixels on camera image
    obj.getColors()             # flat RGB list
    obj.getNumberOfColors()     # int
```

### C API (for reference)

```c
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>

WbDeviceTag camera = wb_robot_get_device("camera");
wb_camera_enable(camera, TIME_STEP);
wb_camera_recognition_enable(camera, TIME_STEP);

int n = wb_camera_recognition_get_number_of_objects(camera);
const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
```

---

## 4. Problems Encountered

### Problem 1 — Recognition node stripped on reload

**Symptom:**
Adding `recognition Recognition {}` inside `JetBotRaspberryPiCamera` in the .wbt file
was ignored/stripped every time Webots reloaded the world.

**Root Cause:**
`JetBotRaspberryPiCamera` is a PROTO node. PROTO nodes only accept fields declared
in their own `.proto` file interface. The `recognition` field is not declared in
JetBotRaspberryPiCamera's PROTO, so Webots silently ignores it on load.

**Fix:**
Replace `JetBotRaspberryPiCamera` with a plain `Camera` node which supports all
standard Camera fields including `recognition`.

---

### Problem 2 — Camera blurry after switching to plain Camera node

**Symptom:**
After replacing JetBotRaspberryPiCamera with a plain Camera node, the camera view
became blurry/low quality.

**Root Cause:**
Plain `Camera` nodes default to 64x64 resolution if `width` and `height` are not
explicitly set. The original JetBotRaspberryPiCamera had 1280x720 built into its PROTO.

**Fix:**
Add `width 1280` and `height 720` fields to the Camera node.

---

### Problem 3 — Console errors at runtime

**Symptom:**
```
Error: wb_camera_recognition_enable() called on a Camera without Recognition node.
Error: wb_camera_recognition_get_objects() called on a Camera without Recognition node.
```

**Root Cause:**
Same as Problem 1 — the Recognition node was not actually applied because
JetBotRaspberryPiCamera PROTO does not support it.

**Fix:** Same as Problem 1 — switch to plain Camera node.

---

## 5. All Changes Made

### 5.1 worlds/ure.wbt

#### Change A — Removed unused EXTERNPROTO

Removed:
```
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/nvidia/jetbot/protos/JetBotRaspberryPiCamera.proto"
```

#### Change B — Replaced JetBotRaspberryPiCamera with plain Camera

Before:
```
JetBotRaspberryPiCamera {
  translation 0.01 0.1 0.02
  rotation 0 0 1 1.5708003061004252
  fieldOfView 2.2
  far 2
}
```

After:
```
Camera {
  translation 0.01 0.1 0.02
  rotation 0 0 1 1.5708003061004252
  name "camera"
  width 1280
  height 720
  fieldOfView 2.2
  far 2
  recognition Recognition {
  }
}
```

Key additions:
- `name "camera"` — must match `getDevice('camera')` in test.py
- `width 1280` and `height 720` — restores original resolution
- `recognition Recognition {}` — enables the recognition engine

#### Change C — Added recognitionColors to CUBE_1 solids

Added to both `CUBE_1` and `CUBE_1(1)` Solid nodes:
```
recognitionColors [
  1 1 1
]
```
This registers the cubes as white objects detectable by the recognition engine.
Without this field, `getRecognitionObjects()` returns empty even with recognition enabled.

---

### 5.2 controllers/test/test.py

#### Change A — Enable recognition on camera

Added after `camera.enable(TIME_STEP)`:
```python
camera.recognitionEnable(TIME_STEP)
```

#### Change B — Replaced detect_white_cube() with detect_cube()

Old approach (manual pixel scanning):
```python
def detect_white_cube():
    image = camera.getImage()
    # ... loop every pixel, check RGB threshold manually
```

New approach (recognition API):
```python
def detect_cube():
    objects = camera.getRecognitionObjects()
    if len(objects) == 0:
        return None

    obj = objects[0]
    pos_on_image = obj.getPositionOnImage()
    size_on_image = obj.getSizeOnImage()
    position_3d  = obj.getPosition()
    model_name   = obj.getModel()

    return {
        "cx": pos_on_image[0],
        "cy": pos_on_image[1],
        "area": size_on_image[0] * size_on_image[1],
        "position_3d": position_3d,
        "model": model_name
    }
```

#### Change C — Updated wait_until_cube_ready() to use detect_cube()

```python
def wait_until_cube_ready(max_steps=500):
    print("Waiting for cube to enter pick zone...")

    image_center_x = camera.getWidth() / 2
    image_center_y = camera.getHeight() / 2
    PICK_ZONE_X = 120
    PICK_ZONE_Y = 120
    MIN_CUBE_AREA = 150

    for i in range(max_steps):
        result = detect_cube()

        if result is not None:
            cx, cy, area = result["cx"], result["cy"], result["area"]
            error_x = abs(cx - image_center_x)
            error_y = abs(cy - image_center_y)

            if i % 10 == 0:
                print(f"Cube: model={result['model']} cx={round(cx,1)} cy={round(cy,1)} area={area}")

            if error_x < PICK_ZONE_X and error_y < PICK_ZONE_Y and area > MIN_CUBE_AREA:
                print("Cube is ready to pick!")
                return True
        else:
            if i % 20 == 0:
                print("No cube detected")

        if robot.step(TIME_STEP) == -1:
            return False

    print("Timeout: cube did not enter pick zone")
    return False
```

---

## 6. Rules for Editing .wbt Files

Always follow these rules to avoid Webots overwriting your changes:

1. **Close Webots completely** before editing any .wbt file
2. Make your edits while Webots is closed
3. Open Webots after editing — it will load the new state cleanly
4. If you edit while Webots is open, it will overwrite your file when it auto-saves

---

## 7. Recognition API Tips

**recognitionColors is mandatory on objects:**
Even with `recognition Recognition {}` on the camera, objects will NOT be detected
unless they have a `recognitionColors` field in their Solid node.

**Recognition does not depend on image resolution:**
The recognition engine works at simulation geometry level, not pixel level.
You can use lower resolution (e.g. 640x480) and recognition still works perfectly.
Only use high resolution if you also need raw pixel image quality.

**Useful resolution options:**

| Resolution  | Use case                              |
|-------------|---------------------------------------|
| 1280 x 720  | High quality image display            |
| 640 x 480   | Balanced quality + performance        |
| 320 x 240   | Lightweight, recognition-only use     |

**getPosition() returns 3D coords:**
`obj.getPosition()` gives [x, y, z] of the object relative to the robot/camera.
This can be used directly for more precise arm targeting instead of hardcoded poses.

**Filter by model name:**
If you have multiple object types, filter by model:
```python
for obj in camera.getRecognitionObjects():
    if obj.getModel() == "CUBE_1":
        # handle this object type
```

---

## 8. Applying This Pattern to Other Webots Projects

Checklist for any new project:

- [ ] Camera node in .wbt must be a plain `Camera` node (not a PROTO)
- [ ] Add `recognition Recognition {}` inside the Camera node
- [ ] Add `recognitionColors [r g b]` to every Solid you want detected
- [ ] Set `width` and `height` explicitly on the Camera node
- [ ] In controller: call both `camera.enable()` AND `camera.recognitionEnable()`
- [ ] Edit .wbt only while Webots is fully closed
- [ ] Camera `name` in .wbt must match `getDevice('camera')` in your controller
