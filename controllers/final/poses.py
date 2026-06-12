# poses.py
# Pre-defined joint angle configurations for the UR5e arm.
# All values in radians: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]

"""
HOME       = [ 0.000, -1.570,  1.570, -1.570, -1.570,  0.000]
PICK_ABOVE = [-0.000, -1.210,  1.370, -1.770, -1.570, -0.000]
PICK_DOWN  = [-0.000, -1.010,  1.590, -2.170, -1.570, -0.020]
BIN_ABOVE  = [ 3.060, -1.550,  1.050, -1.050, -1.570, -0.080]
BIN_DOWN   = [ 3.060, -1.550,  1.450, -1.450, -1.570, -0.080]
SAFE_MID   = [ 2.410, -1.570,  1.150, -1.210, -1.570,  0.000]
"""
# =========================
# SORTING BIN POSES
# All values in radians:
# [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
# =========================

# Thùng cho cube màu xanh dương
BLUE_CUBE_BIN_ABOVE = [ 2.540, -1.550,  1.050, -1.050, -1.570, -0.080]
BLUE_CUBE_BIN_DOWN  = [ 2.540, -1.550,  1.450, -1.450, -1.570, -0.080]

# Thùng cho cube màu đỏ
RED_CUBE_BIN_ABOVE  = [ 2.850, -1.550,  1.050, -1.050, -1.570, -0.080]
RED_CUBE_BIN_DOWN   = [ 2.850, -1.550,  1.450, -1.450, -1.570, -0.080]

# Thùng cho cylinder màu vàng
YELLOW_CYLINDER_BIN_ABOVE = [ 3.150, -1.550,  1.050, -1.050, -1.570, -0.080]
YELLOW_CYLINDER_BIN_DOWN  = [ 3.150, -1.550,  1.450, -1.450, -1.570, -0.080]

# Thùng cho sphere màu xanh lá
GREEN_SPHERE_BIN_ABOVE = [ 3.480, -1.550,  1.050, -1.050, -1.570, -0.080]
GREEN_SPHERE_BIN_DOWN  = [ 3.480, -1.550,  1.450, -1.450, -1.570, -0.080]

HOME      = [ 0.000, -1.570,  1.570, -1.570, -1.570,  0.000]
PICK_ABOVE = [-0.000, -1.210,  1.370, -1.770, -1.570, -0.000]
PICK_DOWN  = [-0.000, -1.010,  1.590, -2.170, -1.570, -0.020]
BIN_ABOVE = [ 3.060, -1.550, 1.050, -1.050, -1.570, -0.080]
SAFE_MID   = [ 2.410, -1.570,  1.150, -1.210, -1.570,  0.000]
BIN_DOWN  = [ 3.060, -1.550, 1.450, -1.450, -1.570, -0.080]
