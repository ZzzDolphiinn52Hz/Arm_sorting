"""Sorting-bin mapping and bin-selection helpers."""

import config
import poses

BIN_POSES = {
    "BLUE_CUBE_BIN": (
        poses.BLUE_CUBE_BIN_ABOVE,
        poses.BLUE_CUBE_BIN_DOWN,
    ),
    "RED_CUBE_BIN": (
        poses.RED_CUBE_BIN_ABOVE,
        poses.RED_CUBE_BIN_DOWN,
    ),
    "YELLOW_CYLINDER_BIN": (
        poses.YELLOW_CYLINDER_BIN_ABOVE,
        poses.YELLOW_CYLINDER_BIN_DOWN,
    ),
    "GREEN_SPHERE_BIN": (
        poses.GREEN_SPHERE_BIN_ABOVE,
        poses.GREEN_SPHERE_BIN_DOWN,
    ),
    "UNKNOWN_BIN": (
        poses.BIN_ABOVE,
        poses.BIN_DOWN,
    ),
}


def normalize_bin_name(bin_name):
    """Return a known bin name, falling back to UNKNOWN_BIN."""
    if bin_name not in BIN_POSES:
        print(f"[SORT WARNING] Unknown bin '{bin_name}', using UNKNOWN_BIN")
        return "UNKNOWN_BIN"
    return bin_name


def get_bin_pose(bin_name):
    """Return (normalized_bin_name, (above_pose, down_pose))."""
    normalized = normalize_bin_name(bin_name)
    return normalized, BIN_POSES[normalized]


def is_allowed_bin(bin_name):
    """Filter helper for single-bin testing."""
    if config.TEST_ONLY_BIN is None:
        return True
    return bin_name == config.TEST_ONLY_BIN
