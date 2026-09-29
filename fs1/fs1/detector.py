"""
Cube ROI geometry and color classification for the shelf slots.

Given a detected ArUco marker (see ``fs1.marker``), this module locates the
cube that sits above it in the camera image and classifies its color, so
``fs1.vision_node`` can report which shelf slots are occupied and with what.
"""

import cv2
import numpy as np


ARUCO_SIZE_CM = 2
CUBE_SIZE_CM = 3
CUBE_DISTANCE_CM = 6
ROI_MARGIN_CM = 0.5


def calculate_cube_roi(marker_center, marker_size_px):
    """
    Find the pixel region where the cube above a marker should be.

    Uses the marker's own pixel size to convert the known real-world
    distance and cube size (``CUBE_DISTANCE_CM``, ``CUBE_SIZE_CM``) into
    pixels, so it works at any camera distance/zoom.

    Args:
        marker_center: ``(x, y)`` pixel coordinates of the marker's center.
        marker_size_px: the marker's width in pixels, as detected.

    Returns:
        A tuple ``(cube_center, (x_min, y_min, x_max, y_max))``.
    """
    pixels_per_cm = (marker_size_px / ARUCO_SIZE_CM)

    distance_px = (pixels_per_cm * CUBE_DISTANCE_CM)

    cube_center = (marker_center[0], int(marker_center[1] - distance_px)
    )

    roi_size_cm = (CUBE_SIZE_CM + (ROI_MARGIN_CM * 2))

    roi_size_px = int(pixels_per_cm * roi_size_cm)

    half_roi = roi_size_px // 2

    x_min = cube_center[0] - half_roi
    x_max = cube_center[0] + half_roi

    y_min = cube_center[1] - half_roi
    y_max = cube_center[1] + half_roi

    return cube_center, (
        x_min,
        y_min,
        x_max,
        y_max
    )


def detect_cube_color(roi):
    """
    Classify the dominant piece color inside a cube's ROI image.

    Converts to HSV and measures how much of the ROI falls inside each
    known color range (green/blue/purple/white). The color with the
    highest percentage wins; if even the best match covers less than 15%
    of the ROI, the slot is treated as empty (``None``).

    Args:
        roi: BGR image crop of the cube's region (from
            ``calculate_cube_roi``).

    Returns:
        A tuple ``(color_name_or_None, percentages_by_color)``.
    """
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    color_ranges = {

        "green": (
            np.array([40, 70, 50]),
            np.array([95, 255, 255])
        ),

        "blue": (
            np.array([107, 219, 91]),
            np.array([142, 255, 255])
        ),

        "purple": (
            np.array([115, 120, 0]),
            np.array([126, 218, 255])
        ),

        "white": (
            np.array([0, 0, 150]),
            np.array([179, 70, 255])
        )
    }

    total_pixels = (roi.shape[0] * roi.shape[1])

    color_percentages = {}

    for color, (lower, upper) in color_ranges.items():

        mask = cv2.inRange(hsv, lower, upper)

        pixels_detected = cv2.countNonZero(mask)

        percentage = (pixels_detected / total_pixels) * 100

        color_percentages[color] = percentage

    detected_color = max(color_percentages, key=color_percentages.get)

    if color_percentages[detected_color] < 15:

        return None, color_percentages

    return detected_color, color_percentages


def create_shelf_state():
    """Build the 8-slot shelf state, all empty (``occupied: False``)."""
    shelf_state = []


    for position in range(1, 9):

        shelf_state.append({
            "position": position,
            "occupied": False,
            "color": None
        })

    return shelf_state