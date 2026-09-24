import cv2
import numpy as np


ARUCO_SIZE_CM = 2
CUBE_SIZE_CM = 3
CUBE_DISTANCE_CM = 6
ROI_MARGIN_CM = 0.5


def calculate_cube_roi(marker_center, marker_size_px):

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

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    color_ranges = {

        "green": (
            np.array([40, 70, 50]),
            np.array([95, 255, 255])
        ),

        "blue": (
            np.array([99, 219, 101]),
            np.array([115, 255, 255])
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

    shelf_state = []

    for position in range(1, 9):

        shelf_state.append({
            "position": position,
            "occupied": False,
            "color": None
        })

    return shelf_state