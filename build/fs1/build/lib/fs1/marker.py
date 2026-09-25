import cv2
import cv2.aruco as aruco
import numpy as np


def create_detector():
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)

    return detector


def detect_markers(detector, frame):

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    corners, ids, rejected = detector.detectMarkers(gray)

    marker_to_slot = {
        0: 1,
        1: 2,
        2: 3,
        3: 4,
        4: 5,
        5: 6,
        6: 7,
        7: 8
    }

    markers = {}

    if ids is not None:

        for i, marker_id in enumerate(ids.flatten()):

            marker_id = int(marker_id)

            marker_corners = corners[i][0]

            center_x = np.mean(
                marker_corners[:, 0]
            )

            center_y = np.mean(
                marker_corners[:, 1]
            )

            center = (
                int(center_x),
                int(center_y)
            )

            x_min = np.min(
                marker_corners[:, 0]
            )

            x_max = np.max(
                marker_corners[:, 0]
            )

            marker_width_px = (
                x_max - x_min
            )

            slot = marker_to_slot.get(marker_id)

            markers[marker_id] = {
                "corners": marker_corners,
                "position": slot,
                "center": center,
                "size_px": marker_width_px
            }

    return markers