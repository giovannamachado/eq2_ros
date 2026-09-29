"""ArUco marker detection for the shelf slots (ids 0..7 -> slots 1..8)."""

import cv2
import cv2.aruco as aruco
import numpy as np


def create_detector():
    """
    Create the ArUco detector (DICT_4X4_50).

    OpenCV >= 4.7 exposes ``ArucoDetector``; older versions (4.5.x, as in the
    kortex_humble image) only have the legacy functions. In that case a
    ``(dictionary, parameters)`` tuple is returned and ``detect_markers``
    uses the legacy call, so both versions produce the same result.
    """
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    if hasattr(aruco, "ArucoDetector"):
        parameters = aruco.DetectorParameters()
        return aruco.ArucoDetector(dictionary, parameters)

    return dictionary, aruco.DetectorParameters_create()


def detect_markers(detector, frame):
    """
    Find the shelf's ArUco markers in a frame and map each to its slot.

    Args:
        detector: value returned by ``create_detector()``.
        frame: BGR image to scan.

    Returns:
        dict keyed by marker id, each value ``{"corners", "position",
        "center", "size_px"}``. Ids outside 0..7 (not a shelf slot) are
        left out.
    """
    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    if isinstance(detector, tuple):
        dictionary, parameters = detector
        corners, ids, rejected = aruco.detectMarkers(
            gray, dictionary, parameters=parameters)
    else:
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

            # Ids outside the shelf (0..7) are not slots: ignore them
            # instead of letting the vision node crash on position=None.
            if slot is None:
                continue

            markers[marker_id] = {
                "corners": marker_corners,
                "position": slot,
                "center": center,
                "size_px": marker_width_px
            }

    return markers
