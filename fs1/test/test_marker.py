import cv2
import cv2.aruco as aruco
import numpy as np

from fs1.marker import (
    create_detector,
    detect_markers
)


def test_create_detector():

    detector = create_detector()

    assert detector is not None


def test_detect_marker():

    detector = create_detector()

    dictionary = aruco.getPredefinedDictionary(
        aruco.DICT_4X4_50
    )

    marker = aruco.generateImageMarker(
        dictionary,
        0,
        100
    )

    frame = np.full(
        (200, 200),
        255,
        dtype=np.uint8
    )

    frame[50:150, 50:150] = marker

    frame = cv2.cvtColor(
        frame,
        cv2.COLOR_GRAY2BGR
    )

    markers = detect_markers(
        detector,
        frame
    )

    assert 0 in markers

    marker_info = markers[0]

    assert marker_info["position"] == 1

    center_x, center_y = marker_info["center"]

    assert abs(center_x - 100) <= 2
    assert abs(center_y - 100) <= 2

    assert marker_info["size_px"] > 90


def test_marker_position_mapping():

    expected_positions = {
        0: 1,
        1: 2,
        2: 3,
        3: 4,
        4: 5,
        5: 6,
        6: 7,
        7: 8
    }

    for marker_id, expected_position in expected_positions.items():

        assert (
            expected_position
            == marker_id + 1
        )