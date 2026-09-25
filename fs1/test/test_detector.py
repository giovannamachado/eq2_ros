import cv2
import numpy as np

from fs1.detector import (
    calculate_cube_roi,
    detect_cube_color,
    create_shelf_state
)


def test_create_shelf_state():

    shelf_state = create_shelf_state()

    assert len(shelf_state) == 8

    for position, slot in enumerate(shelf_state, start=1):

        assert slot["position"] == position
        assert slot["occupied"] is False
        assert slot["color"] is None


def test_calculate_cube_roi():

    marker_center = (100, 200)
    marker_size_px = 20

    cube_center, roi = calculate_cube_roi(
        marker_center,
        marker_size_px
    )

    assert cube_center == (100, 140)

    x_min, y_min, x_max, y_max = roi

    assert x_min == 80
    assert y_min == 120
    assert x_max == 120
    assert y_max == 160


def test_detect_green_cube():

    roi = np.zeros(
        (100, 100, 3),
        dtype=np.uint8
    )

    roi[:] = (0, 255, 0)

    color, percentages = detect_cube_color(roi)

    assert color == "green"
    assert percentages["green"] > 15


def test_detect_blue_cube():

    roi = np.zeros(
        (100, 100, 3),
        dtype=np.uint8
    )

    roi[:] = (255, 0, 0)

    color, percentages = detect_cube_color(roi)

    assert color == "blue"
    assert percentages["blue"] > 15


def test_detect_purple_cube():

    hsv = np.zeros(
        (100, 100, 3),
        dtype=np.uint8
    )

    hsv[:] = (120, 180, 200)

    roi = cv2.cvtColor(
        hsv,
        cv2.COLOR_HSV2BGR
    )

    color, percentages = detect_cube_color(roi)

    assert color == "purple"
    assert percentages["purple"] > 15


def test_detect_white_cube():

    roi = np.zeros(
        (100, 100, 3),
        dtype=np.uint8
    )

    roi[:] = (255, 255, 255)

    color, percentages = detect_cube_color(roi)

    assert color == "white"
    assert percentages["white"] > 15


def test_detect_no_cube():

    roi = np.zeros(
        (100, 100, 3),
        dtype=np.uint8
    )

    roi[:] = (0, 0, 0)

    color, percentages = detect_cube_color(roi)

    assert color is None