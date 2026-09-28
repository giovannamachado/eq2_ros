"""Synthetic-shelf test of the ArUco + cube color pipeline (marker/detector)."""

import cv2
import cv2.aruco as aruco
import numpy as np

from fs1.detector import (
    calculate_cube_roi,
    create_shelf_state,
    detect_cube_color,
)
from fs1.marker import create_detector, detect_markers

MARKER_PX = 100  # ArUco side; detector assumes 2 cm -> 50 px/cm
CUBE_BGR = {
    'green': (0, 200, 0),
    'blue': (255, 0, 0),
}


def draw_marker(marker_id):
    """Return a MARKER_PX x MARKER_PX grayscale ArUco image."""
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, 'generateImageMarker'):
        return aruco.generateImageMarker(dictionary, marker_id, MARKER_PX)
    return aruco.drawMarker(dictionary, marker_id, MARKER_PX)


def make_shelf(cubes):
    """
    Build a frame with markers 0..7 (2 rows x 4) and colored cubes.

    ``cubes`` maps marker id -> color name; each cube is drawn 6 cm above
    its marker, as ``calculate_cube_roi`` expects.
    """
    frame = np.full((900, 1000, 3), 90, dtype=np.uint8)  # dark gray shelf
    for marker_id in range(8):
        row, col = divmod(marker_id, 4)
        cx, cy = 130 + col * 230, 420 + row * 400
        tile = draw_marker(marker_id)
        frame[cy - 50:cy + 50, cx - 50:cx + 50] = cv2.cvtColor(
            tile, cv2.COLOR_GRAY2BGR)
        if marker_id in cubes:
            top = cy - 300
            frame[top - 75:top + 75, cx - 75:cx + 75] = \
                CUBE_BGR[cubes[marker_id]]
    return frame


def classify(frame):
    """Run the same per-marker steps as ``VisionNode.process_frame``."""
    detector = create_detector()
    shelf_state = create_shelf_state()
    for info in detect_markers(detector, frame).values():
        _, (x0, y0, x1, y1) = calculate_cube_roi(
            info['center'], info['size_px'])
        roi = frame[max(y0, 0):y1, max(x0, 0):x1]
        if roi.size == 0:
            continue
        color, _ = detect_cube_color(roi)
        if color is not None:
            shelf_state[info['position'] - 1]['occupied'] = True
            shelf_state[info['position'] - 1]['color'] = color
    return shelf_state


def test_detects_all_eight_markers_and_maps_ids_to_positions():
    markers = detect_markers(create_detector(), make_shelf({}))

    assert sorted(markers) == list(range(8))
    assert {i: m['position'] for i, m in markers.items()} == {
        i: i + 1 for i in range(8)}
    assert all(abs(m['size_px'] - MARKER_PX) < 5 for m in markers.values())


def test_shelf_state_reports_occupied_slots_with_color():
    state = classify(make_shelf({1: 'green', 4: 'blue'}))

    occupied = {s['position']: s['color'] for s in state if s['occupied']}
    assert occupied == {2: 'green', 5: 'blue'}


def test_empty_shelf_has_no_occupied_slots():
    state = classify(make_shelf({}))

    assert not any(s['occupied'] for s in state)
    assert len(state) == 8


def test_no_markers_in_blank_frame():
    blank = np.full((480, 640, 3), 255, dtype=np.uint8)

    assert detect_markers(create_detector(), blank) == {}


def test_markers_outside_the_shelf_are_ignored():
    frame = make_shelf({})
    stray = cv2.cvtColor(draw_marker(20), cv2.COLOR_GRAY2BGR)
    frame[20:120, 850:950] = stray

    markers = detect_markers(create_detector(), frame)

    assert 20 not in markers
    assert sorted(markers) == list(range(8))
    assert all(m['position'] is not None for m in markers.values())
    classify(frame)  # must not raise (used to crash with position=None)
