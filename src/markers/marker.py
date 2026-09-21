import cv2
import cv2.aruco as aruco
import numpy as np


ARUCO_SIZE_CM = 2
CUBE_SIZE_CM = 3
CUBE_DISTANCE_CM = 6
ROI_MARGIN_CM = 0.5


def create_detector():

    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    parameters = aruco.DetectorParameters()

    detector = aruco.ArucoDetector(dictionary, parameters)

    return detector


def detect_markers(detector, frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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

            center_x = np.mean(marker_corners[:, 0])

            center_y = np.mean(marker_corners[:, 1])

            center = (int(center_x), int(center_y))

            slot = marker_to_slot.get(marker_id)

            markers[marker_id] = {
                "corners": marker_corners,
                "position": slot,
                "center": center
            }

    return markers


def main():

    camera = cv2.VideoCapture(0)

    detector = create_detector()

    while True:

        ret, frame = camera.read()

        if not ret:
            print("Erro ao capturar imagem da câmera.")
            break

        markers = detect_markers(detector, frame)

        for marker_id, info in markers.items():

            center = info["center"]

            corners = info["corners"]

            x_min = np.min(corners[:, 0])

            x_max = np.max(corners[:, 0])

            marker_width_px = (x_max - x_min)

            pixels_per_cm = (marker_width_px / ARUCO_SIZE_CM)

            distance_px = (pixels_per_cm * CUBE_DISTANCE_CM)

            cube_center = (center[0], int(center[1] - distance_px))

            roi_size_cm = (CUBE_SIZE_CM +(ROI_MARGIN_CM * 2))

            roi_size_px = int(pixels_per_cm * roi_size_cm)

            half_roi = roi_size_px // 2

            x_min_roi = (cube_center[0] - half_roi)

            x_max_roi = (cube_center[0] + half_roi)

            y_min_roi = (cube_center[1] - half_roi)

            y_max_roi = (cube_center[1] + half_roi)

            print(
                f"ID: {marker_id} | "
                f"ArUco: {center} | "
                f"Cubo: {cube_center}"
            )

            aruco.drawDetectedMarkers(
                frame,
                [corners.reshape(1, 4, 2)],
                np.array([[marker_id]])
            )

            cv2.circle( frame, center, 6, (0, 255, 0), -1)

            cv2.circle(frame, cube_center, 8, (255, 0, 255), -1)

            cv2.rectangle( frame, (x_min_roi, y_min_roi), (x_max_roi, y_max_roi), (255, 255, 0), 2)

        cv2.imshow("ArUco", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    camera.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()