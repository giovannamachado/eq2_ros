import cv2
import numpy as np

from markers.marker import create_detector, detect_markers


ARUCO_SIZE_CM = 2
CUBE_SIZE_CM = 3
CUBE_DISTANCE_CM = 6
ROI_MARGIN_CM = 0.5


def calculate_cube_roi(marker_center, marker_size_px):
    pixels_per_cm = marker_size_px / ARUCO_SIZE_CM

    distance_px = pixels_per_cm * CUBE_DISTANCE_CM

    cube_center = (
        marker_center[0],
        int(marker_center[1] - distance_px)
    )

    roi_size_cm = (
        CUBE_SIZE_CM +
        (ROI_MARGIN_CM * 2)
    )

    roi_size_px = int(
        pixels_per_cm * roi_size_cm
    )

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


def main():
    camera = cv2.VideoCapture(0)

    detector = create_detector()

    while True:
        ret, frame = camera.read()

        if not ret:
            print("Erro ao capturar imagem da câmera.")
            break

        markers = detect_markers(
            detector,
            frame
        )

        for marker_id, info in markers.items():

            marker_center = info["center"]
            corners = info["corners"]

            x_min = np.min(
                corners[:, 0]
            )

            x_max = np.max(
                corners[:, 0]
            )

            marker_size_px = x_max - x_min

            cube_center, roi = calculate_cube_roi(
                marker_center,
                marker_size_px
            )

            x_min_roi, y_min_roi, x_max_roi, y_max_roi = roi

            print(
                f"ID: {marker_id} | "
                f"ArUco: {marker_center} | "
                f"Cubo: {cube_center}"
            )

            cv2.circle(
                frame,
                marker_center,
                6,
                (0, 255, 0),
                -1
            )

            cv2.circle(
                frame,
                cube_center,
                8,
                (255, 0, 255),
                -1
            )

            cv2.rectangle(
                frame,
                (x_min_roi, y_min_roi),
                (x_max_roi, y_max_roi),
                (255, 255, 0),
                2
            )

        cv2.imshow(
            "Cube Detector",
            frame
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()