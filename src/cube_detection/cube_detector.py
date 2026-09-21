import cv2
import numpy as np

from src.markers.marker import create_detector, detect_markers


ARUCO_SIZE_CM = 2
CUBE_SIZE_CM = 3
CUBE_DISTANCE_CM = 6
ROI_MARGIN_CM = 0.5


def calculate_cube_roi(marker_center, marker_size_px):

    pixels_per_cm = (
        marker_size_px / ARUCO_SIZE_CM
    )

    distance_px = (
        pixels_per_cm * CUBE_DISTANCE_CM
    )

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


def detect_cube_color(roi):

    hsv = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2HSV
    )

    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]

    print(
        "Hue mínimo:",
        np.min(hue)
    )

    print(
        "Hue máximo:",
        np.max(hue)
    )

    print(
        "Hue médio:",
        np.mean(hue)
    )

    color_ranges = {

        "green": (
            np.array([40, 70, 50]),
            np.array([95, 255, 255])
        ),

        "blue": (
            np.array([95, 70, 50]),
            np.array([130, 255, 255])
        ),

        "purple": (
            np.array([130, 70, 50]),
            np.array([170, 255, 255])
        ),

        "white": (
            np.array([0, 0, 150]),
            np.array([179, 70, 255])
        )
    }

    total_pixels = (
        roi.shape[0] *
        roi.shape[1]
    )

    color_percentages = {}

    for color, (lower, upper) in color_ranges.items():

        mask = cv2.inRange(
            hsv,
            lower,
            upper
        )

        pixels_detected = cv2.countNonZero(
            mask
        )

        percentage = (
            pixels_detected /
            total_pixels
        ) * 100

        color_percentages[color] = percentage

        if color == "blue":

            blue_hues = hue[mask > 0]
            blue_saturation = saturation[mask > 0]
            blue_value = value[mask > 0]

            if len(blue_hues) > 0:

                print(
                    "Hue dos pixels classificados "
                    "como azul:",
                    "mínimo =", np.min(blue_hues),
                    "| máximo =", np.max(blue_hues),
                    "| médio =", np.mean(blue_hues)
                )

                print(
                    "Saturation dos pixels "
                    "classificados como azul:",
                    "mínimo =", np.min(blue_saturation),
                    "| máximo =", np.max(blue_saturation),
                    "| médio =", np.mean(blue_saturation)
                )

                print(
                    "Value dos pixels "
                    "classificados como azul:",
                    "mínimo =", np.min(blue_value),
                    "| máximo =", np.max(blue_value),
                    "| médio =", np.mean(blue_value)
                )

    detected_color = max(
        color_percentages,
        key=color_percentages.get
    )

    if color_percentages[detected_color] < 15:
        return None, color_percentages

    return detected_color, color_percentages


def main():

    camera = cv2.VideoCapture(0)

    detector = create_detector()

    while True:

        ret, frame = camera.read()

        if not ret:
            print(
                "Erro ao capturar imagem da câmera."
            )
            break

        markers = detect_markers(
            detector,
            frame
        )

        for marker_id, info in markers.items():

            marker_center = info["center"]

            marker_size_px = info["size_px"]

            cube_center, roi = calculate_cube_roi(
                marker_center,
                marker_size_px
            )

            (
                x_min_roi,
                y_min_roi,
                x_max_roi,
                y_max_roi
            ) = roi

            roi_image = frame[
                y_min_roi:y_max_roi,
                x_min_roi:x_max_roi
            ]

            if roi_image.size > 0:

                color, percentages = detect_cube_color(
                    roi_image
                )

                print(
                    f"ID: {marker_id} | "
                    f"Cor: {color} | "
                    f"Percentuais: {percentages}"
                )

                cv2.imshow(
                    f"Cube ROI {marker_id}",
                    roi_image
                )

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