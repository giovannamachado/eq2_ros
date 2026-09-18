import cv2
import numpy as np

from markers.marker import create_detector, detect_markers


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

        print("Markers encontrados:", markers)

        cv2.imshow("Cube Detector", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()