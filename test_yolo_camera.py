import cv2
from ultralytics import YOLO


def main():
    model = YOLO("yolo11n.pt")

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera.")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Cannot read frame.")
            break

        results = model(frame, verbose=False)

        annotated_frame = results[0].plot()

        cv2.imshow("YOLO Camera Test", annotated_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()