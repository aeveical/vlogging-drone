from ultralytics import YOLO as yolo
import cv2 as cv

model = yolo("yolo11n.pt")

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        print("Can't recieve frame")
        break

    if ret:
        results = model.track(frame, persist=True)
        annotated_frame = results[0].plot()
        cv.imshow("Live", annotated_frame)

    if cv.waitKey(1) == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
