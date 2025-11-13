# optimized_yolo_fixed.py
from ultralytics import YOLO
import cv2, time

ENGINE_IMGSZ = 640  # MUST match your current engine's max size

model = YOLO("yolo11n.engine")  # TRT engine compiled for 640x640

gst = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! "
    "appsink drop=1 max-buffers=1 sync=false"
)
cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise RuntimeError("Failed to open camera")

cv2.setUseOptimized(True)
window_name = "People (YOLO11n TRT)"
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

# Warmup at the CORRECT size so TRT context builds and no crash
for _ in range(3):
    ok, f = cap.read()
    if not ok:
        break
    model.predict(f, imgsz=ENGINE_IMGSZ, classes=[0], conf=0.4, iou=0.45, max_det=20, verbose=False)

prev = time.time()

while True:
    ok, frame = cap.read()
    if not ok:
        break

    results = model.predict(
        frame,
        imgsz=ENGINE_IMGSZ,   # match engine
        classes=[0],          # person only
        conf=0.40,
        iou=0.45,
        max_det=20,
        verbose=False
    )
    r = results[0]

    if r.boxes is not None and len(r.boxes) > 0:
        xyxy = r.boxes.xyxy
        confs = r.boxes.conf
        clss = r.boxes.cls
        if xyxy.is_cuda:
            xyxy = xyxy.cpu(); confs = confs.cpu(); clss = clss.cpu()
        xyxy = xyxy.numpy(); confs = confs.numpy(); clss = clss.numpy()

        h, w = frame.shape[:2]
        thick = max(1, int(round((h + w) / 600)))
        for (x1, y1, x2, y2), c, cls in zip(xyxy, confs, clss):
            if int(cls) != 0:
                continue
            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), thick)
            cv2.putText(frame, f"person {c:.2f}", (x1, max(y1 - 6, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), max(1, thick - 1), cv2.LINE_AA)

    now = time.time()
    fps = 1.0 / (now - prev) if now > prev else 0.0
    prev = now
    cv2.putText(frame, f"FPS: {fps:.1f}", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow(window_name, frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q') or k == 27:
        break

cap.release()
cv2.destroyAllWindows()

