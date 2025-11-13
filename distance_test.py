# optimized_yolo_with_distance.py
# Based on your optimized person-only script, adds monocular distance via 1-point calibration.

from ultralytics import YOLO
import cv2, time

ENGINE_IMGSZ = 640  # must match your TensorRT engine
MODEL_PATH = "yolo11n.engine"

# ----- distance calibration -----
# Do this once: place a person ~2–5 m away, read the printed bbox height (h0_px),
# measure the real distance D0_m with a tape measure, then set CALIB_K = D0_m * h0_px.
D0_m   = 2.0       # meters at calibration
h0_px  = 220.0     # bbox height in pixels observed at calibration distance
CALIB_K = D0_m * h0_px  # proportionality constant K = D0 * h0

# Optional: if you prefer to assume an average person height, you can refine later using fx, but not required here.

# ----- model and camera -----
model = YOLO(MODEL_PATH)

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
window_name = "People + Distance (YOLO11n TRT)"
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

# Warmup
for _ in range(3):
    ok, f = cap.read()
    if not ok:
        break
    model.predict(f, imgsz=ENGINE_IMGSZ, classes=[0], conf=0.4, iou=0.45, max_det=20, verbose=False)

prev = time.time()

def estimate_distance_m(bbox_h_px: float, eps: float = 1e-6) -> float:
    # Z ≈ K / h_px where K = D0 * h0 from calibration
    return float(CALIB_K / max(bbox_h_px, eps))

while True:
    ok, frame = cap.read()
    if not ok:
        break

    results = model.predict(
        frame,
        imgsz=ENGINE_IMGSZ,
        classes=[0],       # person only
        conf=0.40,
        iou=0.45,
        max_det=20,
        verbose=False
    )
    r = results[0]

    if r.boxes is not None and len(r.boxes) > 0:
        xyxy = r.boxes.xyxy
        confs = r.boxes.conf
        clss  = r.boxes.cls
        if xyxy.is_cuda:
            xyxy = xyxy.cpu(); confs = confs.cpu(); clss = clss.cpu()
        xyxy = xyxy.numpy(); confs = confs.numpy(); clss = clss.numpy()

        h, w = frame.shape[:2]
        thick = max(1, int(round((h + w) / 600)))

        for (x1, y1, x2, y2), c, cls in zip(xyxy, confs, clss):
            if int(cls) != 0:
                continue
            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            bbox_h = max(1, y2 - y1)
            Z_m = estimate_distance_m(bbox_h)

            # draw box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), thick)

            # label: confidence + distance
            label = f"person {c:.2f} | {Z_m:.2f} m"
            # background for readability
            (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, max(1, thick-1))
            cv2.rectangle(frame, (x1, max(y1 - th - 8, 0)), (x1 + tw + 4, y1), (0, 220, 0), -1)
            cv2.putText(frame, label, (x1 + 2, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), max(1, thick-1), cv2.LINE_AA)

    # FPS overlay
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

