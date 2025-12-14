from ultralytics import YOLO
import cv2, time
import numpy as np

class directions:

    ENGINE_IMGSZ = 640  # must match your TensorRT engine
    MODEL_PATH = "yolo11n.pt"

    # ----- distance calibration -----
    # Do this once: place a person ~2–5 m away, read the printed bbox height (h0_px),
    # measure the real distance D0_m with a tape measure, then set CALIB_K = D0_m * h0_px.
    D0_m   = 2.0       # meters at calibration
    h0_px  = 220.0     # bbox height in pixels observed at calibration distance
    CALIB_K = D0_m * h0_px  # proportionality constant K = D0 * h0
    X_MAX = 3820 # screen width
    Y_MAX = 2464 # screen height

    def __init__(self, yaw_angle, height_change, distance, boxA, boxB, cap, model, window_name):
        self.yaw_angle = yaw_angle
        self.height_change = height_change
        self.distance = distance
        self.boxA = boxA
        self.boxB = boxB
        self.cap = cap
        self.model = model
        self.window_name = window_name


    # Optional: if you prefer to assume an average person height, you can refine later using fx, but not required here.

    # ----- model and camera -----

    # gst = (
    #     "nvarguscamerasrc sensor-id=0 ! "
    #     "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    #     "nvvidconv ! video/x-raw, format=BGRx ! "
    #     "videoconvert ! video/x-raw, format=BGR ! "
    #     "appsink drop=1 max-buffers=1 sync=false"
    # )
    def start_cam(self):
        self.model = YOLO(directions.MODEL_PATH)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera")

        cv2.setUseOptimized(True)
        self.window_name = "People + Distance (YOLO11n TRT)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        # Warmup
        for _ in range(3):
            ok, f = self.cap.read()
            if not ok:
                break
            self.model.predict(f, imgsz=directions.ENGINE_IMGSZ, classes=[0], conf=0.4, iou=0.45, max_det=20, verbose=False)

#        prev = time.time() # unused?

    def estimate_distance_m(self, bbox_h_px: float, eps: float = 1e-6) -> float:
        # Z ≈ K / h_px where K = D0 * h0 from calibration
        return float(directions.CALIB_K / max(bbox_h_px, eps))

    def iou(self, boxA, boxB):
        x1A, y1A, x2A, y2A = boxA
        x1B, y1B, x2B, y2B = boxB

        # intersection rectangle
        xA = max(x1A, x1B)
        yA = max(y1A, y1B)
        xB = min(x2A, x2B)
        yB = min(y2A, y2B)

        interW = max(0, xB - xA)
        interH = max(0, yB - yA)
        interArea = interW * interH

        # areas
        areaA = max(0, (x2A - x1A)) * max(0, (y2A - y1A))
        areaB = max(0, (x2B - x1B)) * max(0, (y2B - y1B))

        union = areaA + areaB - interArea + 1e-6  # avoid divide by zero
        return interArea / union


    prev_box=None
    
    def get_directions(self):

        while True:
            ok, frame = self.cap.read()
            if not ok:
                break

            results = self.model.predict(
                frame,
                imgsz=directions.ENGINE_IMGSZ,
                classes=[0],       # person only
                conf=0.40,
                iou=0.45,
                max_det=20,
                verbose=False
            )
            r = results[0]
            boxes=r.boxes

            detections=[]

            if boxes is not None and len(r.boxes) > 0: # probably a pylance issue
                
                xyxy = boxes.xyxy
                confs = boxes.conf
                clss  = boxes.cls
                if xyxy.is_cuda: # pylance
                    xyxy = xyxy.cpu(); confs = confs.cpu(); clss = clss.cpu()
                xyxy = xyxy.numpy(); confs = confs.numpy(); clss = clss.numpy() # These errors are just a pylance thing

                h, w = frame.shape[:2]
                thick = max(1, int(round((h + w) / 600)))

                for (x1, y1, x2, y2), c, cls in zip(xyxy, confs, clss):
                    if int(cls) == 0:
                        detections.append((int(x1), int(y1), int(x2), int(y2)))
                    x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
                    bbox_h = max(1, y2 - y1)
                    Z_m = self.estimate_distance_m(bbox_h)

            chosen = None

            if prev_box is None:
                # first frame → choose largest (closest) person
                if len(detections) > 0:
                    detections.sort(key=lambda b: (b[2]-b[0])*(b[3]-b[1]), reverse=True)
                    chosen = detections[0]
                    prev_box = chosen
            else:
                # compute IOU vs previous frame box
                best_iou = 0
                best_box = None

                for det in detections:
                    score = self.iou(prev_box, det)
                    if score > best_iou:
                        best_iou = score
                        best_box = det

                if best_iou > 0.1:
                    # good match → same person
                    chosen = best_box
                    prev_box = best_box
                else:
                    # target lost → pick largest person
                    if len(detections) > 0:
                        detections.sort(key=lambda b: (b[2]-b[0])*(b[3]-b[1]), reverse=True)
                        chosen = detections[0]
                        prev_box = chosen

            if chosen is not None:
                    x1,y1,x2,y2= chosen
                    # draw box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 0), thick)
                    # center of bounding box for drone stuff
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    self.yaw_angle = 1640 - cx # gets angle we need to yaw it
                    self.height_change = cy*0.0015 - 2 # gets the height we want to go to, normalize hover at 2m


## ---- for the jetson this is the last of the code we need but we'll keep the rest for testing --- ##

                    # draw a small circle at the center
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

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

            cv2.imshow(self.window_name, frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q') or k == 27:
                break

    def end_cv(self):
        self.cap.release()
        cv2.destroyAllWindows()
