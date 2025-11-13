import numpy as np
import cv2

gst = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! "
    "appsink drop=1 max-buffers=1 sync=false"
)

name = "Test_Video" + '.mp4'
cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(name, fourcc, 20.0, (1280,720))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()

