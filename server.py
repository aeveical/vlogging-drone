import cv2
import queue
import threading
from flask import Flask, Response, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

frame_q = queue.Queue(maxsize=2)

HTML = """
<!doctype html>
<html>
  <body style="font-family:sans-serif">
    <h3>Vlogging Drone</h3>
    <img src="/video" style="width:100%;max-width:960px"/>
    <pre id="logs" style="height:40vh;overflow:auto;background:#111;color:#0f0"></pre>

    <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
    <script>
      const socket = io();
      socket.on("log", line => {
        const logs = document.getElementById("logs");
        logs.textContent += line;
        logs.scrollTop = logs.scrollHeight;
      });
    </script>
  </body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/video")
def video():
    def gen():
        while True:
            frame = frame_q.get()
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                jpg.tobytes() + b"\r\n"
            )
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

# -------- public API --------

def push_frame(frame_bgr):
    while not frame_q.empty():
        frame_q.get_nowait()
    try:
        frame_q.put_nowait(frame_bgr)
    except queue.Full:
        pass

def push_log(line):
    socketio.emit("log", line)

def start_server():
    socketio.run(app, host="0.0.0.0", port=8000)
