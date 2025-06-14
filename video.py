import time
import cv2
from controller import controller

def generate_frames():
    while True:
        if controller.current_frame is not None:
            try:
                ret, buffer = cv2.imencode('.jpg', controller.current_frame)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception as e:
                print(f"[ERROR] Frame encoding error: {e}")
        time.sleep(0.033)