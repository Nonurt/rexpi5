# === GEREKLİ MODÜLLER ===
from flask import Flask, render_template, Response     # Web arayüzü ve video stream için Flask modülleri
import cv2                                              # OpenCV ile kamera erişimi ve görüntü işleme
import time                                             # FPS ayarı ve zamanlama için
from routes import controller                           # Harici controller objesi, kamera ve kontrol yönetimi

app = Flask(__name__)

# === VIDEO STREAMING ===

def generate_frames():
    """Generate video frames for streaming"""
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

        time.sleep(0.033)  # ~30 FPS


# === FLASK ROUTES ===

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    try:
        print("=" * 50)
        print("🕷️  ADVANCED SPIDER ROBOT CONTROL SYSTEM")
        print("=" * 50)
        print("🔧 Features:")
        print("   ✅ AI Human Detection & Tracking")
        print("   ✅ Advanced Camera Control (Pan/Tilt)")
        print("   ✅ 4-Mode Power System (Low/Medium/High/Max)")
        print("   ✅ ROI (Region of Interest) Zone Detection")
        print("   ✅ Smooth Spider Movements")
        print("   ✅ Web Interface Control")
        print("   ✅ Real-time Video Streaming")
        print("=" * 50)
        print("🌐 Web Interface: http://localhost:5000")
        print("📹 Video Stream: http://localhost:5000/video_feed")
        print("=" * 50)

        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
        controller.stop_camera()
        print("[INFO] System stopped safely")
    except Exception as e:
        print(f"[ERROR] System error: {e}")
        controller.stop_camera()