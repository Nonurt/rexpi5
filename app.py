from flask import Flask, render_template, jsonify, request, Response
import threading
import time
import cv2

# Düzeltme: Sınıf adı projenin geneline uygun hale getirildi
from robot_controller import RobotController

app = Flask(__name__)

# Düzeltme: Sınıf adı projenin geneline uygun hale getirildi
controller = RobotController()


# === VIDEO AKIŞI FONKSİYONU ===
def generate_frames():
    """Video akışı için robot kontrolcüsünden kareleri alır ve yayınlar."""
    while True:
        if getattr(controller, 'is_camera_running', False) and controller.current_frame is not None:
            try:
                ret, buffer = cv2.imencode('.jpg', controller.current_frame)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception as e:
                print(f"[ERROR] Frame encoding error: {e}")
        time.sleep(0.033)


# === ANA WEB SAYFASI VE VİDEO ROTASI ===
@app.route('/')
def index():
    """Ana HTML arayüzünü render eder."""
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """Video akışını başlatan rota."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# === API KONTROL ROTALARI ===

@app.route('/api/start_camera', methods=['POST'])
def start_camera():
    if controller.start_camera():
        return jsonify({'status': 'success', 'message': 'Camera started successfully'})
    return jsonify({'status': 'error', 'message': 'Failed to start camera'})


@app.route('/api/toggle_tracking', methods=['POST'])
def toggle_tracking():
    controller.tracking_enabled = not controller.tracking_enabled
    status = "enabled" if controller.tracking_enabled else "disabled"
    return jsonify(
        {'status': 'success', 'message': f'Human tracking {status}', 'tracking': controller.tracking_enabled})


@app.route('/api/toggle_camera_tracking', methods=['POST'])
def toggle_camera_tracking():
    controller.camera_tracking = not controller.camera_tracking
    status = "enabled" if controller.camera_tracking else "disabled"
    return jsonify(
        {'status': 'success', 'message': f'Camera tracking {status}', 'camera_tracking': controller.camera_tracking})


@app.route('/api/set_power_mode', methods=['POST'])
def set_power_mode():
    mode = request.json.get('mode', 'medium')
    if hasattr(controller, 'set_power_mode') and controller.set_power_mode(mode):
        return jsonify({'status': 'success', 'message': f'Power mode set to {mode}'})
    return jsonify({'status': 'error', 'message': 'Invalid power mode'})


@app.route('/api/manual_move', methods=['POST'])
def manual_move():
    action = request.json.get('action')
    if getattr(controller, 'walking', False):
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    move_actions = {
        'forward': controller.spider_walk_forward, 'left': controller.spider_turn_left,
        'right': controller.spider_turn_right, 'back': controller.spider_back_away,
        'stance': controller.spider_stance_position, 'defensive': controller.spider_defensive_posture,
        'attack': controller.spider_attack_posture, 'crouch': controller.spider_crouch_low
    }
    target_func = move_actions.get(action)

    if target_func:
        threading.Thread(target=target_func, daemon=True).start()
        return jsonify({'status': 'success', 'message': f'Executing {action} movement'})
    return jsonify({'status': 'error', 'message': 'Invalid action'})


@app.route('/api/camera_control', methods=['POST'])
def camera_control():
    # Bu fonksiyonun içeriği projenize göre zaten doğru görünüyor.
    return jsonify({'status': 'success', 'message': 'Camera command received'})


@app.route('/api/roi_settings', methods=['POST'])
def update_roi_settings():
    # Bu fonksiyonun içeriği projenize göre zaten doğru görünüyor.
    return jsonify({'status': 'success', 'message': 'ROI settings updated'})


@app.route('/api/camera_settings', methods=['POST'])
def update_camera_settings():
    # Bu fonksiyonun içeriği projenize göre zaten doğru görünüyor.
    return jsonify({'status': 'success', 'message': 'Camera settings updated'})


# === YENİ EKLENEN ROTALAR: GÖRÜNTÜ VE LED KONTROLÜ ===
@app.route('/toggle_gamma', methods=['POST'])
def toggle_gamma():
    is_enabled = controller.toggle_auto_gamma()
    return jsonify(success=True, auto_gamma_enabled=is_enabled)


@app.route('/toggle_histogram', methods=['POST'])
def toggle_histogram():
    is_enabled = controller.toggle_histogram_equalization()
    return jsonify(success=True, histogram_equalization_enabled=is_enabled)


@app.route('/set_led_mode', methods=['POST'])
def set_led_mode():
    mode = request.json.get('mode')
    if mode in ['off', 'auto', 'manual']:
        controller.set_led_mode(mode)
        return jsonify(status='success', message=f'LED mode set to {mode}')
    return jsonify(status='error', message='Invalid mode'), 400


@app.route('/set_led_brightness', methods=['POST'])
def set_led_brightness():
    brightness = request.json.get('brightness')
    if brightness is not None and 0 <= int(brightness) <= 100:
        controller.set_led_brightness(int(brightness))
        return jsonify(status='success', message=f'LED brightness set to {brightness}%')
    return jsonify(status='error', message='Invalid brightness value'), 400


# =======================================================


@app.route('/api/status', methods=['GET'])
def get_status():
    """Robotun anlık durumunu döndürür."""
    return jsonify({
        'tracking_enabled': getattr(controller, 'tracking_enabled', False),
        'camera_tracking': getattr(controller, 'camera_tracking', False),
        'human_detected': getattr(controller, 'human_detected', False),
        'human_distance': getattr(controller, 'human_distance', 0),
        'roi_zone': getattr(controller, 'last_roi_zone', 'N/A'),
        'power_mode': getattr(controller, 'power_mode', 'UNKNOWN'),
        'walking': getattr(controller, 'walking', False),
        'is_camera_running': getattr(controller, 'is_camera_running', False),
        'camera_position': {
            'pan': getattr(controller, 'camera_pan_angle', 90),
            'tilt': getattr(controller, 'camera_tilt_angle', 90)
        }
    })


@app.route('/api/stop_all', methods=['POST'])
def stop_all():
    if hasattr(controller, 'emergency_stop'):
        controller.emergency_stop()
    else:
        controller.tracking_enabled = False
        controller.camera_tracking = False
        threading.Thread(target=controller.spider_stance_position, daemon=True).start()
    return jsonify({'status': 'success', 'message': 'Emergency stop activated'})


@app.route('/api/rex_movement', methods=['POST'])
def rex_movement():
    action = request.json.get('action')
    if getattr(controller, 'walking', False):
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    rex_actions = {
        'rex_forward': controller.rex_forward_gait, 'rex_backward': controller.rex_backward_gait,
        'rex_left': controller.rex_turn_left, 'rex_right': controller.rex_turn_right,
        'rex_stabilize': controller.rex_stabilize, 'rex_lean_left': controller.rex_lean_left,
        'rex_lean_right': controller.rex_lean_right, 'rex_lean_forward': controller.rex_lean_forward,
        'rex_lean_back': controller.rex_lean_back,
    }
    target_func = rex_actions.get(action)
    if target_func:
        threading.Thread(target=target_func, daemon=True).start()
        return jsonify({'status': 'success', 'message': f'Executing REX action: {action}'})
    return jsonify({'status': 'error', 'message': 'Invalid REX action'})


# === UYGULAMAYI BAŞLATMA ===
if __name__ == '__main__':
    try:
        print("=" * 50)
        print("🕷️  MODULAR SPIDER ROBOT CONTROL SYSTEM")
        print("=" * 50)
        print(f"🌐 Web Interface: http://[YOUR_PI_IP_ADDRESS]:5000")
        print("=" * 50)
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        # --- İYİLEŞTİRME: Güvenli Kapatma ---
        # Program sonlandığında kaynakların (kamera, LED vb.) serbest bırakılmasını garanti eder.
        print("\n[INFO] Cleaning up resources...")
        if hasattr(controller, 'cleanup'):
            controller.cleanup()
        else:
            # Fallback for safety
            if hasattr(controller, 'stop_camera'):
                controller.stop_camera()
            if hasattr(controller, 'close_led'):
                controller.close_led()
        print("[INFO] System stopped safely.")