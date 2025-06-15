# app.py
from flask import Flask, render_template, jsonify, request, Response
import threading
import time
import cv2

# Ana robot kontrolcüsünü kendi dosyasından import et
from robot_controller import HumanTrackingServoController

app = Flask(__name__)

# Ana kontrolcü nesnesini bir kez oluştur
# Bu nesne tüm istekler arasında durumu koruyacak
controller = HumanTrackingServoController()


# === VIDEO AKIŞI FONKSİYONU ===
def generate_frames():
    """Video akışı için robot kontrolcüsünden kareleri alır ve yayınlar."""
    while True:
        # Kontrolcünün mevcut karesini bekle
        if controller.current_frame is not None:
            try:
                # Kareyi JPEG formatına çevir
                ret, buffer = cv2.imencode('.jpg', controller.current_frame)
                if ret:
                    frame = buffer.tobytes()
                    # HTTP multipart response olarak kareyi yolla
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception as e:
                print(f"[ERROR] Frame encoding error: {e}")

        # CPU kullanımını düşürmek için kısa bir bekleme (~30 FPS)
        time.sleep(0.033)


# === ANA WEB SAYFASI VE VİDEO ROTASI ===

@app.route('/')
def index():
    """Ana HTML arayüzünü render eder."""
    # `templates` klasöründeki index.html dosyasını arar
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """Video akışını başlatan rota."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# === API KONTROL ROTALARI ===

@app.route('/api/start_camera', methods=['POST'])
def start_camera():
    """Kamerayı ve AI tespitini başlatır."""
    if controller.start_camera():
        return jsonify({'status': 'success', 'message': 'Camera started successfully'})
    else:
        return jsonify({'status': 'error', 'message': 'Failed to start camera'})


@app.route('/api/toggle_tracking', methods=['POST'])
def toggle_tracking():
    """Robotun insanı takip etme (yürüme/dönme) modunu açar/kapatır."""
    controller.tracking_enabled = not controller.tracking_enabled
    status = "enabled" if controller.tracking_enabled else "disabled"
    return jsonify(
        {'status': 'success', 'message': f'Human tracking {status}', 'tracking': controller.tracking_enabled})


@app.route('/api/toggle_camera_tracking', methods=['POST'])
def toggle_camera_tracking():
    """Kameranın insanı pan/tilt yaparak takip etme modunu açar/kapatır."""
    controller.camera_tracking = not controller.camera_tracking
    status = "enabled" if controller.camera_tracking else "disabled"
    return jsonify(
        {'status': 'success', 'message': f'Camera tracking {status}', 'camera_tracking': controller.camera_tracking})


@app.route('/api/set_power_mode', methods=['POST'])
def set_power_mode():
    """Robotun güç modunu (hız, adım aralığı vb.) ayarlar."""
    data = request.get_json()
    mode = data.get('mode', 'medium')

    if controller.set_power_mode(mode):
        settings = controller.get_power_settings()
        return jsonify({
            'status': 'success',
            'message': f'Power mode set to {settings["name"]}',
            'power_mode': mode,
            'settings': settings
        })
    else:
        return jsonify({'status': 'error', 'message': 'Invalid power mode'})


@app.route('/api/manual_move', methods=['POST'])
def manual_move():
    """Temel manuel hareket komutlarını (duruş, saldırı, savunma vb.) işler."""
    data = request.get_json()
    action = data.get('action')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    # Hareket fonksiyonlarını ayrı thread'lerde çalıştırarak web arayüzünün kilitlenmesini engelle
    try:
        if action == 'forward':
            threading.Thread(target=controller.spider_walk_forward, daemon=True).start()
        elif action == 'left':
            threading.Thread(target=controller.spider_turn_left, daemon=True).start()
        elif action == 'right':
            threading.Thread(target=controller.spider_turn_right, daemon=True).start()
        elif action == 'back':
            threading.Thread(target=controller.spider_back_away, daemon=True).start()
        elif action == 'stance':
            threading.Thread(target=controller.spider_stance_position, daemon=True).start()
        elif action == 'defensive':
            threading.Thread(target=controller.spider_defensive_posture, daemon=True).start()
        elif action == 'attack':
            threading.Thread(target=controller.spider_attack_posture, daemon=True).start()
        elif action == 'crouch':
            threading.Thread(target=controller.spider_crouch_low, daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid action'})

        return jsonify({'status': 'success', 'message': f'Executing {action} movement'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Movement error: {str(e)}'})


@app.route('/api/camera_control', methods=['POST'])
def camera_control():
    """Manuel kamera pan/tilt kontrolünü sağlar."""
    data = request.get_json()
    action = data.get('action')

    try:
        if action == 'center':
            controller.set_camera_position(pan_angle=90, tilt_angle=90)
        elif action == 'left':
            new_pan = max(controller.camera_limits['pan_min'], controller.camera_pan_angle - 10)
            controller.set_camera_position(pan_angle=new_pan)
        elif action == 'right':
            new_pan = min(controller.camera_limits['pan_max'], controller.camera_pan_angle + 10)
            controller.set_camera_position(pan_angle=new_pan)
        elif action == 'up':
            new_tilt = max(controller.camera_limits['tilt_min'], controller.camera_tilt_angle - 10)
            controller.set_camera_position(tilt_angle=new_tilt)
        elif action == 'down':
            new_tilt = min(controller.camera_limits['tilt_max'], controller.camera_tilt_angle + 10)
            controller.set_camera_position(tilt_angle=new_tilt)
        else:
            return jsonify({'status': 'error', 'message': 'Invalid camera action'})

        return jsonify({
            'status': 'success',
            'message': f'Camera moved {action}',
            'pan': controller.camera_pan_angle,
            'tilt': controller.camera_tilt_angle
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Camera control error: {str(e)}'})


@app.route('/api/roi_settings', methods=['POST'])
def update_roi_settings():
    """ROI (İlgi Alanı) ayarlarını günceller."""
    data = request.get_json()

    if 'stop_distance' in data:
        controller.roi_stop_distance = max(30, min(200, int(data['stop_distance'])))
    if 'min_distance' in data:
        controller.roi_min_distance = max(20, min(100, int(data['min_distance'])))
    if 'enabled' in data:
        controller.roi_enabled = bool(data['enabled'])

    return jsonify({
        'status': 'success',
        'message': 'ROI settings updated',
        'settings': {
            'stop_distance': controller.roi_stop_distance,
            'min_distance': controller.roi_min_distance,
            'enabled': controller.roi_enabled
        }
    })


@app.route('/api/camera_settings', methods=['POST'])
def update_camera_settings():
    """Kamera takip ayarlarını (dead zone, hız vb.) günceller."""
    data = request.get_json()

    if 'smooth_delay' in data:
        controller.camera_smooth_delay = max(0.01, min(0.2, float(data['smooth_delay'])))
    if 'dead_zone_radius' in data:
        controller.dead_zone_radius = max(20, min(150, int(data['dead_zone_radius'])))
    if 'step_size' in data:
        controller.camera_step_size = max(1, min(10, int(data['step_size'])))

    return jsonify({
        'status': 'success',
        'message': 'Camera settings updated',
        'settings': {
            'smooth_delay': controller.camera_smooth_delay,
            'dead_zone_radius': controller.dead_zone_radius,
            'step_size': controller.camera_step_size
        }
    })


@app.route('/api/status', methods=['GET'])
def get_status():
    """Robotun anlık durumunu (takip, mesafe, güç modu vb.) döndürür."""
    return jsonify({
        'tracking_enabled': controller.tracking_enabled,
        'camera_tracking': controller.camera_tracking,
        'human_detected': controller.human_detected,
        'human_distance': controller.human_distance,
        'roi_zone': controller.last_roi_zone,
        'power_mode': controller.power_mode,
        'walking': controller.walking,
        'camera_running': controller.is_camera_running,
        'camera_position': {
            'pan': controller.camera_pan_angle,
            'tilt': controller.camera_tilt_angle
        },
        'roi_settings': {
            'stop_distance': controller.roi_stop_distance,
            'min_distance': controller.roi_min_distance,
            'enabled': controller.roi_enabled
        }
    })


@app.route('/api/stop_all', methods=['POST'])
def stop_all():
    """Acil durum durdurma. Tüm hareketleri ve takibi durdurur."""
    controller.tracking_enabled = False
    controller.camera_tracking = False
    controller.walking = False

    # Robotu güvenli bir duruş pozisyonuna getir
    threading.Thread(target=controller.spider_stance_position, daemon=True).start()

    return jsonify({'status': 'success', 'message': 'All functions stopped - Emergency stop activated'})


# === YENİ EKLENEN API ROTALARI (PID, LED, GÖRÜNTÜ) ===

@app.route('/api/pid_settings', methods=['POST'])
def update_pid_settings():
    """Web arayüzünden gelen PID kazançlarını günceller."""
    data = request.get_json()
    try:
        controller.pan_kp = float(data.get('pan_kp', controller.pan_kp))
        controller.pan_ki = float(data.get('pan_ki', controller.pan_ki))
        controller.pan_kd = float(data.get('pan_kd', controller.pan_kd))
        controller.tilt_kp = float(data.get('tilt_kp', controller.tilt_kp))
        controller.tilt_ki = float(data.get('tilt_ki', controller.tilt_ki))
        controller.tilt_kd = float(data.get('tilt_kd', controller.tilt_kd))
        print(f"[API] PID settings updated.")
        return jsonify(success=True, message="PID settings updated.")
    except Exception as e:
        return jsonify(success=False, message=str(e)), 400


@app.route('/api/led_control', methods=['POST'])
def led_control():
    """Web arayüzünden LED'i kontrol eder."""
    data = request.get_json()
    action = data.get('action')
    if action == 'toggle':
        new_state = controller.toggle_led()
        return jsonify(success=True, enabled=new_state)
    elif action == 'set_brightness':
        brightness = int(data.get('brightness', 50))
        controller.set_led_brightness(brightness)
        return jsonify(success=True, brightness=brightness)
    return jsonify(success=False, message="Invalid action."), 400


@app.route('/api/image_enhancement', methods=['POST'])
def image_enhancement_control():
    """Web arayüzünden görüntü iyileştirme modlarını açar/kapatır."""
    data = request.get_json()
    toggle_type = data.get('type')
    if toggle_type == 'gamma':
        new_state = controller.toggle_auto_gamma()
        return jsonify(success=True, type='gamma', enabled=new_state)
    elif toggle_type == 'histogram':
        new_state = controller.toggle_histogram_equalization()
        return jsonify(success=True, type='histogram', enabled=new_state)
    return jsonify(success=False, message="Invalid type."), 400


# === GELİŞMİŞ HAREKET ROTALARI ===

@app.route('/api/leg_control', methods=['POST'])
def leg_control():
    """Belirli bacak gruplarını (ön, arka, sol, sağ) kontrol eder."""
    data = request.get_json()
    action = data.get('action')
    leg_group = data.get('leg_group', 'all')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        if action == 'lift':
            threading.Thread(target=controller.lift_legs, args=(leg_group,), daemon=True).start()
        elif action == 'lower':
            threading.Thread(target=controller.lower_legs, args=(leg_group,), daemon=True).start()
        elif action == 'spread':
            threading.Thread(target=controller.spread_legs, args=(leg_group,), daemon=True).start()
        elif action == 'center':
            threading.Thread(target=controller.center_legs, args=(leg_group,), daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid leg action'})

        return jsonify({'status': 'success', 'message': f'{action} {leg_group} legs'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Leg control error: {str(e)}'})


@app.route('/api/gait_control', methods=['POST'])
def gait_control():
    """Alternatif yürüme algoritmalarını tetikler."""
    data = request.get_json()
    gait_type = data.get('gait_type')
    direction = data.get('direction', 'forward')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        if gait_type == 'front_lift_walk':
            threading.Thread(target=controller.front_lift_gait, args=(direction,), daemon=True).start()
        elif gait_type == 'rear_drive_walk':
            threading.Thread(target=controller.rear_drive_gait, args=(direction,), daemon=True).start()
        elif gait_type == 'alternating_pairs':
            threading.Thread(target=controller.alternating_pairs_gait, args=(direction,), daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid gait type'})

        return jsonify({'status': 'success', 'message': f'Executing {gait_type} gait {direction}'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Gait control error: {str(e)}'})


@app.route('/api/rex_movement', methods=['POST'])
def rex_movement():
    """REX tarzı yürüme ve eğilme hareketlerini kontrol eder."""
    data = request.get_json()
    action = data.get('action')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        if action == 'rex_forward':
            threading.Thread(target=controller.rex_forward_gait, daemon=True).start()
        elif action == 'rex_backward':
            threading.Thread(target=controller.rex_backward_gait, daemon=True).start()
        elif action == 'rex_left':
            threading.Thread(target=controller.rex_turn_left, daemon=True).start()
        elif action == 'rex_right':
            threading.Thread(target=controller.rex_turn_right, daemon=True).start()
        elif action == 'rex_stabilize':
            threading.Thread(target=controller.rex_stabilize, daemon=True).start()
        elif action == 'rex_lean_left':
            threading.Thread(target=controller.rex_lean_left, daemon=True).start()
        elif action == 'rex_lean_right':
            threading.Thread(target=controller.rex_lean_right, daemon=True).start()
        elif action == 'rex_lean_forward':
            threading.Thread(target=controller.rex_lean_forward, daemon=True).start()
        elif action == 'rex_lean_back':
            threading.Thread(target=controller.rex_lean_back, daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid REX action'})

        return jsonify({'status': 'success', 'message': f'Executing REX {action}'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'REX movement error: {str(e)}'})


# === UYGULAMAYI BAŞLATMA ===
if __name__ == '__main__':
    try:
        print("=" * 50)
        print("🕷️  MODULAR SPIDER ROBOT CONTROL SYSTEM")
        print("=" * 50)
        print("🌐 Web Interface: http://localhost:5000")
        print("📹 Video Stream: http://localhost:5000/video_feed")
        print("=" * 50)

        # Flask sunucusunu başlat
        # threaded=True, birden fazla isteği aynı anda yönetebilmesini sağlar
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutting down...")
        # GÜVENLİ KAPATMA: Donanım kaynaklarını (Kamera, LED, PCA9685) serbest bırakır.
        controller.cleanup()
        print("[SYSTEM] System stopped safely.")
    except Exception as e:
        print(f"[FATAL ERROR] System shutting down due to an error: {e}")
        # Hata durumunda da kaynakları temizlemeye çalışır.
        controller.cleanup()
