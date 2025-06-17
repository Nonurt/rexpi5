import cv2
import time
import numpy as np
import threading
from math import hypot
from config import VIDEO, AI_MODEL, CAMERA_SETTINGS, ROI_SETTINGS, CAMERA_TRACKING_SETTINGS


class CameraAIHandler:
    """
    Kamera işlemleri, AI insan tespiti, görüntü işleme ve kamera servo
    kontrolü için tüm metotları içeren bir "mixin" sınıfı.
    İlk gördüğü insanı "target" olarak kilitler ve hedefi merkezdeki dairenin
    (dead-zone) içinde tutmaya çalışır.
    """

    # ───────────────────────────── AI MODEL ──────────────────────────────
    def init_ai_model(self):
        try:
            print("[INFO] Loading AI Model…")
            self.net = cv2.dnn.readNetFromCaffe(AI_MODEL['prototxt_path'], AI_MODEL['model_path'])
            print("[INFO] AI Model loaded OK")
        except Exception as e:
            print(f"[ERROR] AI Model could not be loaded: {e}")
            self.net = None

    # ───────────────────────────── CAMERA START/STOP ─────────────────────
    def start_camera(self):
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("[ERROR] Cannot open camera")
                return False

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  VIDEO['width'])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO['height'])
            self.is_camera_running = True

            self.init_camera_ai_variables()
            threading.Thread(target=self.camera_loop, daemon=True).start()
            print("[INFO] Camera started")
            return True
        except Exception as e:
            print(f"[ERROR] Camera could not be started: {e}")
            return False

    def stop_camera(self):
        self.is_camera_running = False
        self.tracking_enabled = False
        self.camera_tracking  = False
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        print("[INFO] Camera stopped")

    def init_camera_position(self):
        """Kamera servosunu merkeze alır ve başlangıç pozisyonuna getirir."""
        print("[CAMERA] Setting initial camera position...")
        self.set_camera_position(pan_angle=90, tilt_angle=90, smooth=False)
        time.sleep(1)
        print("[CAMERA] Camera is in center position.")

    # ────────────────────────── STATE & PD VARIABLES ──────────────────────
    def init_camera_ai_variables(self):
        # Frame geometrisi
        self.frame_center_x = VIDEO['width']  // 2
        self.frame_center_y = VIDEO['height'] // 2

        # Servo açısı ve kilit
        self.camera_lock       = threading.Lock()
        self.camera_pan_angle  = 90
        self.camera_tilt_angle = 90

        # Durum bayrakları
        self.camera_tracking     = True
        self.tracking_enabled    = False
        self.human_detected      = False
        self.is_searching        = False
        self.last_detection_time = 0
        self.human_distance      = 0
        self.last_roi_zone       = "none"
        self.current_frame       = None

        # Target-lock değişkenleri
        self.target_locked   = False
        self.target_center   = (0, 0)
        self.target_lost_cnt = 0

        # PD kontrol geçmişi
        self.prev_error_pan  = 0
        self.prev_error_tilt = 0

    # ───────────────────────────── CAMERA LOOP ───────────────────────────
    def camera_loop(self):
        while self.is_camera_running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.05)
                    continue

                if self.net is not None:
                    frame = self.detect_humans(frame)

                frame = self.draw_roi_zones(frame)
                frame = self.add_status_overlay(frame)
                self.current_frame = frame
                time.sleep(0.02)
            except Exception as e:
                print(f"[ERROR] Camera loop error: {e}")
                time.sleep(0.2)

    # ────────────────────────── HUMAN DETECTION ──────────────────────────
    def detect_humans(self, frame):
        if self.net is None:
            return frame

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300,300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        persons = []
        for i in range(detections.shape[2]):
            conf = detections[0,0,i,2]
            cls  = int(detections[0,0,i,1])
            if cls == AI_MODEL['person_class_id'] and conf > AI_MODEL['confidence_threshold']:
                box = detections[0,0,i,3:7] * np.array([w,h,w,h])
                x1,y1,x2,y2 = box.astype(int)
                center = ((x1+x2)//2, (y1+y2)//2)
                persons.append({'bbox':(x1,y1,x2,y2),'center':center,'height':y2-y1,'conf':conf})

        target = None
        if self.target_locked and persons:
            best_d = float('inf')
            for p in persons:
                d = hypot(p['center'][0]-self.target_center[0], p['center'][1]-self.target_center[1])
                if d < best_d:
                    best_d = d
                    target = p
            if best_d > 150:
                self.target_lost_cnt += 1
                if self.target_lost_cnt > 30:
                    self.target_locked = False
                    self.target_lost_cnt = 0
            else:
                self.target_lost_cnt = 0
        if not self.target_locked and persons:
            target = max(persons, key=lambda p: p['conf'])
            self.target_locked = True
            self.target_lost_cnt = 0

        if target:
            self.human_detected = True
            self.last_detection_time = time.time()
            self.target_center = target['center']
            self.human_distance = self.calculate_distance(target['height'])
            self.human_position = target['center']
            self.last_roi_zone = self.detect_roi_zone(target['center'])

            x1,y1,x2,y2 = target['bbox']
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.circle(frame, target['center'], 4, (0,255,0), -1)
            cv2.putText(frame, "Target", (x1, y1-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            if self.camera_tracking:
                self.camera_track_human(target['center'])
        else:
            self.human_detected = False
            if self.camera_tracking and not self.is_searching:
                if time.time() - self.last_detection_time > 2.5:
                    print("[CAMERA] Target lost → sweep search")
                    self.is_searching = True
                    threading.Thread(target=self.camera_sweep_search, daemon=True).start()
        return frame

    # ────────────────────────── CAMERA PD CONTROL ─────────────────────────
    def camera_track_human(self, center):
        if not self.camera_tracking or self.camera_lock.locked():
            return
        with self.camera_lock:
            # Hataları hesapla
            err_pan  = self.frame_center_x - center[0]
            err_tilt = self.frame_center_y - center[1]

            # Dead-zone + min hata eşiği
            dz  = CAMERA_TRACKING_SETTINGS['dead_zone_radius']
            eps = CAMERA_TRACKING_SETTINGS.get('min_error', 5)
            if abs(err_pan) < dz and abs(err_tilt) < dz:
                return
            if abs(err_pan) < eps and abs(err_tilt) < eps:
                return

            # PD kazançları
            Kp_pan  = CAMERA_TRACKING_SETTINGS.get('p_gain_pan', 0.06)
            Kd_pan  = CAMERA_TRACKING_SETTINGS.get('d_gain_pan', 0.02)
            Kp_tilt = CAMERA_TRACKING_SETTINGS.get('p_gain_tilt',0.05)
            Kd_tilt = CAMERA_TRACKING_SETTINGS.get('d_gain_tilt',0.02)
            dt = 0.02

            # Türev terimleri
            d_pan  = (err_pan  - self.prev_error_pan)  / dt
            d_tilt = (err_tilt - self.prev_error_tilt) / dt

            # PD hesap
            pan_adj  = Kp_pan  * err_pan  + Kd_pan  * d_pan
            tilt_adj = Kp_tilt * err_tilt + Kd_tilt * d_tilt

            # Slew-rate sınırı
            pan_adj  = np.clip(pan_adj,
                               -CAMERA_TRACKING_SETTINGS['max_step_pan'],
                                CAMERA_TRACKING_SETTINGS['max_step_pan'])
            tilt_adj = np.clip(tilt_adj,
                               -CAMERA_TRACKING_SETTINGS['max_step_tilt'],
                                CAMERA_TRACKING_SETTINGS['max_step_tilt'])

            # Geçmişi güncelle
            self.prev_error_pan  = err_pan
            self.prev_error_tilt = err_tilt

            # Yön tersleme
            pan_sign  = -1 if CAMERA_SETTINGS.get('invert_pan', False) else 1
            tilt_sign =  1 if CAMERA_SETTINGS.get('invert_tilt', False) else -1

            new_pan  = self.camera_pan_angle  + pan_sign  * pan_adj
            new_tilt = self.camera_tilt_angle + tilt_sign * tilt_adj
            self.set_camera_position(new_pan, new_tilt, smooth=True)

    # ────────────────────────── DRAW OVERLAY ──────────────────────────────
    def add_status_overlay(self, frame):
        lines = [
            f"Robot Track: {'ON' if self.tracking_enabled else 'OFF'}",
            f"Camera Track: {'ON' if self.camera_tracking else 'OFF'}",
            f"Human: {'DETECTED' if self.human_detected else 'NOT FOUND'}",
            f"Distance: {self.human_distance} cm",
            f"ROI Zone: {self.last_roi_zone.upper()}",
            f"Power: {self.power_mode.upper()}",
            f"Walking: {'YES' if self.walking else 'NO'}"
        ]
        y0, dy = 20, 20
        for i, line in enumerate(lines):
            y = y0 + i * dy
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        dz = CAMERA_TRACKING_SETTINGS['dead_zone_radius']
        cv2.circle(frame, (self.frame_center_x, self.frame_center_y), int(dz), (0, 255, 255), 1)
        return frame

    # ────────────────────────── DRAW ROI ZONES ────────────────────────────
    def draw_roi_zones(self, frame):
        if not self.roi_enabled:
            return frame
        colors = {'center': (0, 255, 0), 'left': (255, 0, 0), 'right': (255, 0, 0), 'top': (0, 255, 255), 'bottom': (0, 255, 255)}
        for name, zone in ROI_SETTINGS['zones'].items():
            col = colors.get(name, (255, 255, 255))
            cv2.rectangle(frame, (zone['x'], zone['y']), (zone['x']+zone['w'], zone['y']+zone['h']), col, 1)
        return frame

    # ────────────────────────── SERVO POSITION ───────────────────────────
    def set_camera_position(self, pan_angle=None, tilt_angle=None, smooth=False):
        invert_pan = CAMERA_SETTINGS.get('invert_pan', False)
        invert_tilt = CAMERA_SETTINGS.get('invert_tilt', False)
        if pan_angle is not None:
            pan = 180 - pan_angle if invert_pan else pan_angle
            self.camera_pan_angle = max(CAMERA_SETTINGS['pan_min'], min(CAMERA_SETTINGS['pan_max'], pan))
            if smooth:
                self.smooth_servo_move('camera_pan', self.camera_pan_angle)
            else:
                self.set_servo_angle('camera_pan', self.camera_pan_angle)
        if tilt_angle is not None:
            tilt = 180 - tilt_angle if invert_tilt else tilt_angle
            self.camera_tilt_angle = max(CAMERA_SETTINGS['tilt_min'], min(CAMERA_SETTINGS['tilt_max'], tilt))
            if smooth:
                self.smooth_servo_move('camera_tilt', self.camera_tilt_angle)
            else:
                self.set_servo_angle('camera_tilt', self.camera_tilt_angle)

    # ────────────────────────── SWEEP SEARCH ──────────────────────────────
    def camera_sweep_search(self):
        if not self.camera_tracking:
            self.is_searching = False
            return
        print("[CAMERA] Human lost. Starting sweep search...")
        positions = [90, 60, 120, 45, 135, 90]
        for p in positions:
            if self.human_detected:
                print("[CAMERA] Human re-acquired. Stopping.")
                break
            self.set_camera_position(p, None, True)
            time.sleep(0.6)
        if not self.human_detected:
            print("[CAMERA] Sweep finished. Centering.")
            self.set_camera_position(90, 90, True)
        self.is_searching = False