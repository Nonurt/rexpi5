import cv2
import time
import numpy as np
import threading
from config import VIDEO, AI_MODEL, CAMERA_SETTINGS, ROI_SETTINGS, CAMERA_TRACKING_SETTINGS


class CameraAIHandler:
    """
    Kamera işlemleri, AI insan tespiti, görüntü işleme ve kamera servo
    kontrolü için tüm metotları içeren bir "mixin" sınıfı.
    Bu sınıf tek başına çalışmaz, RobotController tarafından miras alınır.
    """

    # YENİ EKLENEN KONTROL DEĞİŞKENLERİ
    # Bu değişkenlerin RobotController'ın __init__ metodunda False olarak başlatılması gerekir.
    auto_gamma_enabled = False
    histogram_equalization_enabled = False

    def init_ai_model(self):
        """AI modelini config dosyasındaki yollardan yükler."""
        try:
            print("[INFO] Loading AI Model...")
            self.net = cv2.dnn.readNetFromCaffe(AI_MODEL['prototxt_path'], AI_MODEL['model_path'])
            print("[INFO] AI Model loaded successfully!")
        except Exception as e:
            print(f"[ERROR] AI Model could not be loaded: {e}")
            self.net = None

    def start_camera(self):
        """Kamera donanımını başlatır ve ana kamera döngüsünü bir thread'de çalıştırır."""
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("[ERROR] Cannot open camera.")
                return False

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO['width'])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO['height'])
            self.is_camera_running = True

            camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            camera_thread.start()

            print("[INFO] Camera started")
            return True
        except Exception as e:
            print(f"[ERROR] Camera could not be started: {e}")
            return False

    def stop_camera(self):
        """Kamerayı güvenli bir şekilde kapatır."""
        self.is_camera_running = False
        self.tracking_enabled = False
        self.camera_tracking = False
        if self.cap:
            self.cap.release()
        print("[INFO] Camera stopped")

    def init_camera_position(self):
        """Kamera servolarını başlangıç pozisyonuna (merkez) ayarlar."""
        print("[CAMERA] Setting initial camera position...")
        self.set_camera_position(pan_angle=90, tilt_angle=90, smooth=False)
        time.sleep(1)
        print("[CAMERA] Camera is in center position.")

    # --- YENİ EKLENEN FONKSİYONLAR ---
    def toggle_auto_gamma(self):
        """Otomatik Gamma modunu açar/kapatır."""
        self.auto_gamma_enabled = not self.auto_gamma_enabled
        print(f"[CAMERA] Auto Gamma set to: {self.auto_gamma_enabled}")
        return self.auto_gamma_enabled

    def toggle_histogram_equalization(self):
        """Histogram Eşitleme modunu açar/kapatır."""
        self.histogram_equalization_enabled = not self.histogram_equalization_enabled
        print(f"[CAMERA] Histogram Equalization set to: {self.histogram_equalization_enabled}")
        return self.histogram_equalization_enabled

    def _apply_auto_gamma(self, frame):
        """Görüntü parlaklığı düşükse gamma düzeltmesi uygular."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)

        # Eğer görüntü çok karanlıksa (örneğin ortalama parlaklık 100'ün altındaysa)
        if mean_brightness < 100:
            # Parlaklığı artırmak için gamma değerini 1'den büyük yap
            gamma = 1.5
            inv_gamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
            return cv2.LUT(frame, table)
        return frame

    def _apply_histogram_equalization(self, frame):
        """Görüntü kontrastını artırmak için histogram eşitleme uygular."""
        # Renkli görüntüde bozulmayı önlemek için YUV renk uzayında sadece parlaklık (Y) kanalına uygulanır
        yuv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        yuv_img[:, :, 0] = cv2.equalizeHist(yuv_img[:, :, 0])
        return cv2.cvtColor(yuv_img, cv2.COLOR_YUV2BGR)

    # --------------------------------

    def camera_loop(self):
        """Sürekli olarak kameradan görüntü okuyan, işleyen ve saklayan ana döngü."""
        while self.is_camera_running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                # --- YENİ EKLENEN GÖRÜNTÜ İYİLEŞTİRME KONTROLLERİ ---
                if self.auto_gamma_enabled:
                    frame = self._apply_auto_gamma(frame)

                if self.histogram_equalization_enabled:
                    frame = self._apply_histogram_equalization(frame)
                # ----------------------------------------------------

                if self.net is not None:
                    frame = self.detect_humans(frame)

                frame = self.draw_roi_zones(frame)
                frame = self.add_status_overlay(frame)

                self.current_frame = frame
                time.sleep(0.02)

            except Exception as e:
                print(f"[ERROR] Camera loop error: {e}")
                time.sleep(1)

    def detect_humans(self, frame):
        # ... (Bu fonksiyonun içeriği aynı kalır, değişiklik yok)
        if self.net is None:
            return frame

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        best_person = None
        best_confidence = 0.0

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            class_id = int(detections[0, 0, i, 1])

            if class_id == AI_MODEL['person_class_id'] and confidence > AI_MODEL['confidence_threshold']:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                if confidence > best_confidence:
                    best_confidence = confidence
                    best_person = {
                        'bbox': (startX, startY, endX, endY),
                        'confidence': confidence,
                        'center': ((startX + endX) // 2, (startY + endY) // 2),
                        'height': endY - startY
                    }

        if best_person:
            self.human_detected = True
            self.last_detection_time = time.time()
            bbox = best_person['bbox']
            center = best_person['center']

            self.human_distance = self.calculate_distance(best_person['height'])
            self.human_position = center
            roi_zone = self.detect_roi_zone(center)
            self.last_roi_zone = roi_zone

            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.putText(frame, f"Distance: {self.human_distance}cm", (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

            if self.camera_tracking:
                self.camera_track_human(center)

            if self.tracking_enabled:
                self.track_human(center, self.human_distance, roi_zone)
        else:
            self.human_detected = False
            if self.camera_tracking:
                current_time = time.time()
                if current_time - getattr(self, 'last_detection_time', 0) > 1.5:
                    if not getattr(self, 'is_searching', False):
                        self.is_searching = True
                        threading.Thread(target=self.camera_sweep_search, daemon=True).start()

        return frame

    def add_status_overlay(self, frame):
        """Görüntünün üzerine robotun anlık durumunu yazan bir katman ekler."""
        # --- DURUM BİLGİSİNE YENİ SATIRLAR EKLENDİ ---
        status_lines = [
            f"Robot Track: {'ON' if self.tracking_enabled else 'OFF'}",
            f"Camera Track: {'ON' if self.camera_tracking else 'OFF'}",
            f"Human: {'DETECTED' if self.human_detected else 'NOT FOUND'}",
            f"Distance: {getattr(self, 'human_distance', 0)} cm",
            f"ROI Zone: {getattr(self, 'last_roi_zone', 'N/A').upper()}",
            f"Power: {self.power_mode.upper()}",
            f"Walking: {'YES' if self.walking else 'NO'}",
            "--- Image Enhancement ---",  # Ayırıcı
            f"Auto Gamma: {'ON' if self.auto_gamma_enabled else 'OFF'}",
            f"Histogram Eq: {'ON' if self.histogram_equalization_enabled else 'OFF'}"
        ]
        # --------------------------------------------

        y0, dy = 20, 20
        for i, line in enumerate(status_lines):
            y = y0 + i * dy
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        dead_zone_radius = CAMERA_TRACKING_SETTINGS['dead_zone_radius']
        cv2.circle(frame, (self.frame_center_x, self.frame_center_y), int(dead_zone_radius), (0, 255, 255), 1)

        return frame

    # ... (Geriye kalan tüm fonksiyonlar: draw_roi_zones, camera_track_human, set_camera_position, camera_sweep_search aynı kalır)
    def draw_roi_zones(self, frame):
        """Yapılandırmadaki ROI bölgelerini ekrana çizer."""
        if not hasattr(self, 'roi_enabled') or not self.roi_enabled:
            return frame

        colors = {'center': (0, 255, 0), 'left': (255, 0, 0), 'right': (255, 0, 0), 'top': (0, 255, 255),
                  'bottom': (0, 255, 255)}

        for zone_name, zone in ROI_SETTINGS['zones'].items():
            color = colors.get(zone_name, (255, 255, 255))
            cv2.rectangle(frame, (zone['x'], zone['y']), (zone['x'] + zone['w'], zone['y'] + zone['h']), color, 1)

        return frame

    def camera_track_human(self, human_center):
        """
        Orantısal kontrol kullanarak kameranın pan ve tilt servolarını
        hareket ettirerek insanı daha pürüzsüz takip etmesini sağlar.
        """
        if not self.camera_tracking or self.camera_lock.locked():
            return

        with self.camera_lock:
            center_x, center_y = human_center
            error_pan = self.frame_center_x - center_x
            error_tilt = self.frame_center_y - center_y
            dead_zone_radius = CAMERA_TRACKING_SETTINGS['dead_zone_radius']
            p_gain = CAMERA_TRACKING_SETTINGS['p_gain']

            if abs(error_pan) < dead_zone_radius and abs(error_tilt) < dead_zone_radius:
                return

            pan_adjustment = error_pan * p_gain
            tilt_adjustment = error_tilt * p_gain
            new_pan_angle = self.camera_pan_angle + pan_adjustment
            new_tilt_angle = self.camera_tilt_angle - tilt_adjustment
            self.set_camera_position(new_pan_angle, new_tilt_angle, smooth=True)

    def set_camera_position(self, pan_angle=None, tilt_angle=None, smooth=False):
        """Kamera servolarını belirtilen açılara ayarlar."""
        if pan_angle is not None:
            self.camera_pan_angle = max(CAMERA_SETTINGS['pan_min'], min(CAMERA_SETTINGS['pan_max'], pan_angle))
            if smooth:
                self.smooth_servo_move('camera_pan', self.camera_pan_angle)
            else:
                self.set_servo_angle('camera_pan', self.camera_pan_angle)

        if tilt_angle is not None:
            self.camera_tilt_angle = max(CAMERA_SETTINGS['tilt_min'], min(CAMERA_SETTINGS['tilt_max'], tilt_angle))
            if smooth:
                self.smooth_servo_move('camera_tilt', self.camera_tilt_angle)
            else:
                self.set_servo_angle('camera_tilt', self.camera_tilt_angle)

    def camera_sweep_search(self):
        """İnsan bulunamadığında kameranın etrafı taramasını sağlar."""
        if not self.camera_tracking:
            self.is_searching = False
            return

        print("[CAMERA] Human lost. Starting sweep search...")
        positions = [90, 60, 120, 45, 135, 90]
        for pan_pos in positions:
            if self.human_detected:
                print("[CAMERA] Human re-acquired during sweep. Stopping search.")
                break
            self.set_camera_position(pan_angle=pan_pos, smooth=True)
            time.sleep(0.6)

        if not self.human_detected:
            print("[CAMERA] Sweep search finished. Human not found. Centering camera.")
            self.set_camera_position(90, 90, smooth=True)

        self.is_searching = False