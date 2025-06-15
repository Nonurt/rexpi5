# camera_ai_handler.py
import cv2
import time
import numpy as np
import threading
from config import VIDEO, AI_MODEL, CAMERA_SETTINGS, ROI_SETTINGS


class CameraAIHandler:
    """
    Kamera işlemleri, AI insan tespiti, görüntü işleme ve kamera servo
    kontrolü için tüm metotları içeren bir "mixin" sınıfı.
    Bu sınıf tek başına çalışmaz, RobotController tarafından miras alınır.
    """

    def init_ai_model(self):
        """AI modelini config dosyasındaki yollardan yükler."""
        try:
            print("[INFO] Loading AI Model...")
            # Ayarları config modülünden al
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

            # Görüntü boyutlarını config modülünden al
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO['width'])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO['height'])
            self.is_camera_running = True

            # Kamera döngüsünü ana programı bloklamayacak şekilde başlat
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

    def camera_loop(self):
        """Sürekli olarak kameradan görüntü okuyan, işleyen ve saklayan ana döngü."""
        while self.is_camera_running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                # AI modeli yüklendiyse insan tespiti yap
                if self.net is not None:
                    frame = self.detect_humans(frame)

                # Görüntü üzerine ROI ve durum bilgilerini çiz
                frame = self.draw_roi_zones(frame)
                frame = self.add_status_overlay(frame)

                # İşlenmiş kareyi, web arayüzünde gösterilmesi için sakla
                self.current_frame = frame

                # FPS'i limitlemek için kısa bekleme
                time.sleep(VIDEO['fps_delay'])

            except Exception as e:
                print(f"[ERROR] Camera loop error: {e}")
                time.sleep(1)

    def detect_humans(self, frame):
        """
        Yapay zeka modelini kullanarak görüntüdeki insanları tespit eder.
        En güvenilir sonucu işler ve ilgili takip fonksiyonlarını tetikler.
        """
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

            # Tespit edilen nesne "person" ise ve güven skoru eşiğin üzerindeyse
            if class_id == AI_MODEL['person_class_id'] and confidence > AI_MODEL['confidence_threshold']:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # En iyi tespiti seç
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
            bbox = best_person['bbox']
            center = best_person['center']

            self.human_distance = self.calculate_distance(best_person['height'])
            self.human_position = center
            roi_zone = self.detect_roi_zone(center)
            self.last_roi_zone = roi_zone

            # Tespit kutusunu ve bilgileri ekrana çiz
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.putText(frame, f"Distance: {self.human_distance}cm", (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

            # Kamera ve robot takip modları aktifse ilgili fonksiyonları çağır
            if self.camera_tracking:
                self.camera_track_human(center)

            if self.tracking_enabled:
                # Not: `track_human` metodu ana `RobotController` sınıfındadır.
                # Mixin sayesinde buradan çağrılabilir.
                self.track_human(center, self.human_distance, roi_zone)
        else:
            self.human_detected = False
            # İnsan bulunamadıysa ve kamera takibi aktifse, arama moduna geç
            if self.camera_tracking:
                current_time = time.time()
                if current_time - getattr(self, 'last_action_time', 0) > 3.0:
                    threading.Thread(target=self.camera_sweep_search, daemon=True).start()
                    self.last_action_time = current_time

        return frame

    def add_status_overlay(self, frame):
        """Görüntünün üzerine robotun anlık durumunu yazan bir katman ekler."""
        status_lines = [
            f"Robot Track: {'ON' if self.tracking_enabled else 'OFF'}",
            f"Camera Track: {'ON' if self.camera_tracking else 'OFF'}",
            f"Human: {'DETECTED' if self.human_detected else 'NOT FOUND'}",
            f"Distance: {self.human_distance} cm",
            f"ROI Zone: {self.last_roi_zone.upper()}",
            f"Power: {self.power_mode.upper()}",
            f"Walking: {'YES' if self.walking else 'NO'}"
        ]

        y0, dy = 20, 20
        for i, line in enumerate(status_lines):
            y = y0 + i * dy
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Kamera merkezini ve "Dead Zone" dairesini çiz
        cv2.circle(frame, (self.frame_center_x, self.frame_center_y), int(self.dead_zone_radius), (0, 255, 255), 1)

        return frame

    def draw_roi_zones(self, frame):
        """Yapılandırmadaki ROI bölgelerini ekrana çizer."""
        if not self.roi_enabled:
            return frame

        colors = {'center': (0, 255, 0), 'left': (255, 0, 0), 'right': (255, 0, 0), 'top': (0, 255, 255),
                  'bottom': (0, 255, 255)}

        for zone_name, zone in ROI_SETTINGS['zones'].items():
            color = colors.get(zone_name, (255, 255, 255))
            cv2.rectangle(frame, (zone['x'], zone['y']), (zone['x'] + zone['w'], zone['y'] + zone['h']), color, 1)

        return frame

    def camera_track_human(self, human_center):
        """Kameranın pan ve tilt servolarını hareket ettirerek insanı takip etmesini sağlar."""
        if not self.camera_tracking or self.camera_lock.locked():
            return

        with self.camera_lock:
            center_x, center_y = human_center

            # Merkeze olan uzaklığı hesapla
            distance_from_center = np.sqrt(
                (center_x - self.frame_center_x) ** 2 + (center_y - self.frame_center_y) ** 2)

            # Eğer hedef "dead zone" içindeyse hareket etme
            if distance_from_center <= self.dead_zone_radius:
                return

            # X ekseni (Pan) kontrolü
            if center_x < self.frame_center_x - self.dead_zone_radius:
                self.camera_pan_angle += self.camera_step_size
            elif center_x > self.frame_center_x + self.dead_zone_radius:
                self.camera_pan_angle -= self.camera_step_size

            # Y ekseni (Tilt) kontrolü
            if center_y < self.frame_center_y - self.dead_zone_radius:
                self.camera_tilt_angle -= self.camera_step_size
            elif center_y > self.frame_center_y + self.dead_zone_radius:
                self.camera_tilt_angle += self.camera_step_size

            # Hesaplanan yeni pozisyonu servolara uygula
            self.set_camera_position(self.camera_pan_angle, self.camera_tilt_angle, smooth=True)

    def set_camera_position(self, pan_angle=None, tilt_angle=None, smooth=False):
        """Kamera servolarını belirtilen açılara ayarlar."""
        # Bu metot `self.servos` ve `self.current_angles` gibi değişkenlere ihtiyaç duyar.
        # Bu değişkenler, bu sınıfı miras alan RobotController sınıfında tanımlıdır.

        # Pan açısını ayarla
        if pan_angle is not None:
            self.camera_pan_angle = max(CAMERA_SETTINGS['pan_min'], min(CAMERA_SETTINGS['pan_max'], pan_angle))
            if smooth:
                self.smooth_servo_move('camera_pan', self.camera_pan_angle)
            else:
                self.set_servo_angle('camera_pan', self.camera_pan_angle)

        # Tilt açısını ayarla
        if tilt_angle is not None:
            self.camera_tilt_angle = max(CAMERA_SETTINGS['tilt_min'], min(CAMERA_SETTINGS['tilt_max'], tilt_angle))
            if smooth:
                self.smooth_servo_move('camera_tilt', self.camera_tilt_angle)
            else:
                self.set_servo_angle('camera_tilt', self.camera_tilt_angle)

    def camera_sweep_search(self):
        """İnsan bulunamadığında kameranın etrafı taramasını sağlar."""
        if not self.camera_tracking: return
        print("[CAMERA] Human lost. Starting sweep search...")

        # Farklı pan pozisyonlarını tara
        positions = [90, 60, 120, 45, 135, 90]
        for pan_pos in positions:
            if self.human_detected:
                print("[CAMERA] Human re-acquired. Stopping search.")
                break
            self.set_camera_position(pan_angle=pan_pos, smooth=True)
            time.sleep(0.5)

        # Arama bittiğinde merkeze dön
        if not self.human_detected:
            self.set_camera_position(90, 90, smooth=True)