import cv2
import time
import numpy as np
import threading
from config import VIDEO, AI_MODEL, CAMERA_SETTINGS, ROI_SETTINGS, CAMERA_TRACKING_SETTINGS, LED_SETTINGS

# --- LED KONTROLÜ: GPIO KÜTÜPHANESİ VE MOCK NESNE ---
try:
    from gpiozero import PWMLED

    IS_RPI = True
    print("[INFO] gpiozero library loaded. Running on Raspberry Pi.")
except (ImportError, RuntimeError):
    IS_RPI = False
    print("[WARN] gpiozero library not found. Running in mock mode (LED will not work).")


    class PWMLED:
        def __init__(self, pin):
            self.pin = pin
            self._value = 0
            print(f"[MOCK] PWMLED created for pin {self.pin}")

        @property
        def value(self):
            return self._value

        @value.setter
        def value(self, val):
            self._value = max(0.0, min(1.0, val))

        def close(self):
            print(f"[MOCK] LED on pin {self.pin} closed.")


# ----------------------------------------------------


class CameraAIHandler:
    """
    Kamera, AI, görüntü işleme, LED ve servo kontrolü için tüm metotları içeren bir "mixin" sınıfı.
    Bu sınıf tek başına çalışmaz, RobotController tarafından miras alınır.
    """

    # Görüntü İyileştirme Değişkenleri
    auto_gamma_enabled = False
    histogram_equalization_enabled = False

    # --- LED KONTROL DEĞİŞKENLERİ ---
    led_mode = 'off'  # Modlar: 'off', 'auto', 'manual'
    manual_led_brightness = LED_SETTINGS['manual_brightness'] / 100.0  # Değer: 0.0 ile 1.0 arası
    led_current_pwm = 0.0

    # -------------------------------

    def init_ai_model(self):
        """AI modelini config dosyasındaki yollardan yükler."""
        try:
            print("[INFO] Loading AI Model...")
            self.net = cv2.dnn.readNetFromCaffe(AI_MODEL['prototxt_path'], AI_MODEL['model_path'])
            print("[INFO] AI Model loaded successfully!")
        except Exception as e:
            print(f"[ERROR] AI Model could not be loaded: {e}")
            self.net = None

    # --- LED KONTROL FONKSİYONLARI ---
    def init_led(self):
        """LED pinini başlatır."""
        try:
            self.led = PWMLED(LED_SETTINGS['pin'])
            print(f"[INFO] LED initialized on GPIO pin {LED_SETTINGS['pin']}")
        except Exception as e:
            print(f"[ERROR] Failed to initialize LED: {e}")
            self.led = None

    def close_led(self):
        """LED'i güvenli bir şekilde kapatır."""
        if hasattr(self, 'led') and self.led:
            self.led.value = 0
            self.led.close()
            print("[INFO] LED resources released.")

    def set_led_mode(self, mode):
        """LED modunu ayarlar ('off', 'auto', 'manual')."""
        if mode in ['off', 'auto', 'manual']:
            self.led_mode = mode
            print(f"[LED] Mode set to: {self.led_mode.upper()}")
        return self.led_mode

    def set_led_brightness(self, brightness_percent):
        """Manuel LED parlaklığını ayarlar (%0-100)."""
        self.manual_led_brightness = max(0, min(100, brightness_percent)) / 100.0
        if self.led_mode == 'manual' and self.led:
            self.led.value = self.manual_led_brightness
            self.led_current_pwm = self.manual_led_brightness
        print(f"[LED] Manual brightness set to: {brightness_percent}%")
        return self.manual_led_brightness * 100

    def _update_led_brightness(self, frame):
        """LED moduna göre parlaklığı günceller."""
        if not hasattr(self, 'led') or not self.led:
            return

        target_pwm = 0.0
        if self.led_mode == 'auto':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mean_brightness = np.mean(gray)
            threshold = LED_SETTINGS['brightness_threshold']
            if mean_brightness < threshold:
                darkness_factor = (threshold - mean_brightness) / threshold
                target_pwm = min(1.0, darkness_factor ** 1.5)
        elif self.led_mode == 'manual':
            target_pwm = self.manual_led_brightness

        self.led.value = target_pwm
        self.led_current_pwm = target_pwm

    # ------------------------------------

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
        if self.cap:
            self.cap.release()
        print("[INFO] Camera stopped")

    def init_camera_position(self):
        """Kamera servolarını başlangıç pozisyonuna (merkez) ayarlar."""
        print("[CAMERA] Setting initial camera position...")
        self.set_camera_position(pan_angle=90, tilt_angle=90, smooth=False)
        time.sleep(1)
        print("[CAMERA] Camera is in center position.")

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
        if mean_brightness < 100:
            gamma = 1.5
            inv_gamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
            return cv2.LUT(frame, table)
        return frame

    def _apply_histogram_equalization(self, frame):
        """Görüntü kontrastını artırmak için histogram eşitleme uygular."""
        yuv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        yuv_img[:, :, 0] = cv2.equalizeHist(yuv_img[:, :, 0])
        return cv2.cvtColor(yuv_img, cv2.COLOR_YUV2BGR)

    def camera_loop(self):
        """Sürekli olarak kameradan görüntü okuyan, işleyen ve saklayan ana döngü."""
        while self.is_camera_running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                # Her döngüde LED'i güncelle
                self._update_led_brightness(frame)

                if self.auto_gamma_enabled:
                    frame = self._apply_auto_gamma(frame)

                if self.histogram_equalization_enabled:
                    frame = self._apply_histogram_equalization(frame)

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
                    best_person = {'bbox': (startX, startY, endX, endY), 'confidence': confidence,
                                   'center': ((startX + endX) // 2, (startY + endY) // 2), 'height': endY - startY}

        if best_person:
            self.human_detected = True
            self.last_detection_time = time.time()
            center = best_person['center']
            # ... (diğer atamalar)
            if self.camera_tracking:
                self.camera_track_human(center)
            # ...
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
        status_lines = [
            f"Robot Track: {'ON' if getattr(self, 'tracking_enabled', False) else 'OFF'}",
            f"Camera Track: {'ON' if getattr(self, 'camera_tracking', False) else 'OFF'}",
            f"Human: {'DETECTED' if getattr(self, 'human_detected', False) else 'NOT FOUND'}",
            f"Power: {getattr(self, 'power_mode', 'N/A').upper()}",
            f"Walking: {'YES' if getattr(self, 'walking', False) else 'NO'}",
            "--- Image Enhancement ---",
            f"Auto Gamma: {'ON' if self.auto_gamma_enabled else 'OFF'}",
            f"Histogram Eq: {'ON' if self.histogram_equalization_enabled else 'OFF'}",
            "--- LED Status ---",
            f"LED Mode: {self.led_mode.upper()}",
            f"LED Brightness: {self.led_current_pwm * 100:.1f}%"
        ]

        y0, dy = 15, 18
        for i, line in enumerate(status_lines):
            y = y0 + i * dy
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        dead_zone_radius = CAMERA_TRACKING_SETTINGS['dead_zone_radius']
        cv2.circle(frame, (self.frame_center_x, self.frame_center_y), int(dead_zone_radius), (0, 255, 255), 1)
        return frame

    def draw_roi_zones(self, frame):
        if not getattr(self, 'roi_enabled', False):
            return frame
        colors = {'center': (0, 255, 0), 'left': (255, 0, 0), 'right': (255, 0, 0), 'top': (0, 255, 255),
                  'bottom': (0, 255, 255)}
        for zone_name, zone in ROI_SETTINGS['zones'].items():
            color = colors.get(zone_name, (255, 255, 255))
            cv2.rectangle(frame, (zone['x'], zone['y']), (zone['x'] + zone['w'], zone['y'] + zone['h']), color, 1)
        return frame

    def camera_track_human(self, human_center):
        if not getattr(self, 'camera_tracking', False) or (hasattr(self, 'camera_lock') and self.camera_lock.locked()):
            return
        with self.camera_lock:
            # ... (izleme mantığı)
            pass

    def set_camera_position(self, pan_angle=None, tilt_angle=None, smooth=False):
        # ... (servo ayarlama mantığı)
        pass

    def camera_sweep_search(self):
        # ... (tarama mantığı)
        pass