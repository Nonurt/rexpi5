# robot_controller.py

import time
import threading
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Projemizdeki diğer modülleri import ediyoruz
import config
from movement_gaits import MovementGaits
from camera_ai_handler import CameraAIHandler


# DÜZELTME: Sınıf adı app.py ile uyumlu olacak şekilde "RobotController" olarak değiştirildi.
class RobotController(MovementGaits, CameraAIHandler):
    """
    Robotun ana kontrol sınıfı. Donanımı başlatır, durumu yönetir
    ve tüm mantığı bir araya getirir.
    """

    def __init__(self):
        # --- Donanım Başlatma ---
        print("[INIT] Initializing I2C and PCA9685 Servo Controller...")
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = config.I2C_FREQUENCY
            self.servos = {name: servo.Servo(self.pca.channels[ch]) for name, ch in config.SERVO_CHANNELS.items()}
            for servo_obj in self.servos.values():
                servo_obj.set_pulse_width_range(*config.SERVO_PULSE_WIDTH_RANGE)
            print(f"[INIT] {len(self.servos)} servos configured.")
        except Exception as e:
            print(f"[FATAL ERROR] Failed to initialize hardware (PCA9685). Check I2C connection: {e}")
            # Donanım olmadan programın devam etmemesi için çıkış yapabiliriz.
            exit()

        # --- Robot Durum Değişkenleri ---
        self.walking = False
        self.walking_lock = threading.Lock()
        self.current_angles = {name: 90 for name in self.servos.keys()}

        # --- Kamera ve Takip Durum Değişkenleri ---
        self.tracking_enabled = False
        self.camera_tracking = False
        self.camera_lock = threading.Lock()
        self.camera_pan_angle = 90
        self.camera_tilt_angle = 90

        # --- Ayarları config dosyasından yükle ---
        print("[INIT] Loading settings from config.py...")
        self.power_mode = "medium"
        self.power_settings = config.POWER_SETTINGS
        self.frame_center_x = config.VIDEO['width'] // 2
        self.frame_center_y = config.VIDEO['height'] // 2

        # --- İnsan Takip Sistemi Değişkenleri ---
        self.human_detected = False
        self.human_position = None
        self.human_distance = 0
        self.last_roi_zone = 'center'
        self.last_action_time = 0

        # --- Kamera ve AI Nesneleri ---
        self.cap = None
        self.net = None
        self.is_camera_running = False
        self.current_frame = None

        # --- Başlatma Fonksiyonlarını Çağır ---
        # Bu metotlar miras alınan sınıflardan gelir.
        self.init_ai_model()
        self.init_camera_position()

        # YENİ EKLENEN: LED başlatma fonksiyonu çağrılıyor.
        self.init_led()

        print("[INIT] Controller initialized successfully.")

    def get_power_settings(self):
        """Mevcut güç modunun ayarlarını döndürür."""
        return self.power_settings.get(self.power_mode, self.power_settings['medium'])

    def set_power_mode(self, mode):
        """Güç modunu değiştirir (low, medium, high, max)."""
        if mode in self.power_settings:
            self.power_mode = mode
            settings = self.get_power_settings()
            print(f"[POWER] Power mode changed: {settings['name']}")
            return True
        return False

    def track_human(self, human_center, distance, roi_zone):
        """Ana insan takip mantığı. ROI kurallarına göre hareket kararı verir."""
        if self.walking or not self.tracking_enabled:
            return

        current_time = time.time()
        settings = self.get_power_settings()

        if current_time - self.last_action_time < settings['cooldown']:
            return

        action = self.check_roi_distance_rules(distance, roi_zone)

        target_func = None
        if action == "retreat":
            target_func = self.rex_backward_gait
        elif action == "turn":
            if human_center[0] < self.frame_center_x:
                target_func = self.rex_turn_left
            else:
                target_func = self.rex_turn_right
        elif action == "approach":
            if abs(human_center[0] - self.frame_center_x) < 100:
                target_func = self.rex_forward_gait
            elif human_center[0] < self.frame_center_x:
                target_func = self.rex_turn_left
            else:
                target_func = self.rex_turn_right

        if target_func:
            print(f"[TRACKING] Action: {action}, Distance: {distance}cm, ROI: {roi_zone}")
            threading.Thread(target=target_func, daemon=True).start()
            self.last_action_time = current_time

    def check_roi_distance_rules(self, distance, roi_zone):
        """ROI mesafe kurallarını kontrol eder ve eylem belirler."""
        if distance <= config.ROI_SETTINGS['min_distance']:
            return "retreat"
        elif distance <= config.ROI_SETTINGS['stop_distance']:
            return "stop" if roi_zone == 'center' else "turn"
        else:
            return "approach"

    def detect_roi_zone(self, human_center):
        """Verilen koordinatın hangi ROI bölgesinde olduğunu tespit eder."""
        x, y = human_center
        sorted_zones = sorted(config.ROI_SETTINGS['zones'].items(), key=lambda item: item[1]['priority'])
        for zone_name, zone in sorted_zones:
            if (zone['x'] <= x <= zone['x'] + zone['w'] and zone['y'] <= y <= zone['y'] + zone['h']):
                return zone_name
        return 'center'

    def calculate_distance(self, bbox_height_px):
        """Bounding box yüksekliğinden mesafeyi tahmin eder."""
        if bbox_height_px <= 0: return 0
        cal = config.DISTANCE_CALIBRATION
        distance = (cal['person_height_cm'] * cal['camera_focal_length']) / bbox_height_px
        return int(max(cal['min_distance_cm'], min(cal['max_distance_cm'], distance)))

    def set_servo_angle(self, servo_name, angle):
        """Tek bir servonun açısını ayarlar."""
        try:
            if servo_name in self.servos:
                angle = max(0, min(180, angle))
                self.servos[servo_name].angle = angle
                self.current_angles[servo_name] = angle
        except Exception as e:
            print(f"[ERROR] Servo '{servo_name}' could not be set: {e}")

    def smooth_servo_move(self, servo_name, target_angle, steps=5):
        """Bir servoyu yumuşak bir şekilde hareket ettirir."""
        if servo_name not in self.servos: return
        current_angle = self.current_angles.get(servo_name, 90)
        step_size = (target_angle - current_angle) / steps
        for i in range(steps):
            self.set_servo_angle(servo_name, current_angle + (step_size * (i + 1)))
            time.sleep(self.get_power_settings()['speed_delay'])

    # YENİ EKLENEN: Acil durum durdurma mantığı
    def emergency_stop(self):
        """Tüm aktif işlemleri durdurur ve robotu güvenli pozisyona alır."""
        print("[EMERGENCY] Emergency stop triggered!")
        self.tracking_enabled = False
        self.camera_tracking = False
        self.walking = True  # Yeni hareketlerin başlamasını engelle
        # Tüm hareket threadleri bittikten sonra duruşa geç
        time.sleep(1.5)
        self.walking = False
        self.rex_stabilize()

    # YENİ EKLENEN: Güvenli kapatma için kaynakları temizleme fonksiyonu
    def cleanup(self):
        """Program sonlandığında tüm kaynakları temizler."""
        print("[CLEANUP] Cleanup sequence started.")
        self.stop_camera()
        self.close_led()  # Bu fonksiyon CameraAIHandler'dan gelir

        # Tüm servoları boşa al (isteğe bağlı ama önerilir)
        print("[CLEANUP] Releasing all servos.")
        self.pca.deinit()

        print("[CLEANUP] Cleanup finished.")