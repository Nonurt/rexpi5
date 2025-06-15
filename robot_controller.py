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


# Ana Sınıf: Diğer modüllerdeki yetenekleri miras alır (inheritance).
# Bu sayede bu sınıf hem hareket (MovementGaits) hem de kamera (CameraAIHandler)
# fonksiyonlarını kendi metotlarıymış gibi kullanabilir.
class HumanTrackingServoController(MovementGaits, CameraAIHandler):
    """
    Robotun ana kontrol sınıfı. Donanımı başlatır, durumu yönetir
    ve tüm mantığı bir araya getirir.
    """

    def __init__(self):
        # --- Donanım Başlatma ---
        print("[INIT] Initializing I2C and PCA9685 Servo Controller...")
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = config.I2C_FREQUENCY

        print("[INIT] Configuring servos from config file...")
        self.servos = {name: servo.Servo(self.pca.channels[ch]) for name, ch in config.SERVO_CHANNELS.items()}
        for servo_obj in self.servos.values():
            servo_obj.set_pulse_width_range(*config.SERVO_PULSE_WIDTH_RANGE)
        print(f"[INIT] {len(self.servos)} servos configured.")

        # --- Robot Durum Değişkenleri ---
        self.walking = False
        self.walking_lock = threading.Lock()  # Hareketlerin çakışmasını önleyen kilit
        self.current_angles = {name: 90 for name in self.servos.keys()}

        # --- Kamera ve Takip Durum Değişkenleri ---
        self.tracking_enabled = False  # Robotun fiziksel takibi (yürüme)
        self.camera_tracking = False  # Kameranın kafa takibi
        self.camera_lock = threading.Lock()
        self.camera_pan_angle = 90
        self.camera_tilt_angle = 90

        # --- Ayarları config dosyasından yükle ---
        print("[INIT] Loading settings from config.py...")
        self.camera_smooth_delay = config.CAMERA_SETTINGS['smooth_delay']
        self.dead_zone_radius = config.CAMERA_SETTINGS['dead_zone_radius']
        self.camera_step_size = config.CAMERA_SETTINGS['step_size']
        self.camera_limits = {k: v for k, v in config.CAMERA_SETTINGS.items() if 'min' in k or 'max' in k}

        self.roi_enabled = config.ROI_SETTINGS['enabled']
        self.roi_stop_distance = config.ROI_SETTINGS['stop_distance']
        self.roi_min_distance = config.ROI_SETTINGS['min_distance']
        self.roi_zones = config.ROI_SETTINGS['zones']
        self.last_roi_zone = 'center'

        self.power_mode = "medium"
        self.power_settings = config.POWER_SETTINGS

        # --- İnsan Takip Sistemi Değişkenleri ---
        self.human_detected = False
        self.human_position = None
        self.human_distance = 0
        self.frame_center_x = config.VIDEO['width'] // 2
        self.frame_center_y = config.VIDEO['height'] // 2
        self.last_action_time = 0

        self.distance_calibration = config.DISTANCE_CALIBRATION

        # --- Kamera ve AI Nesneleri ---
        self.cap = None
        self.net = None
        self.is_camera_running = False
        self.current_frame = None

        # --- Başlatma Fonksiyonlarını Çağır ---
        # Bu metotlar miras alınan CameraAIHandler sınıfından gelir.
        self.init_ai_model()
        self.init_camera_position()
        print("[INIT] Controller initialized successfully.")

    def get_power_settings(self):
        """Mevcut güç modunun ayarlarını döndürür."""
        return self.power_settings[self.power_mode]

    def set_power_mode(self, mode):
        """Güç modunu değiştirir (low, medium, high, max)."""
        if mode in self.power_settings:
            self.power_mode = mode
            settings = self.get_power_settings()
            print(f"[POWER] Power mode changed: {settings['name']}")
            print(f"[POWER] {settings['description']}")
            return True
        return False

    def track_human(self, human_center, distance, roi_zone):
        """Ana insan takip mantığı. ROI kurallarına göre hareket kararı verir."""
        if self.walking:
            return  # Zaten bir hareket varsa yenisini başlatma

        current_time = time.time()
        settings = self.get_power_settings()

        # Her hareket arasında bekleme süresi (cooldown)
        if current_time - self.last_action_time < settings['cooldown']:
            return

        # Mesafeye ve bölgeye göre yapılacak eylemi belirle
        action = self.check_roi_distance_rules(distance, roi_zone)
        print(f"[TRACKING] Action: {action}, Distance: {distance}cm, ROI: {roi_zone}")

        # Eyleme göre uygun hareket fonksiyonunu çağır (Bu fonksiyonlar MovementGaits'ten gelir)
        if action == "retreat":
            threading.Thread(target=self.rex_backward_gait, daemon=True).start()
        elif action == "stop":
            print("[TRACKING] Stopping - Target in center and at a good distance.")
        elif action == "turn":
            # Hedefe doğru dön
            if human_center[0] < self.frame_center_x - 50:
                threading.Thread(target=self.rex_turn_left, daemon=True).start()
            elif human_center[0] > self.frame_center_x + 50:
                threading.Thread(target=self.rex_turn_right, daemon=True).start()
        elif action == "approach":
            # Hedef merkezdeyse ilerle, değilse dön
            if abs(human_center[0] - self.frame_center_x) < 100:
                threading.Thread(target=self.rex_forward_gait, daemon=True).start()
            elif human_center[0] < self.frame_center_x:
                threading.Thread(target=self.rex_turn_left, daemon=True).start()
            else:
                threading.Thread(target=self.rex_turn_right, daemon=True).start()

        self.last_action_time = current_time

    def check_roi_distance_rules(self, distance, roi_zone):
        """ROI mesafe kurallarını kontrol eder ve eylem belirler."""
        # Minimum güvenli mesafeden daha yakınsa geri çekil
        if distance <= self.roi_min_distance:
            return "retreat"

        # Durma mesafesindeyse
        elif distance <= self.roi_stop_distance:
            # ve merkez bölgedeyse dur
            if roi_zone == 'center':
                return "stop"
            # değilse merkeze doğru dön
            else:
                return "turn"

        # Normal takip mesafesindeyse yaklaş
        else:
            return "approach"

    def detect_roi_zone(self, human_center):
        """Verilen koordinatın hangi ROI bölgesinde olduğunu tespit eder."""
        if not self.roi_enabled:
            return 'center'

        x, y = human_center

        # Öncelik sırasına göre bölgeleri kontrol et
        sorted_zones = sorted(self.roi_zones.items(), key=lambda item: item[1]['priority'])

        for zone_name, zone in sorted_zones:
            if (zone['x'] <= x <= zone['x'] + zone['w'] and
                    zone['y'] <= y <= zone['y'] + zone['h']):
                return zone_name

        return 'center'  # Varsayılan bölge

    def calculate_distance(self, bbox_height_px):
        """Tespit edilen insanın bounding box yüksekliğinden mesafesini tahmin eder."""
        if bbox_height_px <= 0:
            return 0

        cal = self.distance_calibration
        # Basit üçgen benzerliği formülü
        distance = (cal['person_height_cm'] * cal['camera_focal_length']) / bbox_height_px

        # Mesafeyi belirlenen min/max aralığında sınırla
        distance = max(cal['min_distance_cm'], min(cal['max_distance_cm'], distance))

        return int(distance)

    def set_servo_angle(self, servo_name, angle):
        """Tek bir servonun açısını güvenli bir şekilde ayarlar."""
        try:
            if servo_name in self.servos:
                # Açıyı 0-180 derece arasında sınırla
                angle = max(0, min(180, angle))
                self.servos[servo_name].angle = angle
                self.current_angles[servo_name] = angle
                return True
        except Exception as e:
            # Genellikle I2C bağlantı hatalarında bu hata oluşur
            print(f"[ERROR] Servo '{servo_name}' could not be set: {e}")
            return False

    def smooth_servo_move(self, servo_name, target_angle, steps=5):
        """Bir servoyu mevcut açısından hedef açıya yumuşak bir şekilde hareket ettirir."""
        if servo_name not in self.servos:
            return

        current_angle = self.current_angles.get(servo_name, 90)
        # Hedef açıya ulaşmak için adım başına ne kadar hareket edileceğini hesapla
        step_size = (target_angle - current_angle) / steps
        settings = self.get_power_settings()

        for i in range(steps):
            new_angle = current_angle + (step_size * (i + 1))
            self.set_servo_angle(servo_name, new_angle)
            # Adımlar arasında güç moduna göre bekle
            time.sleep(settings['speed_delay'])