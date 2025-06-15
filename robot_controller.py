# robot_controller.py
import time
import threading
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Modüler importlar
import config
from movement_gaits import MovementGaits
from camera_ai_handler import CameraAIHandler


# Mixin'leri miras alarak ana sınıfı oluştur
class HumanTrackingServoController(MovementGaits, CameraAIHandler):
    def __init__(self):
        # I2C ve Servo donanımını başlat
        print("[INIT] Initializing I2C and PCA9685 Servo Controller...")
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = config.I2C_FREQUENCY

        print(f"[INIT] Configuring {len(config.SERVO_CHANNELS)} servos...")
        self.servos = {name: servo.Servo(self.pca.channels[ch]) for name, ch in config.SERVO_CHANNELS.items()}
        for servo_obj in self.servos.values():
            servo_obj.set_pulse_width_range(*config.SERVO_PULSE_WIDTH_RANGE)

        # Robot Durum Değişkenleri
        self.walking = False
        self.walking_lock = threading.Lock()
        self.current_angles = {name: 90 for name in self.servos.keys()}

        # Kamera ve Takip Durum Değişkenleri
        self.tracking_enabled = False
        self.camera_tracking = False
        self.camera_lock = threading.Lock()
        self.camera_pan_angle = 90
        self.camera_tilt_angle = 90

        # Ayarları config dosyasından al
        print("[INIT] Loading settings from config.py...")
        self.dead_zone_radius = config.CAMERA_SETTINGS['dead_zone_radius']
        self.camera_limits = {k: v for k, v in config.CAMERA_SETTINGS.items() if 'min' in k or 'max' in k}

        # YENİ P-Controller ayarlarını yükle
        self.p_gain_pan = config.CAMERA_SETTINGS['p_gain_pan']
        self.p_gain_tilt = config.CAMERA_SETTINGS['p_gain_tilt']
        self.max_adjustment = config.CAMERA_SETTINGS['max_adjustment']

        self.roi_enabled = config.ROI_SETTINGS['enabled']
        self.roi_stop_distance = config.ROI_SETTINGS['stop_distance']
        self.roi_min_distance = config.ROI_SETTINGS['min_distance']
        self.roi_zones = config.ROI_SETTINGS['zones']
        self.last_roi_zone = 'center'

        self.power_mode = "medium"
        self.power_settings = config.POWER_SETTINGS

        # İnsan Takip Sistemi Değişkenleri
        self.human_detected = False
        self.human_position = None
        self.human_distance = 0
        self.frame_center_x = config.VIDEO['width'] // 2
        self.frame_center_y = config.VIDEO['height'] // 2
        self.last_action_time = 0

        self.distance_calibration = config.DISTANCE_CALIBRATION

        # Kamera ve AI
        self.cap = None
        self.net = None
        self.is_camera_running = False
        self.current_frame = None

        # Başlatma fonksiyonlarını çağır
        self.init_ai_model()
        self.init_camera_position()
        print("[INIT] Controller initialized successfully.")

    def get_power_settings(self):
        return self.power_settings[self.power_mode]

    def set_power_mode(self, mode):
        if mode in self.power_settings:
            self.power_mode = mode
            settings = self.get_power_settings()
            print(f"[POWER] Power mode changed: {settings['name']}")
            return True
        return False

    def track_human(self, human_center, distance, roi_zone):
        if self.walking: return
        current_time = time.time()
        settings = self.get_power_settings()
        if current_time - self.last_action_time < settings['cooldown']: return

        action = self.check_roi_distance_rules(distance, roi_zone)
        print(f"[TRACKING] Action: {action}, Distance: {distance}cm, ROI: {roi_zone}")

        action_thread = None
        if action == "retreat":
            action_thread = threading.Thread(target=self.rex_backward_gait, daemon=True)
        elif action == "stop":
            print("[TRACKING] Stopping - Target in center at a good distance.")
        elif action == "turn":
            if human_center[0] < self.frame_center_x:
                action_thread = threading.Thread(target=self.rex_turn_left, daemon=True)
            else:
                action_thread = threading.Thread(target=self.rex_turn_right, daemon=True)
        elif action == "approach":
            if abs(human_center[0] - self.frame_center_x) < 100:
                action_thread = threading.Thread(target=self.rex_forward_gait, daemon=True)
            elif human_center[0] < self.frame_center_x:
                action_thread = threading.Thread(target=self.rex_turn_left, daemon=True)
            else:
                action_thread = threading.Thread(target=self.rex_turn_right, daemon=True)

        if action_thread:
            action_thread.start()
            self.last_action_time = current_time

    def check_roi_distance_rules(self, distance, roi_zone):
        if distance <= self.roi_min_distance:
            return "retreat"
        elif distance <= self.roi_stop_distance:
            return "stop" if roi_zone == 'center' else "turn"
        else:
            return "approach"

    def detect_roi_zone(self, human_center):
        if not self.roi_enabled: return 'center'
        x, y = human_center
        sorted_zones = sorted(self.roi_zones.items(), key=lambda item: item[1]['priority'])
        for zone_name, zone in sorted_zones:
            if (zone['x'] <= x <= zone['x'] + zone['w'] and
                    zone['y'] <= y <= zone['y'] + zone['h']):
                return zone_name
        return 'center'

    def calculate_distance(self, bbox_height_px):
        if bbox_height_px <= 0: return 0
        cal = self.distance_calibration
        distance = (cal['person_height_cm'] * cal['camera_focal_length']) / bbox_height_px
        distance = max(cal['min_distance_cm'], min(cal['max_distance_cm'], distance))
        return int(distance)

    def set_servo_angle(self, servo_name, angle):
        try:
            if servo_name in self.servos:
                angle = max(0, min(180, angle))
                self.servos[servo_name].angle = angle
                self.current_angles[servo_name] = angle
                return True
        except Exception as e:
            print(f"[ERROR] Servo '{servo_name}' could not be set: {e}")
        return False

    def smooth_servo_move(self, servo_name, target_angle, steps=10, delay=0.02):
        if servo_name not in self.servos: return
        current_angle = self.current_angles.get(servo_name, 90)
        step_size = (target_angle - current_angle) / steps
        for i in range(steps):
            new_angle = current_angle + (step_size * (i + 1))
            self.set_servo_angle(servo_name, new_angle)
            time.sleep(delay)