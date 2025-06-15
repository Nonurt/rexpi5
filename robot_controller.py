# robot_controller.py

import time
import threading
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# --- DONANIM KONTROLÜ ---
# Programın başında, gerçek bir Raspberry Pi üzerinde çalışıp çalışmadığını kontrol et.
IS_ON_RASPBERRY_PI = False
try:
    import RPi.GPIO as GPIO

    IS_ON_RASPBERRY_PI = True
    print("[INIT] Raspberry Pi detected. Real RPi.GPIO library will be used.")
except (ImportError, RuntimeError):
    print("[INIT] WARNING: Not running on a Raspberry Pi. GPIO functions will be simulated.")

# Projemizdeki diğer modülleri import ediyoruz
import config
from movement_gaits import MovementGaits
from camera_ai_handler import CameraAIHandler


# Ana Sınıf: Diğer modüllerdeki yetenekleri miras alır (inheritance).
class HumanTrackingServoController(MovementGaits, CameraAIHandler):
    """
    Robotun ana kontrol sınıfı. Donanımı başlatır, durumu yönetir
    ve tüm mantığı bir araya getirir.
    """

    def __init__(self):
        # --- Donanım Başlatma ---
        print("[INIT] Initializing I2C and PCA9685 Servo Controller...")
        self.pca = None
        if IS_ON_RASPBERRY_PI:
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(self.i2c)
                self.pca.frequency = config.I2C_FREQUENCY
                print("[INIT] PCA9685 Servo Controller configured successfully.")
            except Exception as e:
                print(f"[FATAL ERROR] Could not initialize PCA9685. Check I2C connection: {e}")
        else:
            print("[INIT] Skipping PCA9685 initialization (not on a Raspberry Pi).")

        if self.pca:
            print("[INIT] Configuring servos from config file...")
            self.servos = {name: servo.Servo(self.pca.channels[ch]) for name, ch in config.SERVO_CHANNELS.items()}
            for servo_obj in self.servos.values():
                servo_obj.set_pulse_width_range(*config.SERVO_PULSE_WIDTH_RANGE)
            print(f"[INIT] {len(self.servos)} servos configured.")
        else:
            self.servos = {}
            print("[WARNING] No servos configured as hardware is not available.")

        # --- LED Donanım Başlatma (Sadece Pi üzerindeyse) ---
        self.led_pwm = None
        if IS_ON_RASPBERRY_PI:
            print("[INIT] Initializing LED on GPIO pin...")
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(config.LED_SETTINGS['pin'], GPIO.OUT)
            self.led_pwm = GPIO.PWM(config.LED_SETTINGS['pin'], 100)
            self.led_pwm.start(0)
        else:
            print("[INIT] Skipping LED initialization (not on a Raspberry Pi).")

        self.led_brightness = 0
        self.led_enabled = False

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

        # --- PID Kontrol Değişkenleri ---
        print("[INIT] Initializing PID controller variables...")
        self.pan_kp = 0.04
        self.pan_ki = 0.008
        self.pan_kd = 0.015
        self.pan_integral = 0
        self.pan_last_error = 0
        self.tilt_kp = 0.04
        self.tilt_ki = 0.008
        self.tilt_kd = 0.015
        self.tilt_integral = 0
        self.tilt_last_error = 0
        self.last_pid_time = time.time()

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

        self.auto_gamma_enabled = False
        self.histogram_equalization_enabled = False

        # --- Diğer değişkenler ---
        self.human_detected = False
        self.human_position = None
        self.human_distance = 0
        self.frame_center_x = config.VIDEO['width'] // 2
        self.frame_center_y = config.VIDEO['height'] // 2
        self.last_action_time = 0
        self.distance_calibration = config.DISTANCE_CALIBRATION
        self.cap = None
        self.net = None
        self.is_camera_running = False
        self.current_frame = None

        # --- Başlatma Fonksiyonlarını Çağır ---
        self.init_ai_model()
        self.init_camera_position()
        print("[INIT] Controller initialized successfully.")

    def set_led_brightness(self, brightness):
        """LED parlaklığını ayarlar (Sadece Pi üzerindeyse)."""
        self.led_brightness = max(0, min(100, brightness))
        if self.led_enabled and self.led_pwm:
            self.led_pwm.ChangeDutyCycle(self.led_brightness)
        print(f"[LED] Brightness set to {self.led_brightness}%")

    def toggle_led(self, state=None):
        """LED'i açar/kapatır (Sadece Pi üzerindeyse)."""
        if state is not None:
            self.led_enabled = state
        else:
            self.led_enabled = not self.led_enabled

        if self.led_pwm:
            if self.led_enabled:
                current_brightness = self.led_brightness if self.led_brightness > 0 else config.LED_SETTINGS[
                    'manual_brightness']
                self.led_pwm.ChangeDutyCycle(current_brightness)
            else:
                self.led_pwm.ChangeDutyCycle(0)

        print(f"[LED] Turned {'ON' if self.led_enabled else 'OFF'}")
        return self.led_enabled

    def cleanup(self):
        """Uygulama kapanırken donanımları güvenle kapatır (Sadece Pi üzerindeyse)."""
        print("[SYSTEM] Cleaning up resources...")
        self.stop_camera()
        if IS_ON_RASPBERRY_PI:
            if self.led_pwm:
                self.led_pwm.stop()
            GPIO.cleanup()
            if self.pca:
                self.pca.deinit()
            print("[SYSTEM] Hardware cleanup complete.")
        else:
            print("[SYSTEM] Skipping hardware cleanup (not on a Raspberry Pi).")

    def get_power_settings(self):
        return self.power_settings[self.power_mode]

    def set_power_mode(self, mode):
        if mode in self.power_settings:
            self.power_mode = mode
            print(f"[POWER] Power mode changed: {self.power_settings[mode]['name']}")
            return True
        return False

    def track_human(self, human_center, distance, roi_zone):
        if self.walking:
            return
        current_time = time.time()
        settings = self.get_power_settings()
        if current_time - self.last_action_time < settings['cooldown']:
            return
        action = self.check_roi_distance_rules(distance, roi_zone)

        if action == "retreat":
            threading.Thread(target=self.rex_backward_gait, daemon=True).start()
        elif action == "stop":
            pass  # Hareket etme
        elif action == "turn":
            if human_center[0] < self.frame_center_x - 50:
                threading.Thread(target=self.rex_turn_left, daemon=True).start()
            elif human_center[0] > self.frame_center_x + 50:
                threading.Thread(target=self.rex_turn_right, daemon=True).start()
        elif action == "approach":
            if abs(human_center[0] - self.frame_center_x) < 100:
                threading.Thread(target=self.rex_forward_gait, daemon=True).start()
            elif human_center[0] < self.frame_center_x:
                threading.Thread(target=self.rex_turn_left, daemon=True).start()
            else:
                threading.Thread(target=self.rex_turn_right, daemon=True).start()

        if action != "stop":
            self.last_action_time = current_time

    def check_roi_distance_rules(self, distance, roi_zone):
        if distance <= self.roi_min_distance:
            return "retreat"
        elif distance <= self.roi_stop_distance:
            return "stop" if roi_zone == 'center' else "turn"
        else:
            return "approach"

    def detect_roi_zone(self, human_center):
        if not self.roi_enabled:
            return 'center'
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
        return int(max(cal['min_distance_cm'], min(cal['max_distance_cm'], distance)))

    def set_servo_angle(self, servo_name, angle):
        """Tek bir servonun açısını ayarlar (Sadece PCA9685 bağlıysa)."""
        if not self.pca: return False
        try:
            if servo_name in self.servos:
                angle = max(0, min(180, angle))
                self.servos[servo_name].angle = angle
                self.current_angles[servo_name] = angle
                return True
        except Exception as e:
            print(f"[ERROR] Servo '{servo_name}' could not be set: {e}")
        return False

    def smooth_servo_move(self, servo_name, target_angle, steps=5):
        if servo_name not in self.servos or not self.pca:
            return
        current_angle = self.current_angles.get(servo_name, 90)
        step_size = (target_angle - current_angle) / steps
        settings = self.get_power_settings()
        for i in range(steps):
            new_angle = current_angle + (step_size * (i + 1))
            self.set_servo_angle(servo_name, new_angle)
            time.sleep(settings['speed_delay'])

    # ... Geriye kalan tüm hareket fonksiyonları (rex_forward_gait vb.) bu sınıfta
    # ... MovementGaits'ten miras alındığı için mevcuttur ve çalışacaktır.
