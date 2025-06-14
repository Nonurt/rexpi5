# === FLASK WEB FRAMEWORK ===
from flask import Flask, render_template, request, jsonify, Response

# === ZAMANLAMA, MATEMATİK, THREAD ===
import time
import threading
import math
import json

# === I2C & SERVO MOTOR ===
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# === GÖRÜNTÜ İŞLEME & YAPAY ZEKA ===
import cv2
import numpy as np

app = Flask(__name__)


class HumanTrackingServoController:
    def __init__(self):
        # Initialize I2C connection
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50  # Servo frequency (50Hz)

        # Define servo channels (LEGS + CAMERA)
        self.servos = {
            # Leg servos
            'front_left_axis': servo.Servo(self.pca.channels[0]),
            'front_left_lift': servo.Servo(self.pca.channels[1]),
            'rear_left_axis': servo.Servo(self.pca.channels[2]),
            'rear_left_lift': servo.Servo(self.pca.channels[3]),
            'rear_right_axis': servo.Servo(self.pca.channels[4]),
            'rear_right_lift': servo.Servo(self.pca.channels[5]),
            'front_right_axis': servo.Servo(self.pca.channels[6]),
            'front_right_lift': servo.Servo(self.pca.channels[7]),
            # CAMERA SERVOS (pins 8 and 9)
            'camera_pan': servo.Servo(self.pca.channels[8]),  # PIN 8: Left-Right (Horizontal)
            'camera_tilt': servo.Servo(self.pca.channels[9])  # PIN 9: Up-Down (Vertical)
        }

        # SG90 Servo angle limits
        for servo_name, servo_obj in self.servos.items():
            servo_obj.set_pulse_width_range(600, 2400)

        # Robot status
        self.walking = False
        self.walking_lock = threading.Lock()
        self.current_angles = {name: 90 for name in self.servos.keys()}

        # CAMERA SERVO SETTINGS
        self.camera_tracking = False
        self.camera_lock = threading.Lock()
        self.camera_pan_angle = 90  # Center position
        self.camera_tilt_angle = 90  # Center position

        # *** YENİ CAMERA CONTROL SETTINGS ***
        self.camera_smooth_delay = 0.05  # Smooth movement delay (ayarlanabilir 0.01-0.2)
        self.dead_zone_radius = 60  # Dead zone radius (ayarlanabilir 20-150)
        self.camera_step_size = 3  # Camera movement step size (ayarlanabilir 1-10)

        self.camera_limits = {
            'pan_min': 20,  # Left limit
            'pan_max': 160,  # Right limit
            'tilt_min': 45,  # Down limit
            'tilt_max': 135  # Up limit
        }

        # ADVANCED ROI (Region of Interest) Settings - Adjustable distance
        self.roi_enabled = True
        self.roi_stop_distance = 80  # cm - Stop at this distance (adjustable from interface)
        self.roi_min_distance = 50  # cm - Minimum safe distance
        self.roi_zones = {
            'center': {'x': 220, 'y': 160, 'w': 200, 'h': 160, 'priority': 1},
            'left': {'x': 0, 'y': 120, 'w': 220, 'h': 240, 'priority': 2},
            'right': {'x': 420, 'y': 120, 'w': 220, 'h': 240, 'priority': 2},
            'top': {'x': 160, 'y': 0, 'w': 320, 'h': 160, 'priority': 3},
            'bottom': {'x': 160, 'y': 320, 'w': 320, 'h': 160, 'priority': 3}
        }
        self.last_roi_zone = 'center'

        # *** ADVANCED POWER MODE SYSTEM - SMOOTHER WALKING ***
        self.power_mode = "medium"
        self.power_settings = {
            "low": {
                "name": "Low Power",
                "description": "Energy efficient, slow movement",
                "lift_range": 30,
                "axis_range": 30,
                "speed_delay": 0.08,  # Faster
                "step_delay": 0.4,  # Faster
                "cooldown": 2.0,
                "camera_speed": 3,
                "movement_speed": 0.05  # Inter-movement wait
            },
            "medium": {
                "name": "Medium Power",
                "description": "Balanced performance",
                "lift_range": 35,
                "axis_range": 40,
                "speed_delay": 0.05,  # Faster
                "step_delay": 0.25,  # Faster
                "cooldown": 1.5,
                "camera_speed": 4,
                "movement_speed": 0.03
            },
            "high": {
                "name": "High Power",
                "description": "Fast and powerful movement",
                "lift_range": 45,
                "axis_range": 50,
                "speed_delay": 0.03,  # Very fast
                "step_delay": 0.15,  # Very fast
                "cooldown": 1.0,
                "camera_speed": 6,
                "movement_speed": 0.02
            },
            "max": {
                "name": "Maximum Power",
                "description": "Most powerful movement - SUPER FAST",
                "lift_range": 55,
                "axis_range": 60,
                "speed_delay": 0.01,  # Minimum wait
                "step_delay": 0.08,  # Minimum wait
                "cooldown": 0.5,
                "camera_speed": 8,
                "movement_speed": 0.01  # Almost instant
            }
        }

        # Human tracking system
        self.tracking_enabled = False
        self.human_detected = False
        self.human_position = None
        self.human_distance = 0
        self.frame_center_x = 320
        self.frame_center_y = 240
        self.last_action_time = 0

        # Distance calculation
        self.distance_calibration = {
            'person_height_cm': 170,
            'camera_focal_length': 600,
            'min_distance_cm': 50,
            'max_distance_cm': 300,
            'target_distance_cm': 100
        }

        # Camera and AI model
        self.cap = None
        self.net = None
        self.is_camera_running = False
        self.current_frame = None

        # AI Model file paths
        self.prototxt_path = "/home/legendeltax/servo_control/models/MobileNetSSD_deploy.prototxt"
        self.model_path = "/home/legendeltax/servo_control/models/MobileNetSSD_deploy.caffemodel"

        # Class names
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]

        self.init_ai_model()
        self.init_camera_position()  # Camera initial position

    def init_camera_position(self):
        """Camera servo initial position"""
        print("[CAMERA] Camera servo initial position...")
        self.set_servo_angle('camera_pan', self.camera_pan_angle)
        self.set_servo_angle('camera_tilt', self.camera_tilt_angle)
        time.sleep(1)
        print("[CAMERA] Camera in center position")

    # SATIR 158'deki mevcut fonksiyonu SİL ve şununla DEĞİŞTİR:

    def set_camera_position(self, pan_angle=None, tilt_angle=None, smooth=True):
        """Set camera position - İyileştirilmiş smooth servo kontrolü"""
        limits = self.camera_limits

        if pan_angle is not None:
            pan_angle = max(limits['pan_min'], min(limits['pan_max'], pan_angle))

            if smooth:
                # SMOOTH MOVEMENT - Kademeli hareket
                current_pan = self.camera_pan_angle
                step_count = max(3, abs(pan_angle - current_pan) // 5)  # Büyük hareketlerde daha çok adım

                for i in range(int(step_count)):
                    intermediate_angle = current_pan + ((pan_angle - current_pan) * (i + 1) / step_count)
                    try:
                        self.servos['camera_pan'].angle = intermediate_angle
                        time.sleep(self.camera_smooth_delay)
                    except Exception as e:
                        print(f"[ERROR] Pan servo smooth movement error: {e}")
                        break
            else:
                # FAST MOVEMENT - Direkt hareket
                try:
                    self.servos['camera_pan'].angle = pan_angle
                except Exception as e:
                    print(f"[ERROR] Pan servo error: {e}")

            self.camera_pan_angle = pan_angle
            print(f"[CAMERA] Pan servo moved to: {pan_angle}° ({'smooth' if smooth else 'fast'})")

        if tilt_angle is not None:
            tilt_angle = max(limits['tilt_min'], min(limits['tilt_max'], tilt_angle))

            if smooth:
                # SMOOTH MOVEMENT - Kademeli hareket
                current_tilt = self.camera_tilt_angle
                step_count = max(3, abs(tilt_angle - current_tilt) // 5)

                for i in range(int(step_count)):
                    intermediate_angle = current_tilt + ((tilt_angle - current_tilt) * (i + 1) / step_count)
                    try:
                        self.servos['camera_tilt'].angle = intermediate_angle
                        time.sleep(self.camera_smooth_delay)
                    except Exception as e:
                        print(f"[ERROR] Tilt servo smooth movement error: {e}")
                        break
            else:
                # FAST MOVEMENT - Direkt hareket
                try:
                    self.servos['camera_tilt'].angle = tilt_angle
                except Exception as e:
                    print(f"[ERROR] Tilt servo error: {e}")

            self.camera_tilt_angle = tilt_angle
            print(f"[CAMERA] Tilt servo moved to: {tilt_angle}° ({'smooth' if smooth else 'fast'})")

        print(f"[CAMERA] Position - Pan: {self.camera_pan_angle}°, Tilt: {self.camera_tilt_angle}°")

    # SATIR 175'teki mevcut fonksiyonu SİL ve şununla DEĞİŞTİR:

    def camera_track_human(self, human_center):
        """Camera human tracking - Dead Zone versiyonu"""
        if not self.camera_tracking:
            return

        settings = self.get_power_settings()
        camera_speed = self.camera_step_size  # Ayarlanabilir hız kullan

        center_x, center_y = human_center
        frame_w, frame_h = 640, 480

        # DEAD ZONE - Ayarlanabilir radius kullan
        dead_zone_radius = self.dead_zone_radius  # Artık ayarlanabilir!
        frame_center_x = frame_w // 2  # 320
        frame_center_y = frame_h // 2  # 240

        # İnsanın merkez noktasının frame merkezine uzaklığı
        distance_from_center = ((center_x - frame_center_x) ** 2 + (center_y - frame_center_y) ** 2) ** 0.5

        # DEAD ZONE içindeyse HAREKET ETME
        if distance_from_center <= dead_zone_radius:
            print(
                f"[CAMERA] Human in DEAD ZONE (distance: {distance_from_center:.1f}px, radius: {dead_zone_radius}px) - NO MOVEMENT")
            return

        # DEAD ZONE dışındaysa smooth hareket et
        print(f"[CAMERA] Human OUTSIDE dead zone (distance: {distance_from_center:.1f}px) - SMOOTH MOVING")

        # X ekseni hareketi (Pan) - SMOOTH
        x_error = center_x - frame_center_x
        if abs(x_error) > dead_zone_radius:
            if x_error > 0:  # İnsan sağda, kamera SOLA dönmeli
                new_pan = self.camera_pan_angle - camera_speed
                print(f"[CAMERA] Human RIGHT, Camera moving LEFT: {new_pan}")
            else:  # İnsan solda, kamera SAĞA dönmeli
                new_pan = self.camera_pan_angle + camera_speed
                print(f"[CAMERA] Human LEFT, Camera moving RIGHT: {new_pan}")

            self.set_camera_position(pan_angle=new_pan, smooth=True)  # SMOOTH hareket

        # Y ekseni hareketi (Tilt) - SMOOTH
        y_error = center_y - frame_center_y
        if abs(y_error) > dead_zone_radius:
            if y_error > 0:  # İnsan altta, kamera AŞAĞI bak
                new_tilt = self.camera_tilt_angle + camera_speed
                print(f"[CAMERA] Human BOTTOM, Camera moving DOWN: {new_tilt}")
            else:  # İnsan üstte, kamera YUKARI bak
                new_tilt = self.camera_tilt_angle - camera_speed
                print(f"[CAMERA] Human TOP, Camera moving UP: {new_tilt}")

            self.set_camera_position(tilt_angle=new_tilt, smooth=True)  # SMOOTH hareket

        print(f"[CAMERA] Human at ({center_x},{center_y}), Error: ({x_error},{y_error})")

    def camera_sweep_search(self):
        """Camera sweep movement - when human not found"""
        if not self.camera_tracking:
            return

        print("[CAMERA] Sweep movement starting...")
        settings = self.get_power_settings()

        # Horizontal sweep
        positions = [90, 60, 120, 90, 45, 135, 90]

        for pan_pos in positions:
            if self.human_detected:  # Human found, stop sweep
                break
            self.set_camera_position(pan_angle=pan_pos)
            time.sleep(0.3)

        # Return to center
        self.set_camera_position(pan_angle=90, tilt_angle=90)

    def detect_roi_zone(self, human_center):
        """ROI zone detection"""
        if not self.roi_enabled:
            return 'center'

        x, y = human_center

        for zone_name, zone in self.roi_zones.items():
            if (zone['x'] <= x <= zone['x'] + zone['w'] and
                    zone['y'] <= y <= zone['y'] + zone['h']):
                return zone_name

        return 'center'  # Default

    def check_roi_distance_rules(self, distance, roi_zone):
        """Check ROI distance rules"""
        # Minimum safe distance check
        if distance <= self.roi_min_distance:
            return "retreat"  # Back away

        # ROI stop distance check
        elif distance <= self.roi_stop_distance:
            if roi_zone == 'center':
                return "stop"  # Stop
            else:
                return "turn"  # Turn

        # Normal tracking distance
        else:
            return "approach"  # Approach

    def draw_roi_zones(self, frame):
        """Draw ROI zones on frame"""
        if not self.roi_enabled:
            return frame

        colors = {
            'center': (0, 255, 0),  # Green
            'left': (255, 0, 0),  # Blue
            'right': (255, 0, 0),  # Blue
            'top': (0, 255, 255),  # Yellow
            'bottom': (0, 255, 255)  # Yellow
        }

        for zone_name, zone in self.roi_zones.items():
            color = colors.get(zone_name, (255, 255, 255))
            cv2.rectangle(frame,
                          (zone['x'], zone['y']),
                          (zone['x'] + zone['w'], zone['y'] + zone['h']),
                          color, 2)
            cv2.putText(frame, zone_name.upper(),
                        (zone['x'], zone['y'] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        return frame

    def get_power_settings(self):
        """Get current power mode settings"""
        return self.power_settings[self.power_mode]

    def set_power_mode(self, mode):
        """Change power mode"""
        if mode in self.power_settings:
            self.power_mode = mode
            settings = self.get_power_settings()
            print(f"[POWER] Power mode changed: {settings['name']}")
            print(f"[POWER] {settings['description']}")
            return True
        return False

    def calculate_distance(self, bbox_height):
        """Calculate human distance"""
        if bbox_height <= 0:
            return 0

        cal = self.distance_calibration
        distance = (cal['person_height_cm'] * cal['camera_focal_length']) / bbox_height
        distance = max(cal['min_distance_cm'], min(cal['max_distance_cm'], distance))

        return int(distance)

    def init_ai_model(self):
        """Load AI model"""
        try:
            print("[INFO] Loading AI Model...")
            self.net = cv2.dnn.readNetFromCaffe(self.prototxt_path, self.model_path)
            print("[INFO] AI Model loaded successfully!")
        except Exception as e:
            print(f"[ERROR] AI Model could not be loaded: {e}")
            self.net = None

    def start_camera(self):
        """Start camera"""
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                return False

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.is_camera_running = True

            camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            camera_thread.start()

            print("[INFO] Camera started")
            return True
        except Exception as e:
            print(f"[ERROR] Camera could not be started: {e}")
            return False

    def stop_camera(self):
        """Stop camera"""
        self.is_camera_running = False
        self.tracking_enabled = False
        self.camera_tracking = False
        if self.cap:
            self.cap.release()
        print("[INFO] Camera stopped")

    def camera_loop(self):
        """Main camera loop"""
        while self.is_camera_running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                # AI detection
                if self.net is not None:
                    frame = self.detect_humans(frame)

                # Add ROI zones
                frame = self.draw_roi_zones(frame)

                # Add status overlay
                frame = self.add_status_overlay(frame)

                self.current_frame = frame

                time.sleep(0.033)  # ~30 FPS

            except Exception as e:
                print(f"[ERROR] Camera loop error: {e}")
                time.sleep(1)
# ...existing code...
    # ...existing code...

    def detect_humans(self, frame):
        """Human detection with AI"""
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

            # Person class = 15
            if class_id == 15 and confidence > 0.3:
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

        # Process best detection
        if best_person:
            self.human_detected = True
            bbox = best_person['bbox']
            center = best_person['center']

            # Calculate distance
            self.human_distance = self.calculate_distance(best_person['height'])
            self.human_position = center

            # ROI zone detection
            roi_zone = self.detect_roi_zone(center)
            self.last_roi_zone = roi_zone

            # *** YENİ: DEAD ZONE MESAFE HESAPLA ***
            frame_center_x, frame_center_y = 320, 240
            distance_from_center = ((center[0] - frame_center_x) ** 2 + (center[1] - frame_center_y) ** 2) ** 0.5

            # Draw bounding box
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.circle(frame, center, 5, (0, 255, 0), -1)

            # Distance and confidence text
            cv2.putText(frame, f"Distance: {self.human_distance}cm",
                        (bbox[0], bbox[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Conf: {best_person['confidence']:.2f}",
                        (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"ROI: {roi_zone.upper()}",
                        (bbox[0], bbox[3] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # *** YENİ: DEAD ZONE STATUS ***
            zone_status = "IN ZONE" if distance_from_center <= 60 else "TRACKING"
            zone_color = (0, 255, 255) if distance_from_center <= 60 else (255, 0, 255)
            cv2.putText(frame, f"Cam: {zone_status}",
                        (bbox[0], bbox[3] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, zone_color, 2)

            # Camera tracking
            if self.camera_tracking:
                self.camera_track_human(center)

            # Robot movement tracking
            if self.tracking_enabled:
                self.track_human(center, self.human_distance, roi_zone)

        else:
            self.human_detected = False
            if self.camera_tracking and not self.human_detected:
                # Start sweep search after 3 seconds
                current_time = time.time()
                if current_time - self.last_action_time > 3.0:
                    threading.Thread(target=self.camera_sweep_search, daemon=True).start()
                    self.last_action_time = current_time

        return frame

    def add_status_overlay(self, frame):
        """Add status information overlay"""
        # Status background
        cv2.rectangle(frame, (10, 10), (350, 150), (0, 0, 0), -1)

        # Status text
        status_lines = [
            f"Tracking: {'ON' if self.tracking_enabled else 'OFF'}",
            f"Camera Track: {'ON' if self.camera_tracking else 'OFF'}",
            f"Human: {'DETECTED' if self.human_detected else 'NOT FOUND'}",
            f"Distance: {self.human_distance}cm",
            f"ROI Zone: {self.last_roi_zone.upper()}",
            f"Power Mode: {self.power_mode.upper()}",
            f"Walking: {'YES' if self.walking else 'NO'}"
        ]

        for i, line in enumerate(status_lines):
            color = (0, 255, 0) if any(word in line for word in ['ON', 'DETECTED', 'YES']) else (255, 255, 255)
            cv2.putText(frame, line, (15, 30 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # *** YENİ: DEAD ZONE DAİRESİ ÇİZ - AYARLANABİLİR ***
        frame_center_x = 320  # 640/2
        frame_center_y = 240  # 480/2
        dead_zone_radius = self.dead_zone_radius  # Artık ayarlanabilir!

        # Dead zone dairesi (sarı renk)
        cv2.circle(frame, (frame_center_x, frame_center_y), int(dead_zone_radius), (0, 255, 255), 2)
        cv2.putText(frame, f"DEAD ZONE ({dead_zone_radius}px)", (frame_center_x - 60, frame_center_y - 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Merkez çarpı işareti
        cv2.line(frame, (frame_center_x - 10, frame_center_y), (frame_center_x + 10, frame_center_y), (0, 255, 255), 2)
        cv2.line(frame, (frame_center_x, frame_center_y - 10), (frame_center_x, frame_center_y + 10), (0, 255, 255), 2)

        return frame

    def track_human(self, human_center, distance, roi_zone):
        """Main human tracking logic"""
        if self.walking:
            return  # Don't start new movement if already walking

        current_time = time.time()

        # Cooldown check
        settings = self.get_power_settings()
        if current_time - self.last_action_time < settings['cooldown']:
            return

        # ROI distance rules
        action = self.check_roi_distance_rules(distance, roi_zone)

        print(f"[TRACKING] Action: {action}, Distance: {distance}cm, ROI: {roi_zone}")

        if action == "retreat":
            self.spider_back_away()
        elif action == "stop":
            print("[TRACKING] Stopping - target in center at good distance")
        elif action == "turn":
            # Turn towards center
            frame_center_x = self.frame_center_x
            if human_center[0] < frame_center_x - 50:
                self.spider_turn_left()
            elif human_center[0] > frame_center_x + 50:
                self.spider_turn_right()
        elif action == "approach":
            # Move forward or turn to center
            frame_center_x = self.frame_center_x
            if abs(human_center[0] - frame_center_x) < 100:  # Human in center
                self.spider_walk_forward()
            elif human_center[0] < frame_center_x:
                self.spider_turn_left()
            else:
                self.spider_turn_right()

        self.last_action_time = current_time

    def set_servo_angle(self, servo_name, angle):
        """Set servo angle safely"""
        try:
            if servo_name in self.servos:
                angle = max(0, min(180, angle))  # Limit angle
                self.servos[servo_name].angle = angle
                self.current_angles[servo_name] = angle
                return True
        except Exception as e:
            print(f"[ERROR] Servo {servo_name} error: {e}")
        return False

    def smooth_servo_move(self, servo_name, target_angle, steps=5):
        """Smooth servo movement"""
        if servo_name not in self.servos:
            return

        current_angle = self.current_angles.get(servo_name, 90)
        step_size = (target_angle - current_angle) / steps
        settings = self.get_power_settings()

        for i in range(steps):
            new_angle = current_angle + (step_size * (i + 1))
            self.set_servo_angle(servo_name, new_angle)
            time.sleep(settings['speed_delay'])

            controller = HumanTrackingServoController()