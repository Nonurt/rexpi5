import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import threading
import math


class ServoController:
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

        # Power mode settings
        self.power_mode = "medium"
        self.power_settings = {
            "low": {
                "name": "Low Power",
                "description": "Energy efficient, slow movement",
                "lift_range": 30,
                "axis_range": 30,
                "speed_delay": 0.08,
                "step_delay": 0.4,
                "cooldown": 2.0,
                "camera_speed": 3,
                "movement_speed": 0.05
            },
            "medium": {
                "name": "Medium Power",
                "description": "Balanced performance",
                "lift_range": 35,
                "axis_range": 40,
                "speed_delay": 0.05,
                "step_delay": 0.25,
                "cooldown": 1.5,
                "camera_speed": 4,
                "movement_speed": 0.03
            },
            "high": {
                "name": "High Power",
                "description": "Fast and powerful movement",
                "lift_range": 45,
                "axis_range": 50,
                "speed_delay": 0.03,
                "step_delay": 0.15,
                "cooldown": 1.0,
                "camera_speed": 6,
                "movement_speed": 0.02
            },
            "max": {
                "name": "Maximum Power",
                "description": "Most powerful movement - SUPER FAST",
                "lift_range": 55,
                "axis_range": 60,
                "speed_delay": 0.01,
                "step_delay": 0.08,
                "cooldown": 0.5,
                "camera_speed": 8,
                "movement_speed": 0.01
            }
        }

        # Initialize camera position
        self.init_camera_position()

    def init_camera_position(self):
        """Camera servo initial position"""
        print("[CAMERA] Camera servo initial position...")
        self.set_servo_angle('camera_pan', 90)
        self.set_servo_angle('camera_tilt', 90)
        time.sleep(1)
        print("[CAMERA] Camera in center position")

    def set_camera_position(self, pan_angle=None, tilt_angle=None, smooth=True):
        """Set camera position with smooth movement"""
        limits = self.camera_limits

        if pan_angle is not None:
            pan_angle = max(limits['pan_min'], min(limits['pan_max'], pan_angle))

            if smooth:
                current_pan = self.camera_pan_angle
                step_count = max(3, abs(pan_angle - current_pan) // 5)

                for i in range(int(step_count)):
                    intermediate_angle = current_pan + ((pan_angle - current_pan) * (i + 1) / step_count)
                    try:
                        self.servos['camera_pan'].angle = intermediate_angle
                        time.sleep(self.camera_smooth_delay)
                    except Exception as e:
                        print(f"[ERROR] Pan servo smooth movement error: {e}")
                        break
            else:
                try:
                    self.servos['camera_pan'].angle = pan_angle
                except Exception as e:
                    print(f"[ERROR] Pan servo error: {e}")

            self.camera_pan_angle = pan_angle
            print(f"[CAMERA] Pan servo moved to: {pan_angle}° ({'smooth' if smooth else 'fast'})")

        if tilt_angle is not None:
            tilt_angle = max(limits['tilt_min'], min(limits['tilt_max'], tilt_angle))

            if smooth:
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
                try:
                    self.servos['camera_tilt'].angle = tilt_angle
                except Exception as e:
                    print(f"[ERROR] Tilt servo error: {e}")

            self.camera_tilt_angle = tilt_angle
            print(f"[CAMERA] Tilt servo moved to: {tilt_angle}° ({'smooth' if smooth else 'fast'})")

        print(f"[CAMERA] Position - Pan: {self.camera_pan_angle}°, Tilt: {self.camera_tilt_angle}°")

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

    def servo_move(self, servo_positions, steps_per_servo, delay):
        """Micro-stepped servo movement"""
        current_positions = {}
        target_positions = {}
        step_sizes = {}

        # Get starting positions
        for servo_name, target in servo_positions.items():
            if servo_name in self.current_angles:
                current_positions[servo_name] = self.current_angles[servo_name]
                target_positions[servo_name] = target

                # Calculate step size
                speed = steps_per_servo.get(servo_name, 3)
                diff = target - current_positions[servo_name]
                step_sizes[servo_name] = diff / max(abs(diff), 1) * speed if diff != 0 else 0

        # Micro-stepped movement - all servos move together
        max_steps = max([abs(target_positions[s] - current_positions[s]) for s in servo_positions.keys()], default=1)

        for step in range(int(max_steps)):
            for servo_name in servo_positions.keys():
                if servo_name in current_positions:
                    current = current_positions[servo_name]
                    target = target_positions[servo_name]
                    step_size = step_sizes[servo_name]

                    # Calculate new position
                    if abs(target - current) > abs(step_size):
                        new_position = current + step_size
                    else:
                        new_position = target

                    # Move servo
                    self.set_servo_angle(servo_name, new_position)
                    current_positions[servo_name] = new_position

            time.sleep(delay)

            # Check if all servos reached target
            all_reached = all(abs(current_positions[s] - target_positions[s]) < 2 for s in servo_positions.keys())
            if all_reached:
                break

    # ===== LEG CONTROL FUNCTIONS =====

    def lift_legs(self, leg_group="all"):
        """Lift specific leg groups"""
        if self.walking:
            return

        with self.walking_lock:
            settings = self.get_power_settings()
            lift_angle = 90 - settings['lift_range']

            leg_groups = {
                'front': ['front_left_lift', 'front_right_lift'],
                'rear': ['rear_left_lift', 'rear_right_lift'],
                'left': ['front_left_lift', 'rear_left_lift'],
                'right': ['front_right_lift', 'rear_right_lift'],
                'all': ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']
            }

            legs = leg_groups.get(leg_group, leg_groups['all'])
            print(f"[LEG] Lifting {leg_group} legs to {lift_angle}°")

            for leg in legs:
                self.smooth_servo_move(leg, lift_angle, steps=3)

            time.sleep(0.5)

    def lower_legs(self, leg_group="all"):
        """Lower specific leg groups"""
        if self.walking:
            return

        with self.walking_lock:
            leg_groups = {
                'front': ['front_left_lift', 'front_right_lift'],
                'rear': ['rear_left_lift', 'rear_right_lift'],
                'left': ['front_left_lift', 'rear_left_lift'],
                'right': ['front_right_lift', 'rear_right_lift'],
                'all': ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']
            }

            legs = leg_groups.get(leg_group, leg_groups['all'])
            print(f"[LEG] Lowering {leg_group} legs to 90°")

            for leg in legs:
                self.smooth_servo_move(leg, 90, steps=3)

            time.sleep(0.5)

    # ===== POSTURE FUNCTIONS =====

    def defensive_posture(self):
        """Defensive posture - legs spread wide"""
        if self.walking:
            return

        with self.walking_lock:
            print("[POSTURE] Defensive posture...")

            # Front legs spread
            self.set_servo_angle('front_left_axis', 45)
            self.set_servo_angle('front_right_axis', 135)
            self.set_servo_angle('front_left_lift', 70)
            self.set_servo_angle('front_right_lift', 70)

            # Rear legs spread
            self.set_servo_angle('rear_left_axis', 45)
            self.set_servo_angle('rear_right_axis', 135)
            self.set_servo_angle('rear_left_lift', 70)
            self.set_servo_angle('rear_right_lift', 70)

            time.sleep(1.0)
            print("[POSTURE] Defensive posture ready")

    def attack_posture(self):
        """Attack posture - front legs raised"""
        if self.walking:
            return

        with self.walking_lock:
            print("[POSTURE] Attack posture...")

            # Front legs raised high
            self.set_servo_angle('front_left_lift', 45)
            self.set_servo_angle('front_right_lift', 45)
            self.set_servo_angle('front_left_axis', 70)
            self.set_servo_angle('front_right_axis', 110)

            # Rear legs normal but ready
            self.set_servo_angle('rear_left_axis', 90)
            self.set_servo_angle('rear_right_axis', 90)
            self.set_servo_angle('rear_left_lift', 90)
            self.set_servo_angle('rear_right_lift', 90)

            time.sleep(1.0)
            print("[POSTURE] Attack posture ready")

    def crouch_low(self):
        """Crouch low to ground"""
        if self.walking:
            return

        with self.walking_lock:
            print("[POSTURE] Crouching low...")

            # All legs low to ground
            for servo_name in ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']:
                self.set_servo_angle(servo_name, 120)

            # Keep axis centered
            for servo_name in ['front_left_axis', 'front_right_axis', 'rear_left_axis', 'rear_right_axis']:
                self.set_servo_angle(servo_name, 90)

            time.sleep(1.0)
            print("[POSTURE] Crouched low")