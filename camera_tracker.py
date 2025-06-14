import cv2
import numpy as np
import time
import threading
import math


class CameraTracker:
    def __init__(self, controller):
        self.controller = controller
        self.cap = None
        self.is_camera_running = False
        self.current_frame = None

        # Camera tracking settings
        self.camera_tracking = False
        self.camera_lock = threading.Lock()
        self.camera_pan_angle = 90  # Center position
        self.camera_tilt_angle = 90  # Center position

        # Camera control parameters
        self.camera_smooth_delay = 0.05  # Smooth movement delay
        self.dead_zone_radius = 60  # Dead zone radius
        self.camera_step_size = 3  # Camera movement step size

        # Camera limits
        self.camera_limits = {
            'pan_min': 20,  # Left limit
            'pan_max': 160,  # Right limit
            'tilt_min': 45,  # Down limit
            'tilt_max': 135  # Up limit
        }

        # ROI (Region of Interest) settings
        self.roi_enabled = True
        self.roi_stop_distance = 80  # cm - Stop at this distance
        self.roi_min_distance = 50  # cm - Minimum safe distance
        self.roi_zones = {
            'center': {'x': 220, 'y': 160, 'w': 200, 'h': 160, 'priority': 1},
            'left': {'x': 0, 'y': 120, 'w': 220, 'h': 240, 'priority': 2},
            'right': {'x': 420, 'y': 120, 'w': 220, 'h': 240, 'priority': 2},
            'top': {'x': 160, 'y': 0, 'w': 320, 'h': 160, 'priority': 3},
            'bottom': {'x': 160, 'y': 320, 'w': 320, 'h': 160, 'priority': 3}
        }
        self.last_roi_zone = 'center'

        # Tracking system
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

    def start_camera(self):
        """Start camera capture"""
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
        """Stop camera capture"""
        self.is_camera_running = False
        self.tracking_enabled = False
        self.camera_tracking = False
        if self.cap:
            self.cap.release()
        print("[INFO] Camera stopped")

    def camera_loop(self):
        """Main camera processing loop"""
        while self.is_camera_running:
            try:
                ret, frame = self.cap.read()
                if not ret:
                    continue

                # AI detection if model is loaded
                if hasattr(self.controller, 'ai_model') and self.controller.ai_model.net is not None:
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

    def detect_humans(self, frame):
        """Human detection with AI"""
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
        self.controller.ai_model.net.setInput(blob)
        detections = self.controller.ai_model.net.forward()

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
            self.human_distance = self.controller.ai_model.calculate_distance(best_person['height'])
            self.human_position = center

            # ROI zone detection
            roi_zone = self.detect_roi_zone(center)
            self.last_roi_zone = roi_zone

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

            # Dead zone status
            distance_from_center = ((center[0] - self.frame_center_x) ** 2 +
                                    (center[1] - self.frame_center_y) ** 2) ** 0.5
            zone_status = "IN ZONE" if distance_from_center <= self.dead_zone_radius else "TRACKING"
            zone_color = (0, 255, 255) if distance_from_center <= self.dead_zone_radius else (255, 0, 255)
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

    def camera_track_human(self, human_center):
        """Camera human tracking with dead zone"""
        if not self.camera_tracking:
            return

        center_x, center_y = human_center
        frame_w, frame_h = 640, 480

        # DEAD ZONE check
        distance_from_center = ((center_x - self.frame_center_x) ** 2 +
                                (center_y - self.frame_center_y) ** 2) ** 0.5

        # If human in dead zone, don't move
        if distance_from_center <= self.dead_zone_radius:
            print(
                f"[CAMERA] Human in DEAD ZONE (distance: {distance_from_center:.1f}px, radius: {self.dead_zone_radius}px) - NO MOVEMENT")
            return

        print(f"[CAMERA] Human OUTSIDE dead zone (distance: {distance_from_center:.1f}px) - SMOOTH MOVING")

        # X axis movement (Pan)
        x_error = center_x - self.frame_center_x
        if abs(x_error) > self.dead_zone_radius:
            if x_error > 0:  # Human right, camera move left
                new_pan = self.camera_pan_angle - self.camera_step_size
                print(f"[CAMERA] Human RIGHT, Camera moving LEFT: {new_pan}")
            else:  # Human left, camera move right
                new_pan = self.camera_pan_angle + self.camera_step_size
                print(f"[CAMERA] Human LEFT, Camera moving RIGHT: {new_pan}")

            self.set_camera_position(pan_angle=new_pan, smooth=True)

        # Y axis movement (Tilt)
        y_error = center_y - self.frame_center_y
        if abs(y_error) > self.dead_zone_radius:
            if y_error > 0:  # Human bottom, camera move down
                new_tilt = self.camera_tilt_angle + self.camera_step_size
                print(f"[CAMERA] Human BOTTOM, Camera moving DOWN: {new_tilt}")
            else:  # Human top, camera move up
                new_tilt = self.camera_tilt_angle - self.camera_step_size
                print(f"[CAMERA] Human TOP, Camera moving UP: {new_tilt}")

            self.set_camera_position(tilt_angle=new_tilt, smooth=True)

        print(f"[CAMERA] Human at ({center_x},{center_y}), Error: ({x_error},{y_error})")

    def camera_sweep_search(self):
        """Camera sweep movement when human not found"""
        if not self.camera_tracking:
            return

        print("[CAMERA] Sweep movement starting...")
        settings = self.controller.get_power_settings()

        # Horizontal sweep pattern
        positions = [90, 60, 120, 90, 45, 135, 90]

        for pan_pos in positions:
            if self.human_detected:  # Human found, stop sweep
                break
            self.set_camera_position(pan_angle=pan_pos)
            time.sleep(0.3)

        # Return to center
        self.set_camera_position(pan_angle=90, tilt_angle=90)

    def set_camera_position(self, pan_angle=None, tilt_angle=None, smooth=True):
        """Set camera position with smooth movement"""
        if pan_angle is not None:
            pan_angle = max(self.camera_limits['pan_min'], min(self.camera_limits['pan_max'], pan_angle))

            if smooth:
                current_pan = self.camera_pan_angle
                step_count = max(3, abs(pan_angle - current_pan) // 5)

                for i in range(int(step_count)):
                    intermediate_angle = current_pan + ((pan_angle - current_pan) * (i + 1) / step_count)
                    self.controller.set_servo_angle('camera_pan', intermediate_angle)
                    time.sleep(self.camera_smooth_delay)
            else:
                self.controller.set_servo_angle('camera_pan', pan_angle)

            self.camera_pan_angle = pan_angle
            print(f"[CAMERA] Pan servo moved to: {pan_angle}° ({'smooth' if smooth else 'fast'})")

        if tilt_angle is not None:
            tilt_angle = max(self.camera_limits['tilt_min'], min(self.camera_limits['tilt_max'], tilt_angle))

            if smooth:
                current_tilt = self.camera_tilt_angle
                step_count = max(3, abs(tilt_angle - current_tilt) // 5)

                for i in range(int(step_count)):
                    intermediate_angle = current_tilt + ((tilt_angle - current_tilt) * (i + 1) / step_count)
                    self.controller.set_servo_angle('camera_tilt', intermediate_angle)
                    time.sleep(self.camera_smooth_delay)
            else:
                self.controller.set_servo_angle('camera_tilt', tilt_angle)

            self.camera_tilt_angle = tilt_angle
            print(f"[CAMERA] Tilt servo moved to: {tilt_angle}° ({'smooth' if smooth else 'fast'})")

        print(f"[CAMERA] Position - Pan: {self.camera_pan_angle}°, Tilt: {self.camera_tilt_angle}°")

    def detect_roi_zone(self, human_center):
        """Detect which ROI zone contains the human"""
        if not self.roi_enabled:
            return 'center'

        x, y = human_center

        for zone_name, zone in self.roi_zones.items():
            if (zone['x'] <= x <= zone['x'] + zone['w'] and
                    zone['y'] <= y <= zone['y'] + zone['h']):
                return zone_name

        return 'center'  # Default

    def check_roi_distance_rules(self, distance, roi_zone):
        """Check ROI distance rules for movement decisions"""
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

    def track_human(self, human_center, distance, roi_zone):
        """Main human tracking logic"""
        if self.controller.walking:
            return  # Don't start new movement if already walking

        current_time = time.time()

        # Cooldown check
        settings = self.controller.get_power_settings()
        if current_time - self.last_action_time < settings['cooldown']:
            return

        # ROI distance rules
        action = self.check_roi_distance_rules(distance, roi_zone)

        print(f"[TRACKING] Action: {action}, Distance: {distance}cm, ROI: {roi_zone}")

        if action == "retreat":
            self.controller.spider_back_away()
        elif action == "stop":
            print("[TRACKING] Stopping - target in center at good distance")
        elif action == "turn":
            # Turn towards center
            if human_center[0] < self.frame_center_x - 50:
                self.controller.spider_turn_left()
            elif human_center[0] > self.frame_center_x + 50:
                self.controller.spider_turn_right()
        elif action == "approach":
            # Move forward or turn to center
            if abs(human_center[0] - self.frame_center_x) < 100:  # Human in center
                self.controller.spider_walk_forward()
            elif human_center[0] < self.frame_center_x:
                self.controller.spider_turn_left()
            else:
                self.controller.spider_turn_right()

        self.last_action_time = current_time

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

    def add_status_overlay(self, frame):
        """Add status information overlay to frame"""
        # Status background
        cv2.rectangle(frame, (10, 10), (350, 150), (0, 0, 0), -1)

        # Status text
        status_lines = [
            f"Tracking: {'ON' if self.tracking_enabled else 'OFF'}",
            f"Camera Track: {'ON' if self.camera_tracking else 'OFF'}",
            f"Human: {'DETECTED' if self.human_detected else 'NOT FOUND'}",
            f"Distance: {self.human_distance}cm",
            f"ROI Zone: {self.last_roi_zone.upper()}",
            f"Power Mode: {self.controller.power_mode.upper()}",
            f"Walking: {'YES' if self.controller.walking else 'NO'}"
        ]

        for i, line in enumerate(status_lines):
            color = (0, 255, 0) if any(word in line for word in ['ON', 'DETECTED', 'YES']) else (255, 255, 255)
            cv2.putText(frame, line, (15, 30 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Dead zone circle
        cv2.circle(frame, (self.frame_center_x, self.frame_center_y),
                   int(self.dead_zone_radius), (0, 255, 255), 2)
        cv2.putText(frame, f"DEAD ZONE ({self.dead_zone_radius}px)",
                    (self.frame_center_x - 60, self.frame_center_y - 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Center crosshair
        cv2.line(frame, (self.frame_center_x - 10, self.frame_center_y),
                 (self.frame_center_x + 10, self.frame_center_y), (0, 255, 255), 2)
        cv2.line(frame, (self.frame_center_x, self.frame_center_y - 10),
                 (self.frame_center_x, self.frame_center_y + 10), (0, 255, 255), 2)

        return frame

    def generate_frames(self):
        """Generate video frames for streaming"""
        while True:
            if self.current_frame is not None:
                try:
                    ret, buffer = cv2.imencode('.jpg', self.current_frame)
                    if ret:
                        frame = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                except Exception as e:
                    print(f"[ERROR] Frame encoding error: {e}")

            time.sleep(0.033)  # ~30 FPS