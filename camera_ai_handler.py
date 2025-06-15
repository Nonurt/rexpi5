# camera_ai_handler.py
import cv2
import time
import numpy as np
import threading
from config import VIDEO, AI_MODEL, CAMERA_SETTINGS, ROI_SETTINGS


class CameraAIHandler:
    def init_ai_model(self):
        try:
            print("[INFO] Loading AI Model...")
            self.net = cv2.dnn.readNetFromCaffe(AI_MODEL['prototxt_path'], AI_MODEL['model_path'])
            print("[INFO] AI Model loaded successfully!")
        except Exception as e:
            print(f"[ERROR] AI Model could not be loaded: {e}")
            self.net = None

    def start_camera(self):
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened(): return False
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
        self.is_camera_running = False
        self.tracking_enabled = False
        self.camera_tracking = False
        if self.cap: self.cap.release()
        print("[INFO] Camera stopped")

    def init_camera_position(self):
        print("[CAMERA] Setting initial camera position...")
        self.set_camera_position(pan_angle=90, tilt_angle=90, smooth=False)
        time.sleep(1)
        print("[CAMERA] Camera is in center position.")

    def camera_loop(self):
        while self.is_camera_running:
            try:
                ret, frame = self.cap.read()
                if not ret: continue
                if self.net is not None: frame = self.detect_humans(frame)
                if self.roi_enabled: frame = self.draw_roi_zones(frame)
                frame = self.add_status_overlay(frame)
                self.current_frame = frame
                time.sleep(VIDEO['fps_delay'])
            except Exception as e:
                print(f"[ERROR] Camera loop error: {e}")
                time.sleep(1)

    def detect_humans(self, frame):
        if self.net is None: return frame
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        best_person = None
        best_confidence = 0.0
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if int(detections[0, 0, i, 1]) == AI_MODEL['person_class_id'] and confidence > AI_MODEL[
                'confidence_threshold']:
                if confidence > best_confidence:
                    best_confidence = confidence
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")
                    best_person = {
                        'bbox': (startX, startY, endX, endY),
                        'center': ((startX + endX) // 2, (startY + endY) // 2),
                        'height': endY - startY
                    }

        if best_person:
            self.human_detected = True
            bbox = best_person['bbox']
            center = best_person['center']
            self.human_distance = self.calculate_distance(best_person['height'])
            self.human_position = center
            self.last_roi_zone = self.detect_roi_zone(center)

            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.putText(frame, f"Insan ({self.human_distance}cm)", (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

            if self.camera_tracking: self.camera_track_human(center)
            if self.tracking_enabled: self.track_human(center, self.human_distance, self.last_roi_zone)
        else:
            self.human_detected = False
            if self.camera_tracking: self.camera_sweep_search()
        return frame

    def add_status_overlay(self, frame):
        status_lines = [
            f"Robot Takip: {'AKTIF' if self.tracking_enabled else 'KAPALI'}",
            f"Kamera Takip: {'AKTIF' if self.camera_tracking else 'KAPALI'}",
            f"Insan: {'TESPIT EDILDI' if self.human_detected else 'YOK'}",
            f"Mesafe: {self.human_distance} cm",
            f"ROI Bolge: {self.last_roi_zone.upper()}",
            f"Guc Modu: {self.power_mode.upper()}",
            f"Yuruyor: {'EVET' if self.walking else 'HAYIR'}"
        ]
        y0, dy = 20, 20
        for i, line in enumerate(status_lines):
            y = y0 + i * dy
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.circle(frame, (self.frame_center_x, self.frame_center_y), int(self.dead_zone_radius), (0, 255, 255), 1,
                   cv2.LINE_AA)
        return frame

    def draw_roi_zones(self, frame):
        for zone_name, zone in ROI_SETTINGS['zones'].items():
            color = (0, 255, 0) if zone_name == 'center' else (255, 0, 0)
            cv2.rectangle(frame, (zone['x'], zone['y']), (zone['x'] + zone['w'], zone['y'] + zone['h']), color, 1)
        return frame

    def camera_track_human(self, human_center):
        if self.walking: return
        if not self.camera_tracking or self.camera_lock.locked(): return

        with self.camera_lock:
            error_x = human_center[0] - self.frame_center_x
            error_y = human_center[1] - self.frame_center_y

            if abs(error_x) < self.dead_zone_radius and abs(error_y) < self.dead_zone_radius: return

            pan_adjustment = -1 * error_x * self.p_gain_pan
            tilt_adjustment = error_y * self.p_gain_tilt

            pan_adjustment = max(-self.max_adjustment, min(self.max_adjustment, pan_adjustment))
            tilt_adjustment = max(-self.max_adjustment, min(self.max_adjustment, tilt_adjustment))

            new_pan_angle = self.camera_pan_angle + pan_adjustment
            new_tilt_angle = self.camera_tilt_angle + tilt_adjustment

            self.set_camera_position(new_pan_angle, new_tilt_angle, smooth=False)

    def set_camera_position(self, pan_angle=None, tilt_angle=None, smooth=False):
        if pan_angle is not None:
            self.camera_pan_angle = max(self.camera_limits['pan_min'], min(self.camera_limits['pan_max'], pan_angle))
            if smooth:
                self.smooth_servo_move('camera_pan', self.camera_pan_angle)
            else:
                self.set_servo_angle('camera_pan', self.camera_pan_angle)

        if tilt_angle is not None:
            self.camera_tilt_angle = max(self.camera_limits['tilt_min'],
                                         min(self.camera_limits['tilt_max'], tilt_angle))
            if smooth:
                self.smooth_servo_move('camera_tilt', self.camera_tilt_angle)
            else:
                self.set_servo_angle('camera_tilt', self.camera_tilt_angle)

    def camera_sweep_search(self):
        if self.walking or self.human_detected: return
        current_time = time.time()
        if current_time - getattr(self, 'last_sweep_time', 0) < 5.0: return
        print("[CAMERA] Human lost. Starting sweep search...")

        threading.Thread(target=self._execute_sweep, daemon=True).start()
        self.last_sweep_time = current_time

    def _execute_sweep(self):
        positions = [120, 60, 90]
        for pos in positions:
            if self.human_detected or self.walking: break
            self.set_camera_position(pan_angle=pos, smooth=True)
            time.sleep(1)