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
        cv2.putText(frame, f"DEAD ZONE ({dead_zone_radius}px)", (frame_center_x-60, frame_center_y-70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Merkez çarpı işareti
        cv2.line(frame, (frame_center_x-10, frame_center_y), (frame_center_x+10, frame_center_y), (0, 255, 255), 2)
        cv2.line(frame, (frame_center_x, frame_center_y-10), (frame_center_x, frame_center_y+10), (0, 255, 255), 2)
        
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
