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
            print(f"[CAMERA] Human in DEAD ZONE (distance: {distance_from_center:.1f}px, radius: {dead_zone_radius}px) - NO MOVEMENT")
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
            'center': (0, 255, 0),   # Green
            'left': (255, 0, 0),     # Blue
            'right': (255, 0, 0),    # Blue
            'top': (0, 255, 255),    # Yellow
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