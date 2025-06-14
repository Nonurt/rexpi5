def spider_stance_position(self):
        """Basic stance position"""
        if self.walking:
            return
        
        with self.walking_lock:
            print("[SPIDER] Moving to stance position...")
            
            # All legs to center position
            for servo_name in self.servos:
                if 'camera' not in servo_name:  # Skip camera servos
                    self.set_servo_angle(servo_name, 90)
            
            time.sleep(0.5)
            print("[SPIDER] Stance position ready")

    
    def spider_walk_algorithm_v2(self, direction="forward", steps=1):
        """Gelişmiş 4-bacaklı spider yürüme algoritması"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print(f"[SPIDER] Walking {direction} - {steps} steps - {settings['name']} mode")
            
            try:
                for step in range(steps):
                    if direction == "forward":
                        self._execute_crawl_forward()
                    elif direction == "fast_forward":
                        self._execute_trot_forward()
                    elif direction == "backward":
                        self._execute_backward_gait()
                    elif direction == "left":
                        self._execute_left_turn_gait()
                    elif direction == "right":
                        self._execute_right_turn_gait()
                    
                    time.sleep(settings['step_delay'])
                
                self.spider_stance_position()
                
            except Exception as e:
                print(f"[ERROR] Walking algorithm error: {e}")
            finally:
                self.walking = False

    


def spider_adaptive_walk(self, direction="forward", distance_cm=0):
        """Adaptif yürüme - mesafeye göre adım sayısını hesaplar"""
        if distance_cm > 0:
            steps = max(1, distance_cm // 10)
        else:
            steps = 1
        
        print(f"[ADAPTIVE] Walking {steps} steps for {distance_cm}cm")
        self.spider_walk_algorithm_v2(direction, steps)

    def spider_wave_gait(self):
        """Wave gait - bacaklar sırayla hareket eder"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print("[SPIDER] Wave gait starting...")
            
            try:
                leg_sequence = ['front_left', 'rear_left', 'rear_right', 'front_right']
                
                for leg_name in leg_sequence:
                    lift_servo = f"{leg_name}_lift"
                    axis_servo = f"{leg_name}_axis"
                    
                    # Lift
                    self.smooth_servo_move(lift_servo, 90 - settings['lift_range'], steps=3)
                    time.sleep(settings['speed_delay'])
                    
                    # Move forward
                    if 'left' in leg_name:
                        self.smooth_servo_move(axis_servo, 90 + settings['axis_range'], steps=4)
                    else:
                        self.smooth_servo_move(axis_servo, 90 - settings['axis_range'], steps=4)
                    time.sleep(settings['speed_delay'] * 2)
                    
                    # Lower
                    self.smooth_servo_move(lift_servo, 90, steps=3)
                    time.sleep(settings['speed_delay'])
                
                self.spider_stance_position()
                
            except Exception as e:
                print(f"[ERROR] Wave gait error: {e}")
            finally:
                self.walking = False

    def spider_creep_gait(self):
        """Creep gait - en yavaş ama en stabil yürüme"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print("[SPIDER] Creep gait - maximum stability")
            
            try:
                moves = [
                    ('front_left_lift', 'front_left_axis', 90 + settings['axis_range']),
                    ('rear_right_lift', 'rear_right_axis', 90 - settings['axis_range']),
                    ('front_right_lift', 'front_right_axis', 90 - settings['axis_range']),
                    ('rear_left_lift', 'rear_left_axis', 90 + settings['axis_range'])
                ]
                
                for lift_servo, axis_servo, target_angle in moves:
                    # Lift slowly
                    self.smooth_servo_move(lift_servo, 90 - settings['lift_range'], steps=5)
                    time.sleep(settings['speed_delay'] * 3)
                    
                    # Move carefully
                    self.smooth_servo_move(axis_servo, target_angle, steps=6)
                    time.sleep(settings['speed_delay'] * 4)
                    
                    # Lower slowly
                    self.smooth_servo_move(lift_servo, 90, steps=5)
                    time.sleep(settings['speed_delay'] * 3)
                
                self.spider_stance_position()
                
            except Exception as e:
                print(f"[ERROR] Creep gait error: {e}")
            finally:
                self.walking = False

    


def spider_walk_forward(self):
        """İleri yürüme - REX algoritması kullan"""
        print("[MAIN] Using REX forward gait instead of spider")
        self.rex_forward_gait()

    def spider_turn_left(self):
        """Sola dönüş - REX algoritması kullan"""
        print("[MAIN] Using REX left turn instead of spider")
        self.rex_turn_left()

    def spider_turn_right(self):
        """Sağa dönüş - REX algoritması kullan"""
        print("[MAIN] Using REX right turn instead of spider")
        self.rex_turn_right()

    def spider_back_away(self):
        """Geri gitme - REX algoritması kullan"""
        print("[MAIN] Using REX backward gait instead of spider")
        self.rex_backward_gait()

    def spider_stance_position(self):
        """Stance pozisyonu - REX stabilize kullan"""
        print("[MAIN] Using REX stabilize instead of spider stance")
        self.rex_stabilize()

    def spider_defensive_posture(self):
        """Defensive posture - legs spread wide"""
        if self.walking:
            return
        
        with self.walking_lock:
            print("[SPIDER] Defensive posture...")
            
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
            print("[SPIDER] Defensive posture ready")

    def spider_attack_posture(self):
        """Attack posture - front legs raised"""
        if self.walking:
            return
        
        with self.walking_lock:
            print("[SPIDER] Attack posture...")
            
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
            print("[SPIDER] Attack posture ready")

    def spider_crouch_low(self):
        """Crouch low to ground"""
        if self.walking:
            return
        
        with self.walking_lock:
            print("[SPIDER] Crouching low...")
            
            # All legs low to ground
            for servo_name in ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']:
                self.set_servo_angle(servo_name, 120)
            
            # Keep axis centered
            for servo_name in ['front_left_axis', 'front_right_axis', 'rear_left_axis', 'rear_right_axis']:
                self.set_servo_angle(servo_name, 90)
            
            time.sleep(1.0)
            print("[SPIDER] Crouched low")

    # SATIR 930'dan SONRA (spider_crouch_low'dan sonra) şunu EKLE:

    # === ADVANCED LEG CONTROL FUNCTIONS ===
    
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

    def spread_legs(self, leg_group="all"):
        """Spread specific leg groups"""
        if self.walking:
            return
        
        with self.walking_lock:
            settings = self.get_power_settings()
            spread_angle = settings['axis_range']
            
            if leg_group == 'front':
                self.set_servo_angle('front_left_axis', 90 + spread_angle)
                self.set_servo_angle('front_right_axis', 90 - spread_angle)
                print("[LEG] Front legs spread wide")
            elif leg_group == 'rear':
                self.set_servo_angle('rear_left_axis', 90 + spread_angle)
                self.set_servo_angle('rear_right_axis', 90 - spread_angle)
                print("[LEG] Rear legs spread wide")
            elif leg_group == 'all':
                for leg in ['front_left_axis', 'rear_left_axis']:
                    self.set_servo_angle(leg, 90 + spread_angle)
                for leg in ['front_right_axis', 'rear_right_axis']:
                    self.set_servo_angle(leg, 90 - spread_angle)
                print("[LEG] All legs spread wide")
            
            time.sleep(0.5)

    def center_legs(self, leg_group="all"):
        """Center specific leg groups"""
        if self.walking:
            return
        
        with self.walking_lock:
            leg_groups = {
                'front': ['front_left_axis', 'front_right_axis'],
                'rear': ['rear_left_axis', 'rear_right_axis'],
                'left': ['front_left_axis', 'rear_left_axis'],
                'right': ['front_right_axis', 'rear_right_axis'],
                'all': ['front_left_axis', 'front_right_axis', 'rear_left_axis', 'rear_right_axis']
            }
            
            legs = leg_groups.get(leg_group, leg_groups['all'])
            print(f"[LEG] Centering {leg_group} legs to 90°")
            
            for leg in legs:
                self.set_servo_angle(leg, 90)
            
            time.sleep(0.5)

    # === ADVANCED GAIT PATTERNS ===
    
    def front_lift_gait(self, direction="forward"):
        """Front legs dominant walking gait"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print(f"[GAIT] Front lift gait - {direction}")
            
            try:
                # Phase 1: Lift front legs high
                self.lift_legs('front')
                time.sleep(settings['speed_delay'] * 2)
                
                # Phase 2: Move front legs forward/backward
                if direction == "forward":
                    self.set_servo_angle('front_left_axis', 90 + settings['axis_range'])
                    self.set_servo_angle('front_right_axis', 90 - settings['axis_range'])
                else:
                    self.set_servo_angle('front_left_axis', 90 - settings['axis_range'])
                    self.set_servo_angle('front_right_axis', 90 + settings['axis_range'])
                
                time.sleep(settings['speed_delay'] * 3)
                
                # Phase 3: Lower front legs
                self.lower_legs('front')
                time.sleep(settings['speed_delay'] * 2)
                
                # Phase 4: Push with rear legs
                if direction == "forward":
                    self.set_servo_angle('rear_left_axis', 90 - settings['axis_range']//2)
                    self.set_servo_angle('rear_right_axis', 90 + settings['axis_range']//2)
                else:
                    self.set_servo_angle('rear_left_axis', 90 + settings['axis_range']//2)
                    self.set_servo_angle('rear_right_axis', 90 - settings['axis_range']//2)
                
                time.sleep(settings['speed_delay'] * 2)
                
                # Return to stance
                self.spider_stance_position()
                
            except Exception as e:
                print(f"[ERROR] Front lift gait error: {e}")
            finally:
                self.walking = False

    def rear_drive_gait(self, direction="forward"):
        """Rear legs dominant walking gait"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print(f"[GAIT] Rear drive gait - {direction}")
            
            try:
                # Phase 1: Lift rear legs
                self.lift_legs('rear')
                time.sleep(settings['speed_delay'] * 2)
                
                # Phase 2: Position rear legs for power
                if direction == "forward":
                    self.set_servo_angle('rear_left_axis', 90 + settings['axis_range'])
                    self.set_servo_angle('rear_right_axis', 90 - settings['axis_range'])
                else:
                    self.set_servo_angle('rear_left_axis', 90 - settings['axis_range'])
                    self.set_servo_angle('rear_right_axis', 90 + settings['axis_range'])
                
                time.sleep(settings['speed_delay'] * 3)
                
                # Phase 3: Lower rear legs with force
                self.lower_legs('rear')
                time.sleep(settings['speed_delay'] * 1)
                
                # Phase 4: Drive forward with rear legs
                if direction == "forward":
                    self.set_servo_angle('rear_left_axis', 90 - settings['axis_range'])
                    self.set_servo_angle('rear_right_axis', 90 + settings['axis_range'])
                else:
                    self.set_servo_angle('rear_left_axis', 90 + settings['axis_range'])
                    self.set_servo_angle('rear_right_axis', 90 - settings['axis_range'])
                
                time.sleep(settings['speed_delay'] * 3)
                
                # Return to stance
                self.spider_stance_position()
                
            except Exception as e:
                print(f"[ERROR] Rear drive gait error: {e}")
            finally:
                self.walking = False

    def alternating_pairs_gait(self, direction="forward"):
        """Alternating front-rear pairs gait"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print(f"[GAIT] Alternating pairs gait - {direction}")
            
            try:
                # Phase 1: Front legs step
                self.lift_legs('front')
                time.sleep(settings['speed_delay'])
                
                if direction == "forward":
                    self.set_servo_angle('front_left_axis', 90 + settings['axis_range'])
                    self.set_servo_angle('front_right_axis', 90 - settings['axis_range'])
                
                time.sleep(settings['speed_delay'] * 2)
                self.lower_legs('front')
                time.sleep(settings['speed_delay'])
                
                # Phase 2: Rear legs step
                self.lift_legs('rear')
                time.sleep(settings['speed_delay'])
                
                if direction == "forward":
                    self.set_servo_angle('rear_left_axis', 90 + settings['axis_range'])
                    self.set_servo_angle('rear_right_axis', 90 - settings['axis_range'])
                
                time.sleep(settings['speed_delay'] * 2)
                self.lower_legs('rear')
                time.sleep(settings['speed_delay'])
                
                # Return to stance
                self.spider_stance_position()
                
            except Exception as e:
                print(f"[ERROR] Alternating pairs gait error: {e}")
            finally:
                self.walking = False

