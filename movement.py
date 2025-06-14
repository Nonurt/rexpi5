def rex_servo_move(self, servo_positions, steps_per_servo, delay):
        """REX tarzı mikro-adımlı servo hareketi - ESP32 srv() fonksiyonunun Python versiyonu"""
        # servo_positions: {servo_name: target_angle}
        # steps_per_servo: {servo_name: step_speed}
        
        current_positions = {}
        target_positions = {}
        step_sizes = {}
        
        # Başlangıç pozisyonlarını al
        for servo_name, target in servo_positions.items():
            if servo_name in self.current_angles:
                current_positions[servo_name] = self.current_angles[servo_name]
                target_positions[servo_name] = target
                
                # Adım büyüklüğünü hesapla
                speed = steps_per_servo.get(servo_name, 3)
                diff = target - current_positions[servo_name]
                step_sizes[servo_name] = diff / max(abs(diff), 1) * speed if diff != 0 else 0
        
        # Mikro-adımlı hareket - tüm servolar birlikte hareket eder
        max_steps = max([abs(target_positions[s] - current_positions[s]) for s in servo_positions.keys()], default=1)
        
        for step in range(int(max_steps)):
            for servo_name in servo_positions.keys():
                if servo_name in current_positions:
                    current = current_positions[servo_name]
                    target = target_positions[servo_name]
                    step_size = step_sizes[servo_name]
                    
                    # Yeni pozisyon hesapla
                    if abs(target - current) > abs(step_size):
                        new_position = current + step_size
                    else:
                        new_position = target
                    
                    # Servo'yu hareket ettir
                    self.set_servo_angle(servo_name, new_position)
                    current_positions[servo_name] = new_position
            
            time.sleep(delay)
            
            # Tüm servolar hedefe ulaştı mı kontrol et
            all_reached = all(abs(current_positions[s] - target_positions[s]) < 2 for s in servo_positions.keys())
            if all_reached:
                break

    def rex_stabilize(self):
        """REX stance position - REX kodundaki stabilize() fonksiyonu"""
        stance_height = 60  # REX kodundaki stance_height değişkeni
        
        servo_positions = {
            'front_left_axis': 90,
            'front_right_axis': 90,
            'rear_left_axis': 90,
            'rear_right_axis': 90,
            'front_left_lift': stance_height + 30,  # ESP32'de lift = stance_height + high*3
            'front_right_lift': stance_height + 30,
            'rear_left_lift': stance_height + 30,
            'rear_right_lift': stance_height + 30
        }
        
        steps_per_servo = {servo: 4 for servo in servo_positions.keys()}
        
        print("[REX] Stabilizing to stance position...")
        self.rex_servo_move(servo_positions, steps_per_servo, 0.005)

    def rex_forward_gait(self):
        """REX kodundaki forward() fonksiyonunun Python versiyonu"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print("[REX] REX Forward gait - 8 phase ESP32 style")
            
            try:
                # REX değişkenleri
                spd = int(settings['speed_delay'] * 1000)  # ms cinsinden
                high = settings['lift_range'] // 3  # lift çarpanı
                bigStep = settings['axis_range'] > 40  # büyük adım modu
                
                # REX kalça makroları
                def HIP_FWD(axis_servo):
                    if 'left' in axis_servo:
                        return 150 if bigStep else 120  # FL/RL için ters yön
                    else:
                        return 30 if bigStep else 60   # FR/RR için normal
                
                def HIP_BCK(axis_servo):
                    if 'left' in axis_servo:
                        return 30 if bigStep else 60   # FL/RL için ters yön
                    else:
                        return 150 if bigStep else 120 # FR/RR için normal
                
                # PHASE 1: FR ileri kayar, FL geriye iter
                print("[REX] Phase 1: FR forward, FL backward push")
                servo_positions = {
                    'front_left_axis': HIP_BCK('front_left_axis'),
                    'front_right_axis': HIP_FWD('front_right_axis'),
                    'rear_left_axis': 90,
                    'rear_right_axis': 90,
                    'front_left_lift': 42 + high*3,
                    'front_right_lift': 6 + high*3,  # FR kaldırılır
                    'rear_left_lift': 42 + high*3,
                    'rear_right_lift': 42 + high*3
                }
                steps_per_servo = {'front_left_axis': 2, 'front_right_axis': 3, 'rear_left_axis': 2, 'rear_right_axis': 2,
                                  'front_left_lift': 2, 'front_right_lift': 3, 'rear_left_lift': 2, 'rear_right_lift': 2}
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 2: Kilit
                print("[REX] Phase 2: Lock position")
                servo_positions.update({
                    'front_left_lift': 33 + high*3,
                    'front_right_lift': 42 + high*3,
                    'rear_left_lift': 33 + high*3,
                    'rear_right_lift': 42 + high*3
                })
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 3: RR ileri kayar, RL geriye iter
                print("[REX] Phase 3: RR forward, RL backward push")
                servo_positions.update({
                    'rear_left_axis': HIP_BCK('rear_left_axis'),
                    'rear_right_axis': HIP_FWD('rear_right_axis'),
                    'rear_left_lift': 42 + high*3,
                    'rear_right_lift': 6 + high*3  # RR kaldırılır
                })
                steps_per_servo.update({'rear_left_axis': 2, 'rear_right_axis': 3})
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 4: Kilit
                print("[REX] Phase 4: Lock position")
                servo_positions.update({
                    'rear_left_lift': 24 + high*3,
                    'rear_right_lift': 33 + high*3
                })
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 5: FL ileri kayar, FR geriye iter
                print("[REX] Phase 5: FL forward, FR backward push")
                servo_positions.update({
                    'front_left_axis': HIP_FWD('front_left_axis'),
                    'front_right_axis': HIP_BCK('front_right_axis'),
                    'front_left_lift': 6 + high*3,  # FL kaldırılır
                    'front_right_lift': 42 + high*3
                })
                steps_per_servo.update({'front_left_axis': 2, 'front_right_axis': 2})
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 6: Kilit
                print("[REX] Phase 6: Lock position")
                servo_positions.update({
                    'front_left_lift': 42 + high*3,
                    'front_right_lift': 6 + high*3
                })
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 7: RL ileri kayar, RR geriye iter
                print("[REX] Phase 7: RL forward, RR backward push")
                servo_positions.update({
                    'rear_left_axis': HIP_FWD('rear_left_axis'),
                    'rear_right_axis': HIP_BCK('rear_right_axis'),
                    'rear_left_lift': 6 + high*3,  # RL kaldırılır
                    'rear_right_lift': 33 + high*3
                })
                steps_per_servo.update({'rear_left_axis': 3, 'rear_right_axis': 2})
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 8: Nötr pozisyon
                print("[REX] Phase 8: Return to neutral")
                servo_positions = {
                    'front_left_axis': 90,
                    'front_right_axis': 90,
                    'rear_left_axis': 90,
                    'rear_right_axis': 90,
                    'front_left_lift': 33 + high*3,
                    'front_right_lift': 33 + high*3,
                    'rear_left_lift': 33 + high*3,
                    'rear_right_lift': 33 + high*3
                }
                steps_per_servo = {servo: 3 for servo in servo_positions.keys()}
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                print("[REX] Forward gait completed")
                
            except Exception as e:
                print(f"[ERROR] REX forward gait error: {e}")
            finally:
                self.walking = False

    def rex_backward_gait(self):
        """REX kodundaki back() fonksiyonunun Python versiyonu"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print("[REX] REX Backward gait - ESP32 style")
            
            try:
                spd = int(settings['speed_delay'] * 1000)
                high = settings['lift_range'] // 3
                bigStep = settings['axis_range'] > 40
                
                def HIP_FWD(axis_servo):
                    if 'left' in axis_servo:
                        return 150 if bigStep else 120
                    else:
                        return 30 if bigStep else 60
                
                def HIP_BCK(axis_servo):
                    if 'left' in axis_servo:
                        return 30 if bigStep else 60
                    else:
                        return 150 if bigStep else 120
                
                # PHASE 1: FR ileri kayar, FL geriye iter
                print("[REX] Phase 1: FR forward, FL backward push")
                servo_positions = {
                    'front_left_axis': HIP_BCK('front_left_axis'),
                    'front_right_axis': HIP_FWD('front_right_axis'),
                    'rear_left_axis': 90,
                    'rear_right_axis': 90,
                    'front_left_lift': 42 + high*3,
                    'front_right_lift': 6 + high*3,  # FR kaldırılır
                    'rear_left_lift': 42 + high*3,
                    'rear_right_lift': 42 + high*3
                }
                steps_per_servo = {'front_left_axis': 2, 'front_right_axis': 3, 'rear_left_axis': 2, 'rear_right_axis': 2,
                                  'front_left_lift': 2, 'front_right_lift': 3, 'rear_left_lift': 2, 'rear_right_lift': 2}
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 2: Kilit
                print("[REX] Phase 2: Lock position")
                servo_positions.update({
                    'front_left_lift': 33 + high*3,
                    'front_right_lift': 42 + high*3,
                    'rear_left_lift': 33 + high*3,
                    'rear_right_lift': 42 + high*3
                })
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 3: RR ileri kayar, RL geriye iter
                print("[REX] Phase 3: RR forward, RL backward push")
                servo_positions.update({
                    'rear_left_axis': HIP_BCK('rear_left_axis'),
                    'rear_right_axis': HIP_FWD('rear_right_axis'),
                    'rear_left_lift': 42 + high*3,
                    'rear_right_lift': 6 + high*3  # RR kaldırılır
                })
                steps_per_servo.update({'rear_left_axis': 2, 'rear_right_axis': 3})
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 4: Kilit
                print("[REX] Phase 4: Lock position")
                servo_positions.update({
                    'rear_left_lift': 24 + high*3,
                    'rear_right_lift': 33 + high*3
                })
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 5: FL ileri kayar, FR geriye iter
                print("[REX] Phase 5: FL forward, FR backward push")
                servo_positions.update({
                    'front_left_axis': HIP_FWD('front_left_axis'),
                    'front_right_axis': HIP_BCK('front_right_axis'),
                    'front_left_lift': 6 + high*3,  # FL kaldırılır
                    'front_right_lift': 42 + high*3
                })
                steps_per_servo.update({'front_left_axis': 2, 'front_right_axis': 2})
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 6: Kilit
                print("[REX] Phase 6: Lock position")
                servo_positions.update({
                    'front_left_lift': 42 + high*3,
                    'front_right_lift': 6 + high*3
                })
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 7: RL ileri kayar, RR geriye iter
                print("[REX] Phase 7: RL forward, RR backward push")
                servo_positions.update({
                    'rear_left_axis': HIP_FWD('rear_left_axis'),
                    'rear_right_axis': HIP_BCK('rear_right_axis'),
                    'rear_left_lift': 6 + high*3,  # RL kaldırılır
                    'rear_right_lift': 33 + high*3
                })
                steps_per_servo.update({'rear_left_axis': 3, 'rear_right_axis': 2})
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                # PHASE 8: Nötr pozisyon
                print("[REX] Phase 8: Return to neutral")
                servo_positions = {
                    'front_left_axis': 90,
                    'front_right_axis': 90,
                    'rear_left_axis': 90,
                    'rear_right_axis': 90,
                    'front_left_lift': 33 + high*3,
                    'front_right_lift': 33 + high*3,
                    'rear_left_lift': 33 + high*3,
                    'rear_right_lift': 33 + high*3
                }
                steps_per_servo = {servo: 3 for servo in servo_positions.keys()}
                self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                print("[REX] Backward gait completed")
                
            except Exception as e:
                print(f"[ERROR] REX backward gait error: {e}")
            finally:
                self.walking = False

    def rex_turn_left(self):
        """REX kodundaki turn_left() fonksiyonunun Python versiyonu"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print("[REX] REX Left turn - ESP32 style")
            
            try:
                spd = int(settings['speed_delay'] * 1000)
                high = settings['lift_range'] // 3
                
                # REX turn_left sequence - 8 fazlı dönüş
                phases = [
                    # (FL, RL, RR, FR, FL_lift, RL_lift, RR_lift, FR_lift, steps)
                    (120, 90, 90, 60, 42+high*3, 6+high*3, 33+high*3, 42+high*3, [2,3,2,2]),
                    (120, 90, 90, 60, 42+high*3, 33+high*3, 33+high*3, 42+high*3, [2,3,2,2]),
                    (100, 70, 110, 80, 42+high*3, 33+high*3, 6+high*3, 42+high*3, [2,2,3,2]),
                    (100, 70, 110, 80, 42+high*3, 33+high*3, 33+high*3, 24+high*3, [2,2,3,2]),
                    (80, 50, 130, 100, 42+high*3, 33+high*3, 33+high*3, 6+high*3, [2,2,2,3]),
                    (80, 50, 130, 100, 42+high*3, 33+high*3, 33+high*3, 42+high*3, [2,2,2,3]),
                    (120, 90, 90, 60, 6+high*3, 33+high*3, 33+high*3, 42+high*3, [3,2,2,2]),
                    (120, 90, 90, 60, 42+high*3, 33+high*3, 33+high*3, 33+high*3, [3,2,2,2])
                ]
                
                for i, (fl_axis, rl_axis, rr_axis, fr_axis, fl_lift, rl_lift, rr_lift, fr_lift, steps) in enumerate(phases):
                    print(f"[REX] Turn left phase {i+1}/8")
                    
                    servo_positions = {
                        'front_left_axis': fl_axis,
                        'rear_left_axis': rl_axis,
                        'rear_right_axis': rr_axis,
                        'front_right_axis': fr_axis,
                        'front_left_lift': fl_lift,
                        'rear_left_lift': rl_lift,
                        'rear_right_lift': rr_lift,
                        'front_right_lift': fr_lift
                    }
                    
                    steps_per_servo = {
                        'front_left_axis': steps[0], 'rear_left_axis': steps[1],
                        'rear_right_axis': steps[2], 'front_right_axis': steps[3],
                        'front_left_lift': steps[0], 'rear_left_lift': steps[1],
                        'rear_right_lift': steps[2], 'front_right_lift': steps[3]
                    }
                    
                    self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                print("[REX] Left turn completed")
                
            except Exception as e:
                print(f"[ERROR] REX left turn error: {e}")
            finally:
                self.walking = False

    def rex_turn_right(self):
        """REX kodundaki turn_right() fonksiyonunun Python versiyonu"""
        if self.walking:
            return
        
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            
            print("[REX] REX Right turn - ESP32 style")
            
            try:
                spd = int(settings['speed_delay'] * 1000)
                high = settings['lift_range'] // 3
                
                # REX turn_right sequence - turn_left'in tersi
                phases = [
                    (60, 50, 130, 100, 6+high*3, 33+high*3, 33+high*3, 42+high*3, [3,2,2,2]),
                    (60, 50, 130, 100, 42+high*3, 33+high*3, 33+high*3, 42+high*3, [3,2,2,2]),
                    (80, 70, 110, 80, 42+high*3, 33+high*3, 33+high*3, 6+high*3, [2,2,2,3]),
                    (80, 70, 110, 80, 42+high*3, 33+high*3, 33+high*3, 42+high*3, [2,2,2,3]),
                    (100, 90, 90, 60, 42+high*3, 33+high*3, 6+high*3, 42+high*3, [2,2,3,2]),
                    (100, 90, 90, 60, 42+high*3, 33+high*3, 33+high*3, 42+high*3, [2,2,3,2]),
                    (120, 90, 90, 60, 42+high*3, 6+high*3, 33+high*3, 42+high*3, [2,3,2,2]),
                    (120, 90, 90, 60, 42+high*3, 33+high*3, 33+high*3, 42+high*3, [2,3,2,2])
                ]
                
                for i, (fl_axis, rl_axis, rr_axis, fr_axis, fl_lift, rl_lift, rr_lift, fr_lift, steps) in enumerate(phases):
                    print(f"[REX] Turn right phase {i+1}/8")
                    
                    servo_positions = {
                        'front_left_axis': fl_axis,
                        'rear_left_axis': rl_axis,
                        'rear_right_axis': rr_axis,
                        'front_right_axis': fr_axis,
                        'front_left_lift': fl_lift,
                        'rear_left_lift': rl_lift,
                        'rear_right_lift': rr_lift,
                        'front_right_lift': fr_lift
                    }
                    
                    steps_per_servo = {
                        'front_left_axis': steps[0], 'rear_left_axis': steps[1],
                        'rear_right_axis': steps[2], 'front_right_axis': steps[3],
                        'front_left_lift': steps[0], 'rear_left_lift': steps[1],
                        'rear_right_lift': steps[2], 'front_right_lift': steps[3]
                    }
                    
                    self.rex_servo_move(servo_positions, steps_per_servo, spd/1000)
                
                print("[REX] Right turn completed")
                
            except Exception as e:
                print(f"[ERROR] REX right turn error: {e}")
            finally:
                self.walking = False

    # === REX LEAN FUNCTIONS ===
    
    def rex_lean_left(self):
        """REX kodundaki lean_left() fonksiyonu"""
        servo_positions = {
            'front_left_lift': 60 + 30,   # s12=60
            'rear_left_lift': 60 + 30,    # s22=60  
            'rear_right_lift': 120 + 30,  # s32=120
            'front_right_lift': 120 + 30  # s42=120
        }
        print("[REX] Leaning left...")
        self.rex_servo_move(servo_positions, {servo: 3 for servo in servo_positions.keys()}, 0.05)

    def rex_lean_right(self):
        """REX kodundaki lean_right() fonksiyonu"""
        servo_positions = {
            'front_left_lift': 120 + 30,  # s12=120
            'rear_left_lift': 120 + 30,   # s22=120
            'rear_right_lift': 60 + 30,   # s32=60
            'front_right_lift': 60 + 30   # s42=60
        }
        print("[REX] Leaning right...")
        self.rex_servo_move(servo_positions, {servo: 3 for servo in servo_positions.keys()}, 0.05)

    def rex_lean_forward(self):
        """REX kodundaki lean_forward() fonksiyonu"""
        servo_positions = {
            'front_left_lift': 60 + 30,   # s12=60
            'rear_left_lift': 120 + 30,   # s22=120
            'rear_right_lift': 120 + 30,  # s32=120
            'front_right_lift': 60 + 30   # s42=60
        }
        print("[REX] Leaning forward...")
        self.rex_servo_move(servo_positions, {servo: 3 for servo in servo_positions.keys()}, 0.05)

    def rex_lean_back(self):
        """REX kodundaki lean_back() fonksiyonu"""
        servo_positions = {
            'front_left_lift': 120 + 30,  # s12=120
            'rear_left_lift': 60 + 30,    # s22=60
            'rear_right_lift': 60 + 30,   # s32=60
            'front_right_lift': 120 + 30  # s42=120
        }
        print("[REX] Leaning back...")
        self.rex_servo_move(servo_positions, {servo: 3 for servo in servo_positions.keys()}, 0.05)

    # === SPIDER MOVEMENT FUNCTIONS ===
    
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


def rex_movement():
    """REX Quad style movement control"""
    data = request.get_json()
    action = data.get('action')
    
    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})
    
    try:
        if action == 'rex_forward':
            threading.Thread(target=controller.rex_forward_gait, daemon=True).start()
        elif action == 'rex_backward':
            threading.Thread(target=controller.rex_backward_gait, daemon=True).start()
        elif action == 'rex_left':
            threading.Thread(target=controller.rex_turn_left, daemon=True).start()
        elif action == 'rex_right':
            threading.Thread(target=controller.rex_turn_right, daemon=True).start()
        elif action == 'rex_stabilize':
            threading.Thread(target=controller.rex_stabilize, daemon=True).start()
        elif action == 'rex_lean_left':
            threading.Thread(target=controller.rex_lean_left, daemon=True).start()
        elif action == 'rex_lean_right':
            threading.Thread(target=controller.rex_lean_right, daemon=True).start()
        elif action == 'rex_lean_forward':
            threading.Thread(target=controller.rex_lean_forward, daemon=True).start()
        elif action == 'rex_lean_back':
            threading.Thread(target=controller.rex_lean_back, daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid REX action'})
        
        return jsonify({'status': 'success', 'message': f'Executing REX {action}'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'REX movement error: {str(e)}'})

@app.route('/api/rex_stance_control', methods=['POST'])

def rex_stance_control():
    """REX stance height control"""
    data = request.get_json()
    height = data.get('height', 60)  # Default stance height
    
    # Update controller's stance height for REX functions
    if hasattr(controller, 'rex_stance_height'):
        controller.rex_stance_height = max(30, min(70, int(height)))
    else:
        controller.rex_stance_height = 60
    
    return jsonify({
        'status': 'success',
        'message': f'REX stance height set to {controller.rex_stance_height}',
        'height': controller.rex_stance_height
    })

