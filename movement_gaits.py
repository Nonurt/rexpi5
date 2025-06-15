# movement_gaits.py
import time


class MovementGaits:
    """
    Robotun tüm yürüme, dönüş, duruş ve özel hareket algoritmalarını içeren
    bir "mixin" sınıfıdır. Bu sınıf, ana RobotController tarafından miras alınır
    ve onun özelliklerini (self.walking, self.set_servo_angle vb.) kullanır.
    """

    # === REX QUAD STYLE WALKING ALGORITHM ===

    def rex_servo_move(self, servo_positions, steps_per_servo, delay):
        """REX tarzı mikro-adımlı servo hareketi - ESP32 srv() fonksiyonunun Python versiyonu"""
        current_positions = {}
        target_positions = {}
        step_sizes = {}

        # Başlangıç pozisyonlarını al ve adım büyüklüklerini hesapla
        for servo_name, target in servo_positions.items():
            if servo_name in self.current_angles:
                current_positions[servo_name] = self.current_angles[servo_name]
                target_positions[servo_name] = target

                speed = steps_per_servo.get(servo_name, 3)
                diff = target - current_positions[servo_name]
                step_sizes[servo_name] = diff / max(abs(diff), 1) * speed if diff != 0 else 0

        # Tüm servoların birlikte hareket etmesi için en uzun mesafeyi bul
        if not servo_positions: return
        max_steps = max([abs(target_positions.get(s, 0) - current_positions.get(s, 0)) for s in servo_positions.keys()],
                        default=1)

        for _ in range(int(max_steps / list(steps_per_servo.values())[0]) + 1):
            all_reached = True
            for servo_name in servo_positions.keys():
                if servo_name in current_positions:
                    current = current_positions[servo_name]
                    target = target_positions[servo_name]
                    step_size = step_sizes[servo_name]

                    if abs(target - current) > abs(step_size):
                        new_position = current + step_size
                        all_reached = False
                    else:
                        new_position = target

                    self.set_servo_angle(servo_name, new_position)
                    current_positions[servo_name] = new_position

            time.sleep(delay)
            if all_reached:
                break

    def rex_stabilize(self):
        """REX stance position - REX kodundaki stabilize() fonksiyonu"""
        stance_height = 60
        servo_positions = {
            'front_left_axis': 90, 'front_right_axis': 90,
            'rear_left_axis': 90, 'rear_right_axis': 90,
            'front_left_lift': stance_height + 30,
            'front_right_lift': stance_height + 30,
            'rear_left_lift': stance_height + 30,
            'rear_right_lift': stance_height + 30
        }
        steps_per_servo = {servo: 4 for servo in servo_positions.keys()}
        print("[REX] Stabilizing to stance position...")
        self.rex_servo_move(servo_positions, steps_per_servo, 0.005)

        # movement_gaits.py dosyanıza bu fonksiyonları güncelleyin

        def rex_forward_gait(self):
            """REX kodundaki forward() fonksiyonunun Python versiyonu"""
            if self.walking: return
            with self.walking_lock:
                self.walking = True
                settings = self.get_power_settings()
                print("[REX] REX Forward gait - 8 phase ESP32 style")
                try:
                    spd = settings['speed_delay']
                    high = settings['lift_range'] // 3
                    bigStep = settings['axis_range'] > 40

                    # --- DÜZELTİLMİŞ MANTIK ---
                    # Servo adında 'left' varsa yüksek açı (120/150), 'right' varsa düşük açı (60/30) verilir.
                    def HIP_FWD(servo_name):
                        if 'left' in servo_name:
                            return 150 if bigStep else 120
                        else:  # right
                            return 30 if bigStep else 60

                    # Yukarıdaki mantığın tam tersi.
                    def HIP_BCK(servo_name):
                        if 'left' in servo_name:
                            return 30 if bigStep else 60
                        else:  # right
                            return 150 if bigStep else 120

                    # --- DÜZELTME SONU ---

                    phases = [
                        {'front_right_lift': 6 + high * 3, 'front_left_axis': HIP_BCK('front_left'),
                         'front_right_axis': HIP_FWD('front_right')},
                        {'front_right_lift': 42 + high * 3},
                        {'rear_right_lift': 6 + high * 3, 'rear_left_axis': HIP_BCK('rear_left'),
                         'rear_right_axis': HIP_FWD('rear_right')},
                        {'rear_right_lift': 33 + high * 3},
                        {'front_left_lift': 6 + high * 3, 'front_left_axis': HIP_FWD('front_left'),
                         'front_right_axis': HIP_BCK('front_right')},
                        {'front_left_lift': 42 + high * 3},
                        {'rear_left_lift': 6 + high * 3, 'rear_left_axis': HIP_FWD('rear_left'),
                         'rear_right_axis': HIP_BCK('rear_right')},
                        {'front_left_axis': 90, 'front_right_axis': 90, 'rear_left_axis': 90, 'rear_right_axis': 90,
                         'rear_left_lift': 33 + high * 3}
                    ]
                    for i, p in enumerate(phases):
                        print(f"[REX] Forward phase {i + 1}/8")
                        self.rex_servo_move(p, {s: 3 for s in p.keys()}, spd)
                finally:
                    self.walking = False
                    self.rex_stabilize()

        def rex_backward_gait(self):
            """REX kodundaki back() fonksiyonunun Python versiyonu"""
            if self.walking: return
            with self.walking_lock:
                self.walking = True
                settings = self.get_power_settings()
                print("[REX] REX Backward gait - ESP32 style")
                try:
                    spd = settings['speed_delay']
                    high = settings['lift_range'] // 3
                    bigStep = settings['axis_range'] > 40

                    # --- DÜZELTİLMİŞ MANTIK ---
                    def HIP_FWD(servo_name):
                        if 'left' in servo_name:
                            return 150 if bigStep else 120
                        else:  # right
                            return 30 if bigStep else 60

                    def HIP_BCK(servo_name):
                        if 'left' in servo_name:
                            return 30 if bigStep else 60
                        else:  # right
                            return 150 if bigStep else 120

                    # --- DÜZELTME SONU ---

                    # Geri yürüme, ileri yürümenin tersi mantıkla çalışır
                    phases = [
                        {'front_left_lift': 6 + high * 3, 'front_left_axis': HIP_BCK('front_left'),
                         'front_right_axis': HIP_FWD('front_right')},
                        {'front_left_lift': 42 + high * 3},
                        {'rear_left_lift': 6 + high * 3, 'rear_left_axis': HIP_BCK('rear_left'),
                         'rear_right_axis': HIP_FWD('rear_right')},
                        {'rear_left_lift': 33 + high * 3},
                        {'front_right_lift': 6 + high * 3, 'front_left_axis': HIP_FWD('front_left'),
                         'front_right_axis': HIP_BCK('front_right')},
                        {'front_right_lift': 42 + high * 3},
                        {'rear_right_lift': 6 + high * 3, 'rear_left_axis': HIP_FWD('rear_left'),
                         'rear_right_axis': HIP_BCK('rear_right')},
                        {'front_left_axis': 90, 'front_right_axis': 90, 'rear_left_axis': 90, 'rear_right_axis': 90,
                         'rear_right_lift': 33 + high * 3}
                    ]
                    for i, p in enumerate(phases):
                        print(f"[REX] Backward phase {i + 1}/8")
                        self.rex_servo_move(p, {s: 3 for s in p.keys()}, spd)
                finally:
                    self.walking = False
                    self.rex_stabilize()

    def rex_turn_left(self):
        """REX kodundaki turn_left() fonksiyonunun Python versiyonu"""
        if self.walking: return
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            print("[REX] REX Left turn - ESP32 style")
            try:
                spd = settings['speed_delay']
                high = settings['lift_range'] // 3
                phases = [
                    (120, 90, 90, 60, 42 + high * 3, 6 + high * 3, 33 + high * 3, 42 + high * 3),
                    (120, 90, 90, 60, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3),
                    (100, 70, 110, 80, 42 + high * 3, 33 + high * 3, 6 + high * 3, 42 + high * 3),
                    (100, 70, 110, 80, 42 + high * 3, 33 + high * 3, 33 + high * 3, 24 + high * 3),
                    (80, 50, 130, 100, 42 + high * 3, 33 + high * 3, 33 + high * 3, 6 + high * 3),
                    (80, 50, 130, 100, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3),
                    (120, 90, 90, 60, 6 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3),
                    (120, 90, 90, 60, 42 + high * 3, 33 + high * 3, 33 + high * 3, 33 + high * 3)
                ]
                for i, p in enumerate(phases):
                    servo_pos = {'front_left_axis': p[0], 'rear_left_axis': p[1], 'rear_right_axis': p[2],
                                 'front_right_axis': p[3], 'front_left_lift': p[4], 'rear_left_lift': p[5],
                                 'rear_right_lift': p[6], 'front_right_lift': p[7]}
                    self.rex_servo_move(servo_pos, {s: 3 for s in servo_pos.keys()}, spd)
            finally:
                self.walking = False
                self.rex_stabilize()

    def rex_turn_right(self):
        """REX kodundaki turn_right() fonksiyonunun Python versiyonu"""
        if self.walking: return
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            print("[REX] REX Right turn - ESP32 style")
            try:
                spd = settings['speed_delay']
                high = settings['lift_range'] // 3
                phases = [
                    (60, 50, 130, 100, 6 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3),
                    (60, 50, 130, 100, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3),
                    (80, 70, 110, 80, 42 + high * 3, 33 + high * 3, 33 + high * 3, 6 + high * 3),
                    (80, 70, 110, 80, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3),
                    (100, 90, 90, 60, 42 + high * 3, 33 + high * 3, 6 + high * 3, 42 + high * 3),
                    (100, 90, 90, 60, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3),
                    (120, 90, 90, 60, 42 + high * 3, 6 + high * 3, 33 + high * 3, 42 + high * 3),
                    (120, 90, 90, 60, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3)
                ]
                for i, p in enumerate(phases):
                    servo_pos = {'front_left_axis': p[0], 'rear_left_axis': p[1], 'rear_right_axis': p[2],
                                 'front_right_axis': p[3], 'front_left_lift': p[4], 'rear_left_lift': p[5],
                                 'rear_right_lift': p[6], 'front_right_lift': p[7]}
                    self.rex_servo_move(servo_pos, {s: 3 for s in servo_pos.keys()}, spd)
            finally:
                self.walking = False
                self.rex_stabilize()

    # === REX LEAN FUNCTIONS ===
    def rex_lean_left(self):
        servo_positions = {'front_left_lift': 90, 'rear_left_lift': 90, 'rear_right_lift': 150, 'front_right_lift': 150}
        self.rex_servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, 0.05)

    def rex_lean_right(self):
        servo_positions = {'front_left_lift': 150, 'rear_left_lift': 150, 'rear_right_lift': 90, 'front_right_lift': 90}
        self.rex_servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, 0.05)

    def rex_lean_forward(self):
        servo_positions = {'front_left_lift': 90, 'rear_left_lift': 150, 'rear_right_lift': 150, 'front_right_lift': 90}
        self.rex_servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, 0.05)

    def rex_lean_back(self):
        servo_positions = {'front_left_lift': 150, 'rear_left_lift': 90, 'rear_right_lift': 90, 'front_right_lift': 150}
        self.rex_servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, 0.05)

    # === SPIDER MOVEMENT FUNCTIONS (Artık REX'i çağırıyor) ===
    def spider_walk_forward(self):
        print("[MAIN] Using REX forward gait instead of spider")
        self.rex_forward_gait()

    def spider_turn_left(self):
        print("[MAIN] Using REX left turn instead of spider")
        self.rex_turn_left()

    def spider_turn_right(self):
        print("[MAIN] Using REX right turn instead of spider")
        self.rex_turn_right()

    def spider_back_away(self):
        print("[MAIN] Using REX backward gait instead of spider")
        self.rex_backward_gait()

    def spider_stance_position(self):
        print("[MAIN] Using REX stabilize instead of spider stance")
        self.rex_stabilize()

    # === POSTURES AND ADVANCED LEG CONTROL ===
    def spider_defensive_posture(self):
        if self.walking: return
        with self.walking_lock:
            print("[SPIDER] Defensive posture...")
            self.set_servo_angle('front_left_axis', 45)
            self.set_servo_angle('front_right_axis', 135)
            self.set_servo_angle('front_left_lift', 70)
            self.set_servo_angle('front_right_lift', 70)
            self.set_servo_angle('rear_left_axis', 45)
            self.set_servo_angle('rear_right_axis', 135)
            self.set_servo_angle('rear_left_lift', 70)
            self.set_servo_angle('rear_right_lift', 70)
            time.sleep(1.0)
            print("[SPIDER] Defensive posture ready")

    def spider_attack_posture(self):
        if self.walking: return
        with self.walking_lock:
            print("[SPIDER] Attack posture...")
            self.set_servo_angle('front_left_lift', 45)
            self.set_servo_angle('front_right_lift', 45)
            self.set_servo_angle('front_left_axis', 70)
            self.set_servo_angle('front_right_axis', 110)
            self.set_servo_angle('rear_left_axis', 90)
            self.set_servo_angle('rear_right_axis', 90)
            self.set_servo_angle('rear_left_lift', 90)
            self.set_servo_angle('rear_right_lift', 90)
            time.sleep(1.0)
            print("[SPIDER] Attack posture ready")

    def spider_crouch_low(self):
        if self.walking: return
        with self.walking_lock:
            print("[SPIDER] Crouching low...")
            for servo_name in ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']:
                self.set_servo_angle(servo_name, 120)
            for servo_name in ['front_left_axis', 'front_right_axis', 'rear_left_axis', 'rear_right_axis']:
                self.set_servo_angle(servo_name, 90)
            time.sleep(1.0)
            print("[SPIDER] Crouched low")

    def lift_legs(self, leg_group="all"):
        if self.walking: return
        with self.walking_lock:
            settings = self.get_power_settings()
            lift_angle = 90 - settings['lift_range']
            legs_to_move = self._get_leg_group(leg_group, '_lift')
            print(f"[LEG] Lifting {leg_group} legs to {lift_angle}°")
            for leg in legs_to_move: self.smooth_servo_move(leg, lift_angle, steps=3)
            time.sleep(0.5)

    def lower_legs(self, leg_group="all"):
        if self.walking: return
        with self.walking_lock:
            legs_to_move = self._get_leg_group(leg_group, '_lift')
            print(f"[LEG] Lowering {leg_group} legs to 90°")
            for leg in legs_to_move: self.smooth_servo_move(leg, 90, steps=3)
            time.sleep(0.5)

    def spread_legs(self, leg_group="all"):
        if self.walking: return
        with self.walking_lock:
            settings = self.get_power_settings()
            spread_angle = settings['axis_range']
            if leg_group == 'all' or leg_group == 'front':
                self.set_servo_angle('front_left_axis', 90 + spread_angle)
                self.set_servo_angle('front_right_axis', 90 - spread_angle)
            if leg_group == 'all' or leg_group == 'rear':
                self.set_servo_angle('rear_left_axis', 90 + spread_angle)
                self.set_servo_angle('rear_right_axis', 90 - spread_angle)
            print(f"[LEG] {leg_group.title()} legs spread wide")
            time.sleep(0.5)

    def center_legs(self, leg_group="all"):
        if self.walking: return
        with self.walking_lock:
            legs_to_move = self._get_leg_group(leg_group, '_axis')
            print(f"[LEG] Centering {leg_group} legs to 90°")
            for leg in legs_to_move: self.set_servo_angle(leg, 90)
            time.sleep(0.5)

    def _get_leg_group(self, group_name, suffix):
        groups = {
            'front': ['front_left', 'front_right'],
            'rear': ['rear_left', 'rear_right'],
            'left': ['front_left', 'rear_left'],
            'right': ['front_right', 'rear_right'],
            'all': ['front_left', 'front_right', 'rear_left', 'rear_right']
        }
        return [f"{name}{suffix}" for name in groups.get(group_name, [])]

    # === ADVANCED GAIT PATTERNS ===
    def front_lift_gait(self, direction="forward"):
        if self.walking: return
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            print(f"[GAIT] Front lift gait - {direction}")
            try:
                self.lift_legs('front')
                time.sleep(settings['speed_delay'] * 2)
                angle_mod = 1 if direction == "forward" else -1
                self.set_servo_angle('front_left_axis', 90 + settings['axis_range'] * angle_mod)
                self.set_servo_angle('front_right_axis', 90 - settings['axis_range'] * angle_mod)
                time.sleep(settings['speed_delay'] * 3)
                self.lower_legs('front')
                time.sleep(settings['speed_delay'] * 2)
                self.set_servo_angle('rear_left_axis', 90 - (settings['axis_range'] // 2) * angle_mod)
                self.set_servo_angle('rear_right_axis', 90 + (settings['axis_range'] // 2) * angle_mod)
                time.sleep(settings['speed_delay'] * 2)
            finally:
                self.walking = False
                self.spider_stance_position()

    def rear_drive_gait(self, direction="forward"):
        if self.walking: return
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            print(f"[GAIT] Rear drive gait - {direction}")
            try:
                angle_mod = 1 if direction == "forward" else -1
                self.lift_legs('rear')
                time.sleep(settings['speed_delay'] * 2)
                self.set_servo_angle('rear_left_axis', 90 + settings['axis_range'] * angle_mod)
                self.set_servo_angle('rear_right_axis', 90 - settings['axis_range'] * angle_mod)
                time.sleep(settings['speed_delay'] * 3)
                self.lower_legs('rear')
                time.sleep(settings['speed_delay'] * 1)
                self.set_servo_angle('rear_left_axis', 90 - settings['axis_range'] * angle_mod)
                self.set_servo_angle('rear_right_axis', 90 + settings['axis_range'] * angle_mod)
                time.sleep(settings['speed_delay'] * 3)
            finally:
                self.walking = False
                self.spider_stance_position()

    def alternating_pairs_gait(self, direction="forward"):
        if self.walking: return
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            print(f"[GAIT] Alternating pairs gait - {direction}")
            try:
                angle_mod = 1 if direction == "forward" else -1
                # Front legs
                self.lift_legs('front')
                time.sleep(settings['speed_delay'])
                self.set_servo_angle('front_left_axis', 90 + settings['axis_range'] * angle_mod)
                self.set_servo_angle('front_right_axis', 90 - settings['axis_range'] * angle_mod)
                time.sleep(settings['speed_delay'] * 2)
                self.lower_legs('front')
                time.sleep(settings['speed_delay'])
                # Rear legs
                self.lift_legs('rear')
                time.sleep(settings['speed_delay'])
                self.set_servo_angle('rear_left_axis', 90 + settings['axis_range'] * angle_mod)
                self.set_servo_angle('rear_right_axis', 90 - settings['axis_range'] * angle_mod)
                time.sleep(settings['speed_delay'] * 2)
                self.lower_legs('rear')
                time.sleep(settings['speed_delay'])
            finally:
                self.walking = False
                self.spider_stance_position()