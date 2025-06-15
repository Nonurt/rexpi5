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

        for servo_name, target in servo_positions.items():
            if servo_name in self.current_angles:
                current_positions[servo_name] = self.current_angles[servo_name]
                target_positions[servo_name] = target
                speed = steps_per_servo.get(servo_name, 3)
                diff = target - current_positions[servo_name]
                step_sizes[servo_name] = diff / max(abs(diff), 1) * speed if diff != 0 else 0

        if not servo_positions: return
        num_steps = 30  # Sabit bir adım sayısı ile daha tutarlı bir hız elde edebiliriz.
        for _ in range(num_steps):
            all_reached = True
            for servo_name in servo_positions.keys():
                if servo_name in current_positions:
                    current = current_positions[servo_name]
                    target = target_positions[servo_name]

                    # Adım büyüklüğünü kalan mesafeye göre ayarla
                    remaining_dist = target - current
                    step = remaining_dist / (num_steps - _) if (num_steps - _) > 0 else remaining_dist

                    new_position = current + step

                    if abs(target - new_position) < 1.0:
                        new_position = float(target)
                    else:
                        all_reached = False

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

                def HIP_FWD(servo_name):
                    if 'left' in servo_name:
                        return 150 if bigStep else 120
                    else:
                        return 30 if bigStep else 60

                def HIP_BCK(servo_name):
                    if 'left' in servo_name:
                        return 30 if bigStep else 60
                    else:
                        return 150 if bigStep else 120

                phases = [
                    {'front_right_lift': 6 + high * 3, 'front_left_axis': HIP_BCK('fl'),
                     'front_right_axis': HIP_FWD('fr')},
                    {'front_right_lift': 42 + high * 3},
                    {'rear_right_lift': 6 + high * 3, 'rear_left_axis': HIP_BCK('rl'),
                     'rear_right_axis': HIP_FWD('rr')},
                    {'rear_right_lift': 33 + high * 3},
                    {'front_left_lift': 6 + high * 3, 'front_left_axis': HIP_FWD('fl'),
                     'front_right_axis': HIP_BCK('fr')},
                    {'front_left_lift': 42 + high * 3},
                    {'rear_left_lift': 6 + high * 3, 'rear_left_axis': HIP_FWD('rl'), 'rear_right_axis': HIP_BCK('rr')},
                    {'front_left_axis': 90, 'front_right_axis': 90, 'rear_left_axis': 90, 'rear_right_axis': 90,
                     'rear_left_lift': 33 + high * 3}
                ]
                for p in phases:
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

                def HIP_FWD(servo_name):
                    if 'left' in servo_name:
                        return 150 if bigStep else 120
                    else:
                        return 30 if bigStep else 60

                def HIP_BCK(servo_name):
                    if 'left' in servo_name:
                        return 30 if bigStep else 60
                    else:
                        return 150 if bigStep else 120

                phases = [
                    {'front_left_lift': 6 + high * 3, 'front_left_axis': HIP_BCK('fl'),
                     'front_right_axis': HIP_FWD('fr')},
                    {'front_left_lift': 42 + high * 3},
                    {'rear_left_lift': 6 + high * 3, 'rear_left_axis': HIP_BCK('rl'), 'rear_right_axis': HIP_FWD('rr')},
                    {'rear_left_lift': 33 + high * 3},
                    {'front_right_lift': 6 + high * 3, 'front_left_axis': HIP_FWD('fl'),
                     'front_right_axis': HIP_BCK('fr')},
                    {'front_right_lift': 42 + high * 3},
                    {'rear_right_lift': 6 + high * 3, 'rear_left_axis': HIP_FWD('rl'),
                     'rear_right_axis': HIP_BCK('rr')},
                    {'front_left_axis': 90, 'front_right_axis': 90, 'rear_left_axis': 90, 'rear_right_axis': 90,
                     'rear_right_lift': 33 + high * 3}
                ]
                for p in phases:
                    self.rex_servo_move(p, {s: 3 for s in p.keys()}, spd)
            finally:
                self.walking = False
                self.rex_stabilize()

    # --- YENİDEN DÜZENLENMİŞ DÖNÜŞ FONKSİYONLARI ---
    def rex_turn_left(self):
        """REX sola dönüş - daha okunaklı ve pürüzsüz hale getirildi."""
        if self.walking: return
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            print("[REX] REX Left turn - Refactored")
            try:
                spd = settings['speed_delay']
                high = settings['lift_range'] // 3
                h1, h2, h3, h4 = 42 + high * 3, 33 + high * 3, 6 + high * 3, 24 + high * 3

                phases = [
                    {'front_left_axis': 120, 'rear_left_axis': 90, 'rear_right_axis': 90, 'front_right_axis': 60,
                     'front_left_lift': h1, 'rear_left_lift': h3, 'rear_right_lift': h2, 'front_right_lift': h1},
                    {'front_left_lift': h1, 'rear_left_lift': h2, 'rear_right_lift': h2, 'front_right_lift': h1},
                    {'front_left_axis': 100, 'rear_left_axis': 70, 'rear_right_axis': 110, 'front_right_axis': 80,
                     'front_left_lift': h1, 'rear_left_lift': h2, 'rear_right_lift': h3, 'front_right_lift': h1},
                    {'rear_right_lift': h2, 'front_right_lift': h4},
                    {'front_left_axis': 80, 'rear_left_axis': 50, 'rear_right_axis': 130, 'front_right_axis': 100,
                     'front_left_lift': h1, 'rear_left_lift': h2, 'rear_right_lift': h2, 'front_right_lift': h3},
                    {'front_right_lift': h1},
                    {'front_left_axis': 120, 'rear_left_axis': 90, 'rear_right_axis': 90, 'front_right_axis': 60,
                     'front_left_lift': h3, 'rear_left_lift': h2, 'rear_right_lift': h2, 'front_right_lift': h1},
                    {'front_left_lift': h1, 'rear_right_lift': h2}
                ]
                for p in phases:
                    self.rex_servo_move(p, {s: 3 for s in p.keys()}, spd)
            finally:
                self.walking = False
                self.rex_stabilize()

    def rex_turn_right(self):
        """REX sağa dönüş - daha okunaklı ve pürüzsüz hale getirildi."""
        if self.walking: return
        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()
            print("[REX] REX Right turn - Refactored")
            try:
                spd = settings['speed_delay']
                high = settings['lift_range'] // 3
                h1, h2, h3 = 42 + high * 3, 33 + high * 3, 6 + high * 3

                phases = [
                    {'front_left_axis': 60, 'rear_left_axis': 50, 'rear_right_axis': 130, 'front_right_axis': 100,
                     'front_left_lift': h3, 'rear_left_lift': h2, 'rear_right_lift': h2, 'front_right_lift': h1},
                    {'front_left_lift': h1},
                    {'front_left_axis': 80, 'rear_left_axis': 70, 'rear_right_axis': 110, 'front_right_axis': 80,
                     'front_left_lift': h1, 'rear_left_lift': h2, 'rear_right_lift': h2, 'front_right_lift': h3},
                    {'front_right_lift': h1},
                    {'front_left_axis': 100, 'rear_left_axis': 90, 'rear_right_axis': 90, 'front_right_axis': 60,
                     'front_left_lift': h1, 'rear_left_lift': h2, 'rear_right_lift': h3, 'front_right_lift': h1},
                    {'rear_right_lift': h2},
                    {'front_left_axis': 120, 'rear_left_axis': 90, 'rear_right_axis': 90, 'front_right_axis': 60,
                     'front_left_lift': h1, 'rear_left_lift': h3, 'rear_right_lift': h2, 'front_right_lift': h1},
                    {'rear_left_lift': h2}
                ]
                for p in phases:
                    self.rex_servo_move(p, {s: 3 for s in p.keys()}, spd)
            finally:
                self.walking = False
                self.rex_stabilize()

    # -----------------------------------------------

    # === REX LEAN FUNCTIONS ===
    def rex_lean_left(self):
        servo_positions = {'front_left_lift': 90, 'rear_left_lift': 90, 'rear_right_lift': 150, 'front_right_lift': 150}
        self.rex_servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, 0.01)

    def rex_lean_right(self):
        servo_positions = {'front_left_lift': 150, 'rear_left_lift': 150, 'rear_right_lift': 90, 'front_right_lift': 90}
        self.rex_servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, 0.01)

    def rex_lean_forward(self):
        servo_positions = {'front_left_lift': 90, 'rear_left_lift': 150, 'rear_right_lift': 150, 'front_right_lift': 90}
        self.rex_servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, 0.01)

    def rex_lean_back(self):
        servo_positions = {'front_left_lift': 150, 'rear_left_lift': 90, 'rear_right_lift': 90, 'front_right_lift': 150}
        self.rex_servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, 0.01)

    # === SPIDER MOVEMENT FUNCTIONS (Artık REX'i çağırıyor) ===
    def spider_walk_forward(self):
        print("[MAIN] Using REX forward gait")
        self.rex_forward_gait()

    def spider_turn_left(self):
        print("[MAIN] Using REX left turn")
        self.rex_turn_left()

    def spider_turn_right(self):
        print("[MAIN] Using REX right turn")
        self.rex_turn_right()

    def spider_back_away(self):
        print("[MAIN] Using REX backward gait")
        self.rex_backward_gait()

    def spider_stance_position(self):
        print("[MAIN] Using REX stabilize")
        self.rex_stabilize()

    # --- YENİDEN DÜZENLENMİŞ DURUŞ FONKSİYONLARI ---
    def spider_defensive_posture(self):
        if self.walking: return
        with self.walking_lock:
            print("[SPIDER] Defensive posture (smooth)...")
            positions = {
                'front_left_axis': 45, 'front_right_axis': 135,
                'rear_left_axis': 45, 'rear_right_axis': 135,
                'front_left_lift': 70, 'front_right_lift': 70,
                'rear_left_lift': 70, 'rear_right_lift': 70
            }
            self.rex_servo_move(positions, {s: 3 for s in positions.keys()}, 0.01)
            print("[SPIDER] Defensive posture ready")

    def spider_attack_posture(self):
        if self.walking: return
        with self.walking_lock:
            print("[SPIDER] Attack posture (smooth)...")
            positions = {
                'front_left_lift': 45, 'front_right_lift': 45,
                'front_left_axis': 70, 'front_right_axis': 110,
                'rear_left_axis': 90, 'rear_right_axis': 90,
                'rear_left_lift': 90, 'rear_right_lift': 90
            }
            self.rex_servo_move(positions, {s: 3 for s in positions.keys()}, 0.01)
            print("[SPIDER] Attack posture ready")

    def spider_crouch_low(self):
        if self.walking: return
        with self.walking_lock:
            print("[SPIDER] Crouching low (smooth)...")
            positions = {
                'front_left_lift': 120, 'front_right_lift': 120,
                'rear_left_lift': 120, 'rear_right_lift': 120,
                'front_left_axis': 90, 'front_right_axis': 90,
                'rear_left_axis': 90, 'rear_right_axis': 90
            }
            self.rex_servo_move(positions, {s: 3 for s in positions.keys()}, 0.01)
            print("[SPIDER] Crouched low")

    # -----------------------------------------------

    # --- BACAK KONTROL FONKSİYONLARI (Pürüzsüzleştirildi) ---
    def lift_legs(self, leg_group="all"):
        if self.walking: return
        with self.walking_lock:
            settings = self.get_power_settings()
            lift_angle = 90 - settings['lift_range']
            legs_to_move = self._get_leg_group(leg_group, '_lift')
            print(f"[LEG] Lifting {leg_group} legs to {lift_angle}°")
            positions = {leg: lift_angle for leg in legs_to_move}
            self.rex_servo_move(positions, {s: 3 for s in positions.keys()}, 0.01)

    def lower_legs(self, leg_group="all"):
        if self.walking: return
        with self.walking_lock:
            legs_to_move = self._get_leg_group(leg_group, '_lift')
            print(f"[LEG] Lowering {leg_group} legs to 90°")
            positions = {leg: 90 for leg in legs_to_move}
            self.rex_servo_move(positions, {s: 3 for s in positions.keys()}, 0.01)

    def spread_legs(self, leg_group="all"):
        if self.walking: return
        with self.walking_lock:
            settings = self.get_power_settings()
            spread_angle = settings['axis_range']
            positions = {}
            if leg_group in ['all', 'front']:
                positions['front_left_axis'] = 90 + spread_angle
                positions['front_right_axis'] = 90 - spread_angle
            if leg_group in ['all', 'rear']:
                positions['rear_left_axis'] = 90 + spread_angle
                positions['rear_right_axis'] = 90 - spread_angle
            print(f"[LEG] {leg_group.title()} legs spread wide")
            self.rex_servo_move(positions, {s: 3 for s in positions.keys()}, 0.01)

    def center_legs(self, leg_group="all"):
        if self.walking: return
        with self.walking_lock:
            legs_to_move = self._get_leg_group(leg_group, '_axis')
            print(f"[LEG] Centering {leg_group} legs to 90°")
            positions = {leg: 90 for leg in legs_to_move}
            self.rex_servo_move(positions, {s: 3 for s in positions.keys()}, 0.01)

    # -----------------------------------------------

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
    # Bu gelişmiş yürüme desenleri, zamanlamaları hassas olduğu için şimdilik değiştirilmedi.
    # Temel bacak hareketleri (lift/lower) artık daha pürüzsüz olduğu için bu adımlar da daha iyi görünecektir.
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
                self.lift_legs('front')
                time.sleep(settings['speed_delay'])
                self.set_servo_angle('front_left_axis', 90 + settings['axis_range'] * angle_mod)
                self.set_servo_angle('front_right_axis', 90 - settings['axis_range'] * angle_mod)
                time.sleep(settings['speed_delay'] * 2)
                self.lower_legs('front')
                time.sleep(settings['speed_delay'])
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