import time
import threading  # Eğer sınıf dışında test edilirse hata almamak için


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


# SATIR 644'te şu şekilde düzelt:

def _execute_crawl_forward(self):  # 👈 def'in başına 4 space ekle
    """Statik crawl/creep ileri yürüme – her adımda yalnızca bir bacak havada."""
    settings = self.get_power_settings()
    L = settings['lift_range']
    A = settings['axis_range']
    d = settings['speed_delay']
    # Adım sırası: Ön Sağ → Arka Sol → Ön Sol → Arka Sağ
    STEP = [
        ('front_right_lift', 'front_right_axis', 90 - A),
        ('rear_left_lift', 'rear_left_axis', 90 + A),
        ('front_left_lift', 'front_left_axis', 90 + A),
        ('rear_right_lift', 'rear_right_axis', 90 - A),
    ]
    for lift, axis, tgt in STEP:
        # 1. Ayağı kaldır
        self.smooth_servo_move(lift, 90 + L, steps=4)
        time.sleep(d)
        # 2. Bacağı ileri savur
        self.smooth_servo_move(axis, tgt, steps=5)
        time.sleep(d * 2)
        # 3. Ayağı indir
        self.smooth_servo_move(lift, 90, steps=4)
        time.sleep(d)
        # 4. Destek bacaklarla gövdeyi hafifçe it
        if 'left' in axis:
            self.smooth_servo_move('front_right_axis', 90 - A / 3, steps=2)
            self.smooth_servo_move('rear_right_axis', 90 - A / 3, steps=2)
        else:
            self.smooth_servo_move('front_left_axis', 90 + A / 3, steps=2)
            self.smooth_servo_move('rear_left_axis', 90 + A / 3, steps=2)
        time.sleep(settings['movement_speed'])


def _execute_trot_forward(self):  # 👈 def'in başına 4 space ekle
    """İleri yürüme gaiti"""
    settings = self.get_power_settings()
    lift_angle = settings['lift_range']
    step_angle = settings['axis_range']
    delay = settings['speed_delay']

    # PHASE 1: Lift diagonal pair 1 (Front-Left + Rear-Right)
    print("[GAIT] Phase 1: Lifting FL + RR")
    self.smooth_servo_move('front_left_lift', 90 - lift_angle, steps=3)
    self.smooth_servo_move('rear_right_lift', 90 - lift_angle, steps=3)
    time.sleep(delay * 2)

    # PHASE 2: Move lifted legs forward
    print("[GAIT] Phase 2: Moving FL + RR forward")
    self.smooth_servo_move('front_left_axis', 90 + step_angle, steps=4)
    self.smooth_servo_move('rear_right_axis', 90 - step_angle, steps=4)
    time.sleep(delay * 3)

    # PHASE 3: Lower diagonal pair 1
    print("[GAIT] Phase 3: Lowering FL + RR")
    self.smooth_servo_move('front_left_lift', 90, steps=3)
    self.smooth_servo_move('rear_right_lift', 90, steps=3)
    time.sleep(delay * 2)

    # PHASE 4: Move ground legs backward (propulsion)
    print("[GAIT] Phase 4: Ground propulsion FR + RL")
    self.smooth_servo_move('front_right_axis', 90 + step_angle, steps=4)
    self.smooth_servo_move('rear_left_axis', 90 - step_angle, steps=4)
    time.sleep(delay * 2)

    # PHASE 5: Lift diagonal pair 2 (Front-Right + Rear-Left)
    print("[GAIT] Phase 5: Lifting FR + RL")
    self.smooth_servo_move('front_right_lift', 90 - lift_angle, steps=3)
    self.smooth_servo_move('rear_left_lift', 90 - lift_angle, steps=3)
    time.sleep(delay * 2)

    # PHASE 6: Move lifted legs forward
    print("[GAIT] Phase 6: Moving FR + RL forward")
    self.smooth_servo_move('front_right_axis', 90 - step_angle, steps=4)
    self.smooth_servo_move('rear_left_axis', 90 + step_angle, steps=4)
    time.sleep(delay * 3)

    # PHASE 7: Lower diagonal pair 2
    print("[GAIT] Phase 7: Lowering FR + RL")
    self.smooth_servo_move('front_right_lift', 90, steps=3)
    self.smooth_servo_move('rear_left_lift', 90, steps=3)
    time.sleep(delay * 2)

    # PHASE 8: Move ground legs backward (propulsion)
    print("[GAIT] Phase 8: Ground propulsion FL + RR")
    self.smooth_servo_move('front_left_axis', 90 - step_angle, steps=4)
    self.smooth_servo_move('rear_right_axis', 90 + step_angle, steps=4)
    time.sleep(delay * 2)


def _execute_backward_gait(self):
    """Geri yürüme gaiti"""
    settings = self.get_power_settings()
    lift_angle = settings['lift_range']
    step_angle = settings['axis_range']
    delay = settings['speed_delay']

    print("[GAIT] Backward gait starting...")

    # PHASE 1: Lift FL + RR
    self.smooth_servo_move('front_left_lift', 90 - lift_angle, steps=3)
    self.smooth_servo_move('rear_right_lift', 90 - lift_angle, steps=3)
    time.sleep(delay * 2)

    # PHASE 2: Move backward
    self.smooth_servo_move('front_left_axis', 90 - step_angle, steps=4)
    self.smooth_servo_move('rear_right_axis', 90 + step_angle, steps=4)
    time.sleep(delay * 3)

    # PHASE 3: Lower
    self.smooth_servo_move('front_left_lift', 90, steps=3)
    self.smooth_servo_move('rear_right_lift', 90, steps=3)
    time.sleep(delay * 2)

    # Continue with other diagonal pair
    self.smooth_servo_move('front_right_lift', 90 - lift_angle, steps=3)
    self.smooth_servo_move('rear_left_lift', 90 - lift_angle, steps=3)
    time.sleep(delay * 2)

    self.smooth_servo_move('front_right_axis', 90 + step_angle, steps=4)
    self.smooth_servo_move('rear_left_axis', 90 - step_angle, steps=4)
    time.sleep(delay * 3)

    self.smooth_servo_move('front_right_lift', 90, steps=3)
    self.smooth_servo_move('rear_left_lift', 90, steps=3)
    time.sleep(delay * 2)


def _execute_left_turn_gait(self):
    """Sola dönüş gaiti"""
    settings = self.get_power_settings()
    lift_angle = settings['lift_range']
    turn_angle = settings['axis_range']
    delay = settings['speed_delay']

    print("[GAIT] Left turn gait...")

    # PHASE 1: Lift all legs
    for leg in ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']:
        self.smooth_servo_move(leg, 90 - lift_angle, steps=3)
    time.sleep(delay * 2)

    # PHASE 2: Turn all axis servos left
    for leg in ['front_left_axis', 'front_right_axis', 'rear_left_axis', 'rear_right_axis']:
        self.smooth_servo_move(leg, 90 - turn_angle, steps=4)
    time.sleep(delay * 3)

    # PHASE 3: Lower all legs
    for leg in ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']:
        self.smooth_servo_move(leg, 90, steps=3)
    time.sleep(delay * 2)


def _execute_right_turn_gait(self):
    """Sağa dönüş gaiti"""
    settings = self.get_power_settings()
    lift_angle = settings['lift_range']
    turn_angle = settings['axis_range']
    delay = settings['speed_delay']

    print("[GAIT] Right turn gait...")

    # PHASE 1: Lift all legs
    for leg in ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']:
        self.smooth_servo_move(leg, 90 - lift_angle, steps=3)
    time.sleep(delay * 2)

    # PHASE 2: Turn all axis servos right
    for leg in ['front_left_axis', 'front_right_axis', 'rear_left_axis', 'rear_right_axis']:
        self.smooth_servo_move(leg, 90 + turn_angle, steps=4)
    time.sleep(delay * 3)

    # PHASE 3: Lower all legs
    for leg in ['front_left_lift', 'front_right_lift', 'rear_left_lift', 'rear_right_lift']:
        self.smooth_servo_move(leg, 90, steps=3)
    time.sleep(delay * 2)


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


# SATIR 1000 civarındaki ana yürüyüş fonksiyonlarını değiştir:

# SATIR 1350 civarında şu fonksiyonları BUL ve DEĞİŞTİR:

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
