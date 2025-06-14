 import time             # Adım aralarında gecikme sağlamak için
# === REX QUAD STYLE WALKING ALGORITHM ===

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
                    return 30 if bigStep else 60  # FR/RR için normal

            def HIP_BCK(axis_servo):
                if 'left' in axis_servo:
                    return 30 if bigStep else 60  # FL/RL için ters yön
                else:
                    return 150 if bigStep else 120  # FR/RR için normal

            # PHASE 1: FR ileri kayar, FL geriye iter
            print("[REX] Phase 1: FR forward, FL backward push")
            servo_positions = {
                'front_left_axis': HIP_BCK('front_left_axis'),
                'front_right_axis': HIP_FWD('front_right_axis'),
                'rear_left_axis': 90,
                'rear_right_axis': 90,
                'front_left_lift': 42 + high * 3,
                'front_right_lift': 6 + high * 3,  # FR kaldırılır
                'rear_left_lift': 42 + high * 3,
                'rear_right_lift': 42 + high * 3
            }
            steps_per_servo = {'front_left_axis': 2, 'front_right_axis': 3, 'rear_left_axis': 2, 'rear_right_axis': 2,
                               'front_left_lift': 2, 'front_right_lift': 3, 'rear_left_lift': 2, 'rear_right_lift': 2}
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 2: Kilit
            print("[REX] Phase 2: Lock position")
            servo_positions.update({
                'front_left_lift': 33 + high * 3,
                'front_right_lift': 42 + high * 3,
                'rear_left_lift': 33 + high * 3,
                'rear_right_lift': 42 + high * 3
            })
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 3: RR ileri kayar, RL geriye iter
            print("[REX] Phase 3: RR forward, RL backward push")
            servo_positions.update({
                'rear_left_axis': HIP_BCK('rear_left_axis'),
                'rear_right_axis': HIP_FWD('rear_right_axis'),
                'rear_left_lift': 42 + high * 3,
                'rear_right_lift': 6 + high * 3  # RR kaldırılır
            })
            steps_per_servo.update({'rear_left_axis': 2, 'rear_right_axis': 3})
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 4: Kilit
            print("[REX] Phase 4: Lock position")
            servo_positions.update({
                'rear_left_lift': 24 + high * 3,
                'rear_right_lift': 33 + high * 3
            })
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 5: FL ileri kayar, FR geriye iter
            print("[REX] Phase 5: FL forward, FR backward push")
            servo_positions.update({
                'front_left_axis': HIP_FWD('front_left_axis'),
                'front_right_axis': HIP_BCK('front_right_axis'),
                'front_left_lift': 6 + high * 3,  # FL kaldırılır
                'front_right_lift': 42 + high * 3
            })
            steps_per_servo.update({'front_left_axis': 2, 'front_right_axis': 2})
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 6: Kilit
            print("[REX] Phase 6: Lock position")
            servo_positions.update({
                'front_left_lift': 42 + high * 3,
                'front_right_lift': 6 + high * 3
            })
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 7: RL ileri kayar, RR geriye iter
            print("[REX] Phase 7: RL forward, RR backward push")
            servo_positions.update({
                'rear_left_axis': HIP_FWD('rear_left_axis'),
                'rear_right_axis': HIP_BCK('rear_right_axis'),
                'rear_left_lift': 6 + high * 3,  # RL kaldırılır
                'rear_right_lift': 33 + high * 3
            })
            steps_per_servo.update({'rear_left_axis': 3, 'rear_right_axis': 2})
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 8: Nötr pozisyon
            print("[REX] Phase 8: Return to neutral")
            servo_positions = {
                'front_left_axis': 90,
                'front_right_axis': 90,
                'rear_left_axis': 90,
                'rear_right_axis': 90,
                'front_left_lift': 33 + high * 3,
                'front_right_lift': 33 + high * 3,
                'rear_left_lift': 33 + high * 3,
                'rear_right_lift': 33 + high * 3
            }
            steps_per_servo = {servo: 3 for servo in servo_positions.keys()}
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

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
                'front_left_lift': 42 + high * 3,
                'front_right_lift': 6 + high * 3,  # FR kaldırılır
                'rear_left_lift': 42 + high * 3,
                'rear_right_lift': 42 + high * 3
            }
            steps_per_servo = {'front_left_axis': 2, 'front_right_axis': 3, 'rear_left_axis': 2, 'rear_right_axis': 2,
                               'front_left_lift': 2, 'front_right_lift': 3, 'rear_left_lift': 2, 'rear_right_lift': 2}
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 2: Kilit
            print("[REX] Phase 2: Lock position")
            servo_positions.update({
                'front_left_lift': 33 + high * 3,
                'front_right_lift': 42 + high * 3,
                'rear_left_lift': 33 + high * 3,
                'rear_right_lift': 42 + high * 3
            })
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 3: RR ileri kayar, RL geriye iter
            print("[REX] Phase 3: RR forward, RL backward push")
            servo_positions.update({
                'rear_left_axis': HIP_BCK('rear_left_axis'),
                'rear_right_axis': HIP_FWD('rear_right_axis'),
                'rear_left_lift': 42 + high * 3,
                'rear_right_lift': 6 + high * 3  # RR kaldırılır
            })
            steps_per_servo.update({'rear_left_axis': 2, 'rear_right_axis': 3})
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 4: Kilit
            print("[REX] Phase 4: Lock position")
            servo_positions.update({
                'rear_left_lift': 24 + high * 3,
                'rear_right_lift': 33 + high * 3
            })
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 5: FL ileri kayar, FR geriye iter
            print("[REX] Phase 5: FL forward, FR backward push")
            servo_positions.update({
                'front_left_axis': HIP_FWD('front_left_axis'),
                'front_right_axis': HIP_BCK('front_right_axis'),
                'front_left_lift': 6 + high * 3,  # FL kaldırılır
                'front_right_lift': 42 + high * 3
            })
            steps_per_servo.update({'front_left_axis': 2, 'front_right_axis': 2})
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 6: Kilit
            print("[REX] Phase 6: Lock position")
            servo_positions.update({
                'front_left_lift': 42 + high * 3,
                'front_right_lift': 6 + high * 3
            })
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 7: RL ileri kayar, RR geriye iter
            print("[REX] Phase 7: RL forward, RR backward push")
            servo_positions.update({
                'rear_left_axis': HIP_FWD('rear_left_axis'),
                'rear_right_axis': HIP_BCK('rear_right_axis'),
                'rear_left_lift': 6 + high * 3,  # RL kaldırılır
                'rear_right_lift': 33 + high * 3
            })
            steps_per_servo.update({'rear_left_axis': 3, 'rear_right_axis': 2})
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            # PHASE 8: Nötr pozisyon
            print("[REX] Phase 8: Return to neutral")
            servo_positions = {
                'front_left_axis': 90,
                'front_right_axis': 90,
                'rear_left_axis': 90,
                'rear_right_axis': 90,
                'front_left_lift': 33 + high * 3,
                'front_right_lift': 33 + high * 3,
                'rear_left_lift': 33 + high * 3,
                'rear_right_lift': 33 + high * 3
            }
            steps_per_servo = {servo: 3 for servo in servo_positions.keys()}
            self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

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
                (120, 90, 90, 60, 42 + high * 3, 6 + high * 3, 33 + high * 3, 42 + high * 3, [2, 3, 2, 2]),
                (120, 90, 90, 60, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3, [2, 3, 2, 2]),
                (100, 70, 110, 80, 42 + high * 3, 33 + high * 3, 6 + high * 3, 42 + high * 3, [2, 2, 3, 2]),
                (100, 70, 110, 80, 42 + high * 3, 33 + high * 3, 33 + high * 3, 24 + high * 3, [2, 2, 3, 2]),
                (80, 50, 130, 100, 42 + high * 3, 33 + high * 3, 33 + high * 3, 6 + high * 3, [2, 2, 2, 3]),
                (80, 50, 130, 100, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3, [2, 2, 2, 3]),
                (120, 90, 90, 60, 6 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3, [3, 2, 2, 2]),
                (120, 90, 90, 60, 42 + high * 3, 33 + high * 3, 33 + high * 3, 33 + high * 3, [3, 2, 2, 2])
            ]

            for i, (fl_axis, rl_axis, rr_axis, fr_axis, fl_lift, rl_lift, rr_lift, fr_lift, steps) in enumerate(phases):
                print(f"[REX] Turn left phase {i + 1}/8")

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

                self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

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
                (60, 50, 130, 100, 6 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3, [3, 2, 2, 2]),
                (60, 50, 130, 100, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3, [3, 2, 2, 2]),
                (80, 70, 110, 80, 42 + high * 3, 33 + high * 3, 33 + high * 3, 6 + high * 3, [2, 2, 2, 3]),
                (80, 70, 110, 80, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3, [2, 2, 2, 3]),
                (100, 90, 90, 60, 42 + high * 3, 33 + high * 3, 6 + high * 3, 42 + high * 3, [2, 2, 3, 2]),
                (100, 90, 90, 60, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3, [2, 2, 3, 2]),
                (120, 90, 90, 60, 42 + high * 3, 6 + high * 3, 33 + high * 3, 42 + high * 3, [2, 3, 2, 2]),
                (120, 90, 90, 60, 42 + high * 3, 33 + high * 3, 33 + high * 3, 42 + high * 3, [2, 3, 2, 2])
            ]

            for i, (fl_axis, rl_axis, rr_axis, fr_axis, fl_lift, rl_lift, rr_lift, fr_lift, steps) in enumerate(phases):
                print(f"[REX] Turn right phase {i + 1}/8")

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

                self.rex_servo_move(servo_positions, steps_per_servo, spd / 1000)

            print("[REX] Right turn completed")

        except Exception as e:
            print(f"[ERROR] REX right turn error: {e}")
        finally:
            self.walking = False


# === REX LEAN FUNCTIONS ===

def rex_lean_left(self):
    """REX kodundaki lean_left() fonksiyonu"""
    servo_positions = {
        'front_left_lift': 60 + 30,  # s12=60
        'rear_left_lift': 60 + 30,  # s22=60
        'rear_right_lift': 120 + 30,  # s32=120
        'front_right_lift': 120 + 30  # s42=120
    }
    print("[REX] Leaning left...")
    self.rex_servo_move(servo_positions, {servo: 3 for servo in servo_positions.keys()}, 0.05)


def rex_lean_right(self):
    """REX kodundaki lean_right() fonksiyonu"""
    servo_positions = {
        'front_left_lift': 120 + 30,  # s12=120
        'rear_left_lift': 120 + 30,  # s22=120
        'rear_right_lift': 60 + 30,  # s32=60
        'front_right_lift': 60 + 30  # s42=60
    }
    print("[REX] Leaning right...")
    self.rex_servo_move(servo_positions, {servo: 3 for servo in servo_positions.keys()}, 0.05)


def rex_lean_forward(self):
    """REX kodundaki lean_forward() fonksiyonu"""
    servo_positions = {
        'front_left_lift': 60 + 30,  # s12=60
        'rear_left_lift': 120 + 30,  # s22=120
        'rear_right_lift': 120 + 30,  # s32=120
        'front_right_lift': 60 + 30  # s42=60
    }
    print("[REX] Leaning forward...")
    self.rex_servo_move(servo_positions, {servo: 3 for servo in servo_positions.keys()}, 0.05)


def rex_lean_back(self):
    """REX kodundaki lean_back() fonksiyonu"""
    servo_positions = {
        'front_left_lift': 120 + 30,  # s12=120
        'rear_left_lift': 60 + 30,  # s22=60
        'rear_right_lift': 60 + 30,  # s32=60
        'front_right_lift': 120 + 30  # s42=120
    }
    print("[REX] Leaning back...")
    self.rex_servo_move(servo_positions, {servo: 3 for servo in servo_positions.keys()}, 0.05)
