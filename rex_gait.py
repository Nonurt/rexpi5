import time
import threading
from servo_controller import ServoController


class RexGait(ServoController):
    def __init__(self):
        super().__init__()

    def rex_stabilize(self):
        """REX stance position"""
        stance_height = 60  # Default stance height

        servo_positions = {
            'front_left_axis': 90,
            'front_right_axis': 90,
            'rear_left_axis': 90,
            'rear_right_axis': 90,
            'front_left_lift': stance_height + 30,
            'front_right_lift': stance_height + 30,
            'rear_left_lift': stance_height + 30,
            'rear_right_lift': stance_height + 30
        }

        steps_per_servo = {servo: 4 for servo in servo_positions.keys()}

        print("[REX] Stabilizing to stance position...")
        self.servo_move(servo_positions, steps_per_servo, 0.005)

    def rex_forward_gait(self):
        """REX forward walking gait"""
        if self.walking:
            return

        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()

            print("[REX] REX Forward gait - 8 phase ESP32 style")

            try:
                # REX variables
                spd = int(settings['speed_delay'] * 1000)  # in ms
                high = settings['lift_range'] // 3
                bigStep = settings['axis_range'] > 40

                # REX hip macros
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

                # PHASE 1: FR forward, FL backward push
                print("[REX] Phase 1: FR forward, FL backward push")
                servo_positions = {
                    'front_left_axis': HIP_BCK('front_left_axis'),
                    'front_right_axis': HIP_FWD('front_right_axis'),
                    'rear_left_axis': 90,
                    'rear_right_axis': 90,
                    'front_left_lift': 42 + high * 3,
                    'front_right_lift': 6 + high * 3,
                    'rear_left_lift': 42 + high * 3,
                    'rear_right_lift': 42 + high * 3
                }
                steps_per_servo = {'front_left_axis': 2, 'front_right_axis': 3, 'rear_left_axis': 2,
                                   'rear_right_axis': 2,
                                   'front_left_lift': 2, 'front_right_lift': 3, 'rear_left_lift': 2,
                                   'rear_right_lift': 2}
                self.servo_move(servo_positions, steps_per_servo, spd / 1000)

                # PHASE 2: Lock position
                print("[REX] Phase 2: Lock position")
                servo_positions.update({
                    'front_left_lift': 33 + high * 3,
                    'front_right_lift': 42 + high * 3,
                    'rear_left_lift': 33 + high * 3,
                    'rear_right_lift': 42 + high * 3
                })
                self.servo_move(servo_positions, steps_per_servo, spd / 1000)

                # PHASE 3: RR forward, RL backward push
                print("[REX] Phase 3: RR forward, RL backward push")
                servo_positions.update({
                    'rear_left_axis': HIP_BCK('rear_left_axis'),
                    'rear_right_axis': HIP_FWD('rear_right_axis'),
                    'rear_left_lift': 42 + high * 3,
                    'rear_right_lift': 6 + high * 3
                })
                steps_per_servo.update({'rear_left_axis': 2, 'rear_right_axis': 3})
                self.servo_move(servo_positions, steps_per_servo, spd / 1000)

                # PHASE 4: Lock position
                print("[REX] Phase 4: Lock position")
                servo_positions.update({
                    'rear_left_lift': 24 + high * 3,
                    'rear_right_lift': 33 + high * 3
                })
                self.servo_move(servo_positions, steps_per_servo, spd / 1000)

                # PHASE 5: FL forward, FR backward push
                print("[REX] Phase 5: FL forward, FR backward push")
                servo_positions.update({
                    'front_left_axis': HIP_FWD('front_left_axis'),
                    'front_right_axis': HIP_BCK('front_right_axis'),
                    'front_left_lift': 6 + high * 3,
                    'front_right_lift': 42 + high * 3
                })
                steps_per_servo.update({'front_left_axis': 2, 'front_right_axis': 2})
                self.servo_move(servo_positions, steps_per_servo, spd / 1000)

                # PHASE 6: Lock position
                print("[REX] Phase 6: Lock position")
                servo_positions.update({
                    'front_left_lift': 42 + high * 3,
                    'front_right_lift': 6 + high * 3
                })
                self.servo_move(servo_positions, steps_per_servo, spd / 1000)

                # PHASE 7: RL forward, RR backward push
                print("[REX] Phase 7: RL forward, RR backward push")
                servo_positions.update({
                    'rear_left_axis': HIP_FWD('rear_left_axis'),
                    'rear_right_axis': HIP_BCK('rear_right_axis'),
                    'rear_left_lift': 6 + high * 3,
                    'rear_right_lift': 33 + high * 3
                })
                steps_per_servo.update({'rear_left_axis': 3, 'rear_right_axis': 2})
                self.servo_move(servo_positions, steps_per_servo, spd / 1000)

                # PHASE 8: Return to neutral
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
                self.servo_move(servo_positions, steps_per_servo, spd / 1000)

                print("[REX] Forward gait completed")

            except Exception as e:
                print(f"[ERROR] REX forward gait error: {e}")
            finally:
                self.walking = False

    def rex_turn_left(self):
        """REX left turn movement"""
        if self.walking:
            return

        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()

            print("[REX] Left turn movement")

            try:
                # REX variables
                spd = int(settings['speed_delay'] * 1000)
                high = settings['lift_range'] // 3

                # PHASE 1: Front right and rear left legs forward
                print("[REX] Phase 1: FR and RL forward")
                servo_positions = {
                    'front_left_axis': 120,
                    'front_right_axis': 60,
                    'rear_left_axis': 60,
                    'rear_right_axis': 120,
                    'front_left_lift': 42 + high * 3,
                    'front_right_lift': 6 + high * 3,
                    'rear_left_lift': 6 + high * 3,
                    'rear_right_lift': 42 + high * 3
                }
                self.servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, spd / 1000)

                # PHASE 2: Body rotation
                print("[REX] Phase 2: Body rotation")
                servo_positions.update({
                    'front_left_axis': 60,
                    'front_right_axis': 120,
                    'rear_left_axis': 120,
                    'rear_right_axis': 60,
                    'front_left_lift': 42 + high * 3,
                    'front_right_lift': 42 + high * 3,
                    'rear_left_lift': 42 + high * 3,
                    'rear_right_lift': 42 + high * 3
                })
                self.servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, spd / 1000)

                # Return to neutral
                self.rex_stabilize()
                print("[REX] Left turn completed")

            except Exception as e:
                print(f"[ERROR] REX left turn error: {e}")
            finally:
                self.walking = False

    def rex_turn_right(self):
        """REX right turn movement"""
        if self.walking:
            return

        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()

            print("[REX] Right turn movement")

            try:
                # REX variables
                spd = int(settings['speed_delay'] * 1000)
                high = settings['lift_range'] // 3

                # PHASE 1: Front left and rear right legs forward
                print("[REX] Phase 1: FL and RR forward")
                servo_positions = {
                    'front_left_axis': 60,
                    'front_right_axis': 120,
                    'rear_left_axis': 120,
                    'rear_right_axis': 60,
                    'front_left_lift': 6 + high * 3,
                    'front_right_lift': 42 + high * 3,
                    'rear_left_lift': 42 + high * 3,
                    'rear_right_lift': 6 + high * 3
                }
                self.servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, spd / 1000)

                # PHASE 2: Body rotation
                print("[REX] Phase 2: Body rotation")
                servo_positions.update({
                    'front_left_axis': 120,
                    'front_right_axis': 60,
                    'rear_left_axis': 60,
                    'rear_right_axis': 120,
                    'front_left_lift': 42 + high * 3,
                    'front_right_lift': 42 + high * 3,
                    'rear_left_lift': 42 + high * 3,
                    'rear_right_lift': 42 + high * 3
                })
                self.servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, spd / 1000)

                # Return to neutral
                self.rex_stabilize()
                print("[REX] Right turn completed")

            except Exception as e:
                print(f"[ERROR] REX right turn error: {e}")
            finally:
                self.walking = False

    def rex_backward_gait(self):
        """REX backward walking gait"""
        if self.walking:
            return

        with self.walking_lock:
            self.walking = True
            settings = self.get_power_settings()

            print("[REX] REX Backward gait")

            try:
                # REX variables
                spd = int(settings['speed_delay'] * 1000)
                high = settings['lift_range'] // 3
                bigStep = settings['axis_range'] > 40

                # Reverse of forward gait
                # PHASE 1: FL forward, FR backward
                print("[REX] Phase 1: FL forward, FR backward")
                servo_positions = {
                    'front_left_axis': 60 if bigStep else 90 - 30,
                    'front_right_axis': 120 if bigStep else 90 + 30,
                    'rear_left_axis': 90,
                    'rear_right_axis': 90,
                    'front_left_lift': 6 + high * 3,
                    'front_right_lift': 42 + high * 3,
                    'rear_left_lift': 42 + high * 3,
                    'rear_right_lift': 42 + high * 3
                }
                self.servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, spd / 1000)

                # PHASE 2: RL forward, RR backward
                print("[REX] Phase 2: RL forward, RR backward")
                servo_positions.update({
                    'rear_left_axis': 60 if bigStep else 90 - 30,
                    'rear_right_axis': 120 if bigStep else 90 + 30,
                    'rear_left_lift': 6 + high * 3,
                    'rear_right_lift': 42 + high * 3
                })
                self.servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, spd / 1000)

                # PHASE 3: FR forward, FL backward
                print("[REX] Phase 3: FR forward, FL backward")
                servo_positions.update({
                    'front_left_axis': 120 if bigStep else 90 + 30,
                    'front_right_axis': 60 if bigStep else 90 - 30,
                    'front_left_lift': 42 + high * 3,
                    'front_right_lift': 6 + high * 3
                })
                self.servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, spd / 1000)

                # PHASE 4: RR forward, RL backward
                print("[REX] Phase 4: RR forward, RL backward")
                servo_positions.update({
                    'rear_left_axis': 120 if bigStep else 90 + 30,
                    'rear_right_axis': 60 if bigStep else 90 - 30,
                    'rear_left_lift': 42 + high * 3,
                    'rear_right_lift': 6 + high * 3
                })
                self.servo_move(servo_positions, {s: 3 for s in servo_positions.keys()}, spd / 1000)

                # Return to neutral
                self.rex_stabilize()
                print("[REX] Backward gait completed")

            except Exception as e:
                print(f"[ERROR] REX backward gait error: {e}")
            finally:
                self.walking = False

    # ===== SPIDER MOVEMENT FUNCTIONS (using REX algorithm) =====

    def spider_walk_forward(self):
        """Forward walking using REX algorithm"""
        print("[MAIN] Using REX forward gait instead of spider")
        self.rex_forward_gait()

    def spider_turn_left(self):
        """Left turn using REX algorithm"""
        print("[MAIN] Using REX left turn instead of spider")
        self.rex_turn_left()

    def spider_turn_right(self):
        """Right turn using REX algorithm"""
        print("[MAIN] Using REX right turn instead of spider")
        self.rex_turn_right()

    def spider_back_away(self):
        """Backward movement using REX algorithm"""
        print("[MAIN] Using REX backward gait instead of spider")
        self.rex_backward_gait()

    def spider_stance_position(self):
        """Stance position using REX stabilize"""
        print("[MAIN] Using REX stabilize instead of spider stance")
        self.rex_stabilize()