import time         # Delay ve yürüme adımları arasında bekleme için
import threading    # with self.walking_lock: kullanımı için gereklidir

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
                self.set_servo_angle('rear_left_axis', 90 - settings['axis_range'] // 2)
                self.set_servo_angle('rear_right_axis', 90 + settings['axis_range'] // 2)
            else:
                self.set_servo_angle('rear_left_axis', 90 + settings['axis_range'] // 2)
                self.set_servo_angle('rear_right_axis', 90 - settings['axis_range'] // 2)

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