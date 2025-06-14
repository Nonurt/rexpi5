"""
Cross-platform motion engine for the REX quadruped.
• Raspberry Pi 5 + PCA9685   /   Windows-dev’de mock
• Trim güvenliği ±60 ° – “Center/Stop” komutları 15-165 ° aralığına clip’lenir
"""

import json, time, platform
from pathlib import Path

# ───────────── HW autodetect ─────────────
def is_raspberry_pi() -> bool:
    return platform.system() == "Linux" and (
        "arm" in platform.machine().lower() or "aarch64" in platform.machine().lower()
    )

IS_PI = is_raspberry_pi()
try:
    if IS_PI:
        import busio
        from board import SCL, SDA
        from adafruit_pca9685 import PCA9685
        HW = "real"
        print("[movement] Gerçek PCA9685 aktif.")
    else:
        raise ImportError
except (ImportError, RuntimeError, NotImplementedError):
    HW = "mock"
    print("[movement] Mock PCA9685 modu.")

    class _MockCh:
        duty_cycle = 0
        def __setattr__(self, *_): pass

    class _MockPCA9685:
        def __init__(self, *_, **__):
            self.channels = [_MockCh() for _ in range(16)]
            self.frequency = 50
        def __getattr__(self, _):
            return lambda *a, **k: None

    busio = None
    PCA9685 = _MockPCA9685
    SCL = SDA = None


# ───────────── Config ─────────────
CFG_PATH = Path(__file__).with_name("config.json")
DEFAULTS = {
    "trim":          [-12, 10, -18, 12],
    "stance_height": 60,
    "big_step":      False,
    "spd":           8,
    "high":          10
}
TRIM_LIMIT = 60

def _load_cfg():
    if CFG_PATH.exists():
        try:
            return {**DEFAULTS, **json.loads(CFG_PATH.read_text())}
        except json.JSONDecodeError:
            print("[config] Geçersiz JSON – varsayılana dönüldü.")
    return DEFAULTS.copy()

def _save_cfg(cfg):
    CFG_PATH.write_text(json.dumps(cfg, indent=2))


# ───────────── RexMotion ─────────────
class RexMotion:
    SERVO_MIN, SERVO_MAX, FREQ = 125, 575, 50
    HIP_CH  = (0, 2, 4, 6)
    LIFT_CH = (1, 3, 5, 7)

    FWD_DIR = {0: 180, 2: 180, 4: 0, 6: 0}
    BCK_DIR = {0: 0,   2: 0,   4: 180, 6: 180}

    def __init__(self):
        if HW == "real":
            try:
                i2c = busio.I2C(SCL, SDA)
                self.pwm = PCA9685(i2c, address=0x40)
                self.pwm.frequency = self.FREQ
            except Exception as e:
                print(f"[I2C] {e} – mock’a düştü")
                self.pwm = PCA9685()
        else:
            self.pwm = PCA9685()
        time.sleep(0.05)

        self.cfg = _load_cfg()
        self.s   = [None] * 16
        self.walking = False
        self.center_servos()

    @staticmethod
    def _pulse(angle: int) -> int:
        rng = RexMotion.SERVO_MAX - RexMotion.SERVO_MIN
        return int(angle / 180 * rng + RexMotion.SERVO_MIN)

    def _apply_trim(self, ch: int, angle: int) -> int:
        if ch in self.HIP_CH:
            idx = self.HIP_CH.index(ch)
            t   = max(-TRIM_LIMIT, min(TRIM_LIMIT, self.cfg["trim"][idx]))
            angle += t
        angle = max(15, min(165, angle))
        return angle

    def write(self, ch: int, angle: int):
        ang = self._apply_trim(ch, angle)
        if self.s[ch] == ang:
            return
        duty = int(self._pulse(ang) * 65535 / 4096)
        self.pwm.channels[ch].duty_cycle = duty
        self.s[ch] = ang
        if HW == "mock":
            print(f"[mock] ch{ch} → {ang:3d}°")

    def center_servos(self):
        # Mutlak açı gönderimiyle
        self.raw_servo_cmd("0:90 2:90 4:90 6:90 1:{} 3:{} 5:{} 7:{}".format(
            self.cfg["stance_height"],
            self.cfg["stance_height"],
            self.cfg["stance_height"],
            self.cfg["stance_height"],
        ))

    def stabilize(self):
        self.raw_servo_cmd("0:90 2:90 4:90 6:90")

    def lean_left(self):
        self.raw_servo_cmd("0:90 2:90 4:90 6:90 1:60 3:60 5:120 7:120")

    def lean_right(self):
        self.raw_servo_cmd("0:90 2:90 4:90 6:90 1:120 3:120 5:60 7:60")

    def lean_forward(self):
        self.raw_servo_cmd("0:90 2:90 4:90 6:90 1:60 3:120 5:120 7:60")

    def lean_back(self):
        self.raw_servo_cmd("0:90 2:90 4:90 6:90 1:120 3:60 5:60 7:120")

    def _busy(self):
        if self.walking:
            print("[GAIT] already running")
            return True
        self.walking = True
        return False

    def forward(self):
        if self._busy():
            return
        try:
            # Her hareket preset mutlak açı ile başlar
            self.center_servos()
            time.sleep(0.25)
            # Hareket adımları (örnek, ilerleyerek açılar)
            self.raw_servo_cmd("0:30 2:90 4:90 6:150 1:42 3:42 5:42 7:6")
            time.sleep(0.3)
            self.raw_servo_cmd("0:30 2:90 4:90 6:150 1:33 3:33 5:33 7:42")
            time.sleep(0.3)
            self.raw_servo_cmd("0:30 2:30 4:150 6:150 1:42 3:42 5:6 7:42")
            time.sleep(0.3)
            self.raw_servo_cmd("0:30 2:30 4:150 6:150 1:33 3:24 5:33 7:33")
            time.sleep(0.3)
            self.raw_servo_cmd("0:150 2:30 4:150 6:30 1:6 3:42 5:42 7:42")
            time.sleep(0.3)
            self.raw_servo_cmd("0:150 2:30 4:150 6:30 1:42 3:6 5:33 7:6")
            time.sleep(0.3)
            self.raw_servo_cmd("0:150 2:150 4:30 6:30 1:42 3:6 5:33 7:33")
            time.sleep(0.3)
            self.center_servos()
        except Exception as e:
            print("[GAIT-ERR]", e)
        finally:
            self.walking = False

    def back(self):
        if self._busy():
            return
        try:
            self.center_servos()
            time.sleep(0.25)
            self.raw_servo_cmd("0:150 2:150 4:30 6:30 1:42 3:6 5:42 7:42")
            time.sleep(0.3)
            self.center_servos()
        except Exception as e:
            print("[GAIT-ERR]", e)
        finally:
            self.walking = False

    def turn_left(self):
        # İstersen benzer mutlak servo komutları tanımlayabilirim
        pass

    def turn_right(self):
        # İstersen benzer mutlak servo komutları tanımlayabilirim
        pass

    def raw_servo_cmd(self, cmd: str):
        # Slider hareketi gibi mutlak servo açı komutlarını işler
        for tok in cmd.strip().split():
            if ':' in tok:
                ch, ang = tok.split(':')
                if ch.isdigit() and ang.lstrip('-').isdigit():
                    ch_i, ang_i = int(ch), int(ang)
                    if 0 <= ch_i <= 7:
                        self.write(ch_i, ang_i)

    def adjust_trim(self, delta: int):
        self.cfg["trim"] = [
            max(-TRIM_LIMIT, min(TRIM_LIMIT, t + delta)) for t in self.cfg["trim"]
        ]
        self.save_cfg()
        self.center_servos()

    def save_cfg(self): _save_cfg(self.cfg)
    def get_servo_states(self): return self.s[:8]


rex = RexMotion()
raw_servo_cmd = rex.raw_servo_cmd
