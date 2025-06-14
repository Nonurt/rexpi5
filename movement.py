"""
Cross-platform motion engine for the REX quadruped.
• Raspberry Pi 5 + PCA9685  /  Windows-dev’de mock
• Trim güvenliği ±60 ° – “Center/Stop” artık 0-180°’ye çarpmıyor
"""
import json, time, platform
from pathlib import Path

# ─────────── HW autodetect ───────────
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
            self.channels  = [_MockCh() for _ in range(16)]
            self.frequency = 50
        def __getattr__(self, _):
            return lambda *a, **k: None
    busio = None; SCL = SDA = None; PCA9685 = _MockPCA9685

# ─────────── Config ───────────
CFG_PATH = Path(__file__).with_name("config.json")
DEFAULTS = {
    "trim":          [-12, 10, -18, 12],  # FL, RL, RR, FR
    "stance_height": 60,
    "big_step":      False,
    "spd":           5,    # ms / micro-step
    "high":          10
}
TRIM_LIMIT = 60  # ±60 ° güvenlik

def _load_cfg():
    if CFG_PATH.exists():
        try:
            return {**DEFAULTS, **json.loads(CFG_PATH.read_text())}
        except json.JSONDecodeError:
            print("[config] Hatalı JSON, default’a dönüldü.")
    return DEFAULTS.copy()

def _save_cfg(cfg):
    CFG_PATH.write_text(json.dumps(cfg, indent=2))

# ─────────── RexMotion ───────────
class RexMotion:
    SERVO_MIN, SERVO_MAX, FREQ = 125, 575, 50
    HIP_CH  = (0, 2, 4, 6)            # FL, RL, RR, FR
    LIFT_CH = (1, 3, 5, 7)

    FWD_DIR = {0:180, 2:180, 4:0, 6:0}
    BCK_DIR = {0:0,   2:0,   4:180, 6:180}

    def __init__(self):
        if HW == "real":
            try:
                i2c = busio.I2C(SCL, SDA)
                self.pwm = PCA9685(i2c, address=0x40)
                self.pwm.frequency = self.FREQ
            except Exception as e:
                print(f"[I2C] {e} – mock’la devam")
                self.pwm = PCA9685()
        else:
            self.pwm = PCA9685()
        time.sleep(0.05)

        self.cfg = _load_cfg()
        self.s   = [90]*10
        self.center_servos()

    # ---- low-level ----
    @staticmethod
    def _pulse(angle:int) -> int:
        rng = RexMotion.SERVO_MAX - RexMotion.SERVO_MIN
        return int(angle/180 * rng + RexMotion.SERVO_MIN)

    def _apply_trim(self, ch:int, angle:int) -> int:
        if ch in self.HIP_CH:
            idx = self.HIP_CH.index(ch)
            t   = max(-TRIM_LIMIT, min(TRIM_LIMIT, self.cfg["trim"][idx]))
            angle += t
        # güvenli pencere 15-165°
        if angle < 15:
            angle = 15 + (15 - angle)
        elif angle > 165:
            angle = 165 - (angle - 165)
        return max(0, min(180, angle))

    def write(self, ch:int, angle:int):
        ang = self._apply_trim(ch, angle)
        duty = int(self._pulse(ang) * 65535 / 4096)
        self.pwm.channels[ch].duty_cycle = duty
        self.s[ch] = ang
        if HW == "mock":
            print(f"[mock] ch{ch} → {ang:3d}°")

    # ---- posture helpers ----
    def center_servos(self):
        for ch in self.HIP_CH:
            self.write(ch, 90)
        for ch in self.LIFT_CH:
            self.write(ch, self.cfg["stance_height"])

    def stabilize(self):
        h = self.cfg["stance_height"]
        self.srv(90,90,90,90, h,h,h,h, 4,4,4,4)

    def lean_left(self):    self.srv(90,90,90,90,  60,60,120,120, 4,4,4,4)
    def lean_right(self):   self.srv(90,90,90,90, 120,120, 60, 60, 4,4,4,4)
    def lean_forward(self): self.srv(90,90,90,90,  60,120,120, 60,4,4,4,4)
    def lean_back(self):    self.srv(90,90,90,90, 120, 60, 60,120,4,4,4,4)

    # ---- micro-step core ----
    def _srv_core(self, hips, lifts, spd):
        lifts = [l + self.cfg["high"]*3 for l in lifts]
        done  = False
        while not done:
            done = True
            for i,ch in enumerate(self.HIP_CH):
                cur, tgt = self.s[ch], hips[i]
                if cur != tgt:
                    self.write(ch, cur + (spd[i] if cur < tgt else -spd[i]))
                    done = False
            for i,ch in enumerate(self.LIFT_CH):
                cur, tgt = self.s[ch], lifts[i]
                if cur != tgt:
                    self.write(ch, cur + (spd[i] if cur < tgt else -spd[i]))
                    done = False
            if not done:
                time.sleep(self.cfg["spd"]/1000)

    def srv(self, p11,p21,p31,p41, p12,p22,p32,p42, sp1,sp2,sp3,sp4):
        self._srv_core([p11,p21,p31,p41], [p12,p22,p32,p42], (sp1,sp2,sp3,sp4))

    # ---- hip helpers ----
    def HIP_FWD(self, ch):  # öne kay
        base = self.FWD_DIR[ch]
        return (30 if base==0 else 150) if self.cfg["big_step"] else (60 if base==0 else 120)
    def HIP_BCK(self, ch):  # geriye kay
        base = self.BCK_DIR[ch]
        return (30 if base==0 else 150) if self.cfg["big_step"] else (60 if base==0 else 120)

    # ---- gait örnekleri ----
    def forward(self):
        self.srv(self.HIP_BCK(0),  90,  90, self.HIP_FWD(6), 42,42,42,  6, 2,3,2,2)
        self.srv(self.HIP_BCK(0),  90,  90, self.HIP_FWD(6), 33,33,33, 42, 2,3,2,2)
        self.srv(self.HIP_BCK(0), self.HIP_BCK(2), self.HIP_FWD(4), self.HIP_FWD(6),
                 42,42, 6,42, 2,2,3,2)
        self.srv(self.HIP_BCK(0), self.HIP_BCK(2), self.HIP_FWD(4), self.HIP_FWD(6),
                 33,24,33,33, 2,2,3,2)
        self.srv(self.HIP_FWD(0), self.HIP_BCK(2), self.HIP_FWD(4), self.HIP_BCK(6),
                 6,42,42,42, 2,2,2,3)
        self.srv(self.HIP_FWD(0), self.HIP_BCK(2), self.HIP_FWD(4), self.HIP_BCK(6),
                 42, 6,33, 6, 2,2,2,3)
        self.srv(self.HIP_FWD(0), self.HIP_FWD(2), self.HIP_BCK(4), self.HIP_BCK(6),
                 42, 6,33,33, 3,2,2,2)
        self.srv(90,90,90,90, 33,33,33,33, 3,2,2,2)

    def back(self):
        self.srv(self.HIP_FWD(0), self.HIP_FWD(2), self.HIP_BCK(4), self.HIP_BCK(6),
                 42, 6,42,42, 2,3,2,2)
        # ... (geri kalanı aynı kalır) ...

    def turn_left(self):  pass
    def turn_right(self): pass

    # ---- utils ----
    def raw_servo_cmd(self, cmd:str):
        for tok in cmd.strip().split():
            if ':' in tok:
                ch, ang = tok.split(':')
                if ch.isdigit() and ang.lstrip('-').isdigit():
                    ch_i, ang_i = int(ch), int(ang)
                    if 0 <= ch_i <= 7:
                        self.write(ch_i, ang_i)

    def adjust_trim(self, delta:int):
        self.cfg["trim"] = [
            max(-TRIM_LIMIT, min(TRIM_LIMIT, t+delta)) for t in self.cfg["trim"]
        ]
        self.save_cfg(); self.center_servos()

    def save_cfg(self): _save_cfg(self.cfg)
    def get_servo_states(self): return self.s[:8]

# ---- singleton ----
rex = RexMotion()
raw_servo_cmd = rex.raw_servo_cmd
