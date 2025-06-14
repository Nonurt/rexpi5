# realworld_py.py – REX yolu çizici (hareketleri 2D harita üstünde kaydeder)
# --------------------------------------------------------------
# import realworld_py  # ile etkinleştirilir, patch yapar

from __future__ import annotations
import math, cv2, numpy as np
from pathlib import Path
import movement

# ——— Ayarlar ————————————————————————————
PIXELS_PER_CM = 10
TURN_DEG      = 15
CANVAS_SIZE   = (800, 800)  # (w, h)

# ——— Dosya yolları ———————————————————————
root      = Path(__file__).parent
BASE_IMG  = root / "path_base.png"
DRAW_IMG  = root / "path_draw.png"

# ——— Harita başlat ——————————————————————
if BASE_IMG.exists():
    canvas = cv2.imread(str(BASE_IMG))
    canvas = cv2.resize(canvas, CANVAS_SIZE)
else:
    canvas = 255 * np.ones((*CANVAS_SIZE[::-1], 3), np.uint8)

pos     = np.array(CANVAS_SIZE, dtype=float) / 2
heading = 0.0
cv2.circle(canvas, tuple(pos.astype(int)), 3, (0, 0, 255), -1)

def _save():
    cv2.imwrite(str(DRAW_IMG), canvas)

def _move(cm):
    global pos
    rad = math.radians(heading)
    dxy = np.array([math.sin(rad), -math.cos(rad)]) * cm * PIXELS_PER_CM
    new = pos + dxy
    cv2.line(canvas, tuple(pos.astype(int)), tuple(new.astype(int)), (0, 0, 255), 2)
    pos[:] = new
    _save()

def _turn(deg):
    global heading
    heading = (heading + deg) % 360

# ——— movement.patch ——————————————————————
rex = movement.rex

# Orijinalleri sakla
_o_fwd, _o_back = rex.forward, rex.back
_o_lft, _o_rgt  = rex.turn_left, rex.turn_right

# Patch’li versiyonlar
rex.forward    = (lambda fn=_o_fwd:  lambda *a, **k: (fn(*a, **k), _move(+3))[-1])()
rex.back       = (lambda fn=_o_back: lambda *a, **k: (fn(*a, **k), _move(-3))[-1])()
rex.turn_left  = (lambda fn=_o_lft:  lambda *a, **k: (fn(*a, **k), _turn(-TURN_DEG))[-1])()
rex.turn_right = (lambda fn=_o_rgt:  lambda *a, **k: (fn(*a, **k), _turn(+TURN_DEG))[-1])()

_save()
print("[realworld] path_draw.png güncelleniyor …")
