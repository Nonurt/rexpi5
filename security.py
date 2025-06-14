# security.py – basit yüz kaydetme & ID yöneticisi
# ------------------------------------------------
# • `assign_ids(frame, boxes)`   → [(id,(x1,y1,x2,y2,cx,cy)), ...]
# • Numaraları _kalıcı_ tutmaz; bir oturumda sabit kalması yeterli.

import cv2, itertools, numpy as np
from pathlib import Path
from datetime import datetime

SAVE_FACE_IMAGES = False

# — Yüzlerin kayıt dizini (otomatik oluşturulur)
face_dir = Path(__file__).parent / "faces"
face_dir.mkdir(exist_ok=True)

_next_id = itertools.count().__next__
_tracks  = {}  # id -> (x1,y1,x2,y2)
TARGET_ID = None

# — IoU hesaplayıcı ——————————————————————
def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1, inter_y1 = max(ax1, bx1), max(ay1, by1)
    inter_x2, inter_y2 = min(ax2, bx2), min(ay2, by2)
    if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
        return .0
    inter = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / union

# — ID atama ve eşleştirme fonksiyonu —————————————
def assign_ids(frame, boxes):
    out = []
    for (x1, y1, x2, y2) in boxes:
        best = None
        for tid, tbox in _tracks.items():
            if _iou(tbox, (x1, y1, x2, y2)) > 0.4:
                best = tid
                break
        if best is None:
            best = _next_id()
        _tracks[best] = (x1, y1, x2, y2)
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        out.append((best, (x1, y1, x2, y2, cx, cy)))

        # Yüz kaydı
        if SAVE_FACE_IMAGES:
            f = frame[max(y1, 0):y2, max(x1, 0):x2]
            if f.size:
                ts = datetime.now().strftime("%Y%m%d-%H%M%S")
                path = face_dir / f"{best}_{ts}.jpg"
                cv2.imwrite(str(path), f)

    # silinen yüzleri temizle
    live = {tid for tid, _ in out}
    for tid in list(_tracks.keys()):
        if tid not in live:
            _tracks.pop(tid)
    return out

# — Hedef ID'leri Flask üzerinden sunma ————————————
def get_targets():
    return {
        "ids": list(_tracks.keys()),
        "current": TARGET_ID if TARGET_ID in _tracks else None
    }

# — Hedef kişiyi değiştir ——————————————————
def set_target(tid):
    global TARGET_ID
    TARGET_ID = tid
