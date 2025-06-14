# video.py – Kamera, algılama ve servo takibi (Picamera2 sürümü)
# ──────────────────────────────────────────────────────────────
# • Yüz (SSD-ResNet) + İnsan (YOLOv4-tiny) algılama
# • DETECT_MODE = "person" / "face" / "both"
# • Mesafe kestirimi (en büyük insan kutusundan)
# • Otomatik pan-tilt servo takibi (CH 8-9)
# • FPS ölçümü ve gamma/histogram iyileştirme

import time, cv2, numpy as np
from pathlib import Path
from picamera2 import Picamera2
from picamera2.previews import NullPreview  # Sadece NullPreview import edildi, pykms aranmaması için


import models_cfg as cfg
import movement, security

# Kamera başlatma
W, H = 320, 240
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (W, H)}))
picam2.start_preview(NullPreview())  # Preview hatasını çözer
picam2.start()
time.sleep(1)  # Kamera hazır olması için kısa bekleme

print(f"[VIDEO] Picamera2 başlatıldı: {W}×{H}")

# ... devam eden kodlar ...

# DNN modelleri yükleme
face_net = cv2.dnn.readNetFromCaffe(str(cfg.FACE_PROTO), str(cfg.FACE_MODEL))
yolo_net = cv2.dnn.readNetFromDarknet(str(cfg.PERSON_CFG), str(cfg.PERSON_WEIGHTS))
yolo_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
yolo_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Bayraklar / sabitler
TRACK       = False
APPROACH    = False
AUTO_GAMMA  = False
AUTO_HIST   = False
AUTOSTEP    = False
DETECT_MODE = "person"  # default

FOCAL_PX            = 550
PERSON_REAL_WIDTH_M = 0.4

_margin  = 40
inner    = (_margin, _margin, W - _margin, H - _margin)
_stop_a  = 0.22
_last_fwd = 0

# Pan/Tilt ayarları
PAN_CH, TILT_CH = 8, 9
pan = tilt = 90
movement.rex.write(PAN_CH, pan)
movement.rex.write(TILT_CH, tilt)

# Görüntü işleme LUT'ları
_lut = {}
def gamma(img):
    y = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)[:, :, 0].mean()
    g = 0.6 if y < 90 else 1.4 if y > 160 else 1.0
    if g not in _lut:
        _lut[g] = (np.linspace(0, 1, 256) ** g * 255).astype("uint8")
    return cv2.LUT(img, _lut[g])

clahe = cv2.createCLAHE(2.0, (8, 8))
def hist(img):
    l, a, b = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
    return cv2.cvtColor(cv2.merge((clahe.apply(l), a, b)), cv2.COLOR_LAB2BGR)

# FPS hesaplama
_fps_t = [time.time()]

# Ana görüntü üretici fonksiyon
def next_frame():
    global pan, tilt, _last_fwd

    frame = picam2.capture_array()
    if frame is None:
        return None
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # İnsan kutuları (YOLO)
    p_boxes = []
    if DETECT_MODE in ("person", "both"):
        blob = cv2.dnn.blobFromImage(frame, 1/255, (416, 416), swapRB=True, crop=False)
        yolo_net.setInput(blob)
        for o in yolo_net.forward(yolo_net.getUnconnectedOutLayersNames()):
            for *xywh, conf, cls in o:
                if int(cls) == cfg.PERSON_CLASS_ID and conf > 0.4:
                    cx, cy, w, h = (np.array(xywh) * [W, W, W, H]).astype(int)
                    x1, y1 = int(cx - w / 2), int(cy - h / 2)
                    p_boxes.append((x1, y1, x1 + w, y1 + h))

    # Yüz kutuları (SSD Caffe)
    f_boxes = []
    if DETECT_MODE in ("face", "both"):
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104, 177, 123))
        face_net.setInput(blob)
        det = face_net.forward()
        for i in range(det.shape[2]):
            if det[0, 0, i, 2] > 0.5:
                x1, y1, x2, y2 = (det[0, 0, i, 3:7] * [W, H, W, H]).astype(int)
                f_boxes.append((x1, y1, x2, y2))

    # Hedef seçimi
    tracked = security.assign_ids(frame, f_boxes)
    tgt = None
    if security.TARGET_ID is not None:
        for fid, (_, _, _, _, cx, cy) in tracked:
            if fid == security.TARGET_ID:
                for (x1, y1, x2, y2) in p_boxes:
                    if x1 < cx < x2 and y1 < cy < y2:
                        tgt = (x1, y1, x2, y2, cx, cy)
                        break
                break
    if tgt is None and p_boxes:
        x1, y1, x2, y2 = max(p_boxes, key=lambda b: (b[2]-b[0])*(b[3]-b[1]))
        tgt = (x1, y1, x2, y2, (x1 + x2) // 2, (y1 + y2) // 2)

    for (x1, y1, x2, y2) in p_boxes:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 120, 255), 1)
    for fid, (x1, y1, x2, y2, cx, cy) in tracked:
        col = (0, 255, 0) if fid == security.TARGET_ID else (255, 200, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
        cv2.putText(frame, str(fid), (x1 + 2, y1 + 14), 0, 0.45, col, 1, cv2.LINE_AA)

    if tgt:
        x1, y1, x2, y2, cx, cy = tgt
        px_w = x2 - x1
        if px_w > 0:
            dist_m = PERSON_REAL_WIDTH_M * FOCAL_PX / px_w
            txt = f"{dist_m:4.2f} m"
            cv2.putText(frame, txt, (x1, y1 - 6), 0, 0.55, (0, 255, 255), 2, cv2.LINE_AA)

        if TRACK:
            errx, erry = (W // 2 - cx) / W, (H // 2 - cy) / H
            pan = np.clip(pan + errx * 35, 0, 180)
            tilt = np.clip(tilt + erry * 35, 0, 180)
            movement.rex.write(PAN_CH, int(pan))
            movement.rex.write(TILT_CH, int(tilt))
            if APPROACH and (px_w * (y2 - y1)) / (W * H) < _stop_a and time.time() - _last_fwd > 0.4:
                movement.rex.forward()
                _last_fwd = time.time()

    if AUTO_GAMMA:
        frame = gamma(frame)
    if AUTO_HIST:
        frame = hist(frame)

    now = time.time()
    fps = 1.0 / (now - _fps_t[0])
    _fps_t[0] = now
    cv2.putText(frame, f"{fps:.1f} FPS", (5, H - 10), 0, 0.5, (255, 255, 0), 1)

    _, jpg = cv2.imencode(".jpg", frame)
    time.sleep(0.05)
    return jpg.tobytes()

# API Export
get_targets = security.get_targets
set_target  = security.set_target
