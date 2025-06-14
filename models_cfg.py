# models_cfg.py – Tek noktadan DNN model yolları
# ------------------------------------------------
# • Face detector : Intel-Res10 SSD (Caffe format)
# • Person        : OpenPifPaf (otomatik, PyPI üzerinden gelir)

from pathlib import Path

# ───────────── Model kök klasörü ───────────────
root = Path(__file__).parent / "models"

# ───────────── Yüz tanıma modeli ───────────────
FACE_PROTO = root / "deploy.prototxt"
FACE_MODEL = root / "res10.caffemodel"

# Dosya kontrolü (eksikse açık hata ver)
if not FACE_PROTO.exists():
    raise FileNotFoundError(f"[models_cfg] Eksik: {FACE_PROTO}")
if not FACE_MODEL.exists():
    raise FileNotFoundError(f"[models_cfg] Eksik: {FACE_MODEL}")

# ───────────── Kişi / POSE (YOLO veya OpenPifPaf) ───────────────
PERSON_CFG     = root / "yolov4-tiny.cfg"
PERSON_WEIGHTS = root / "yolov4-tiny.weights"
PERSON_CLASS_ID = 0  # COCO için: 'person' class ID'si 0
