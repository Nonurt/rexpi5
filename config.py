# config.py

# Servo kanalları (LEGS + CAMERA)
SERVO_CHANNELS = {
    # Bacak servoları
    'front_left_axis': 0, 'front_left_lift': 1,
    'rear_left_axis': 2,  'rear_left_lift': 3,
    'rear_right_axis': 4, 'rear_right_lift': 5,
    'front_right_axis': 6,'front_right_lift': 7,
    # Kamera servoları
    'camera_pan': 8,      # PIN 8: Sol-Sağ (Yatay)
    'camera_tilt': 9      # PIN 9: Yukarı-Aşağı (Dikey)
}

# SG90 Servo pals genişlik aralığı
SERVO_PULSE_WIDTH_RANGE = (600, 2400)
I2C_FREQUENCY = 50

# Kamera Kontrol Ayarları
CAMERA_SETTINGS = {
    'smooth_delay': 0.05,   # Pürüzsüz hareket gecikmesi (ayarlanabilir 0.01-0.2)
    'dead_zone_radius': 60, # Ölü bölge yarıçapı (ayarlanabilir 20-150)
    'step_size': 3,         # Kamera hareket adım boyutu (ayarlanabilir 1-10)
    'pan_min': 20,          # Sol limit
    'pan_max': 160,         # Sağ limit
    'tilt_min': 45,         # Aşağı limit
    'tilt_max': 135          # Yukarı limit
}

# Gelişmiş ROI (Region of Interest) Ayarları
ROI_SETTINGS = {
    'enabled': True,
    'stop_distance': 80,  # cm - Bu mesafede dur (arayüzden ayarlanabilir)
    'min_distance': 50,   # cm - Minimum güvenli mesafe
    'zones': {
        'center': {'x': 220, 'y': 160, 'w': 200, 'h': 160, 'priority': 1},
        'left': {'x': 0, 'y': 120, 'w': 220, 'h': 240, 'priority': 2},
        'right': {'x': 420, 'y': 120, 'w': 220, 'h': 240, 'priority': 2},
        'top': {'x': 160, 'y': 0, 'w': 320, 'h': 160, 'priority': 3},
        'bottom': {'x': 160, 'y': 320, 'w': 320, 'h': 160, 'priority': 3}
    }
}

# Güç Modu Ayarları
POWER_SETTINGS = {
    "low": {"name": "Low Power", "description": "Enerji verimli, yavaş hareket", "lift_range": 30, "axis_range": 30, "speed_delay": 0.08, "step_delay": 0.4, "cooldown": 2.0, "camera_speed": 3, "movement_speed": 0.05},
    "medium": {"name": "Medium Power", "description": "Dengeli performans", "lift_range": 35, "axis_range": 40, "speed_delay": 0.05, "step_delay": 0.25, "cooldown": 1.5, "camera_speed": 4, "movement_speed": 0.03},
    "high": {"name": "High Power", "description": "Hızlı ve güçlü hareket", "lift_range": 45, "axis_range": 50, "speed_delay": 0.03, "step_delay": 0.15, "cooldown": 1.0, "camera_speed": 6, "movement_speed": 0.02},
    "max": {"name": "Maximum Power", "description": "En güçlü hareket - SÜPER HIZLI", "lift_range": 55, "axis_range": 60, "speed_delay": 0.01, "step_delay": 0.08, "cooldown": 0.5, "camera_speed": 8, "movement_speed": 0.01}
}

# Mesafe Hesaplama Kalibrasyonu
DISTANCE_CALIBRATION = {
    'person_height_cm': 170,
    'camera_focal_length': 600,
    'min_distance_cm': 50,
    'max_distance_cm': 300,
    'target_distance_cm': 100
}

# AI Model Dosya Yolları ve Sınıflar
AI_MODEL = {
    'prototxt_path': "/home/legendeltax/servo_control/models/MobileNetSSD_deploy.prototxt",
    'model_path': "/home/legendeltax/servo_control/models/MobileNetSSD_deploy.caffemodel",
    'classes': ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"],
    'person_class_id': 15,
    'confidence_threshold': 0.2
}

# Kamera Görüntü Ayarları
VIDEO = {
    'width': 640,
    'height': 480,
    'fps_delay': 0.033 # ~30 FPS
}


CAMERA_TRACKING_SETTINGS = {
    'p_gain': 0.08,
    'i_gain': 0.005,
    'd_gain': 0.01,
    'dead_zone_radius': 30
}
