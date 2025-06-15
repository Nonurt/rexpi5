# config.py

# Servo kanalları (LEGS + CAMERA)
SERVO_CHANNELS = {
    'front_left_axis': 0, 'front_left_lift': 1,
    'rear_left_axis': 2, 'rear_left_lift': 3,
    'rear_right_axis': 4, 'rear_right_lift': 5,
    'front_right_axis': 6, 'front_right_lift': 7,
    'camera_pan': 8, 'camera_tilt': 9
}

SERVO_PULSE_WIDTH_RANGE = (600, 2400)
I2C_FREQUENCY = 50

# === YENİ YÜRÜME YÖN HARİTASI ===
# Bacakların ters/düz montajına göre tüm yönler buradan yönetilir.
GAIT_MAPPING = {
    # Kalça servoları için İLERİ ve GERİ hareket açıları
    'HIP_FWD': {'left': 180, 'right': 0},
    'HIP_BCK': {'left': 0, 'right': 180},
    'HIP_NEUTRAL': 90
}

# === YENİLENMİŞ KAMERA KONTROL AYARLARI ===
CAMERA_SETTINGS = {
    'dead_zone_radius': 40,  # Hedefin ortada sayılacağı piksel yarıçapı.
    'pan_min': 20, 'pan_max': 160,
    'tilt_min': 45, 'tilt_max': 135,

    # --- YENİ ORANSAL KONTROL (P-CONTROLLER) AYARLARI ---
    # Bu ayarlar takibin pürüzsüzlüğünü ve hızını belirler.
    'p_gain_pan': 0.04,  # Yatay takip hassasiyeti. Düşük değer = daha yumuşak.
    'p_gain_tilt': 0.05,  # Dikey takip hassasiyeti.
    'max_adjustment': 8  # Tek bir karede kameranın max kaç derece dönebileceği (ani sıçramaları önler).
}

ROI_SETTINGS = {
    'enabled': True,
    'stop_distance': 80,
    'min_distance': 50,
    'zones': {
        'center': {'x': 220, 'y': 160, 'w': 200, 'h': 160, 'priority': 1},
        'left': {'x': 0, 'y': 120, 'w': 220, 'h': 240, 'priority': 2},
        'right': {'x': 420, 'y': 120, 'w': 220, 'h': 240, 'priority': 2}
    }
}

POWER_SETTINGS = {
    "low": {"name": "Low Power", "lift_range": 30, "axis_range": 30, "speed_delay": 0.08, "step_delay": 0.4,
            "cooldown": 2.0},
    "medium": {"name": "Medium Power", "lift_range": 35, "axis_range": 40, "speed_delay": 0.05, "step_delay": 0.25,
               "cooldown": 1.5},
    "high": {"name": "High Power", "lift_range": 45, "axis_range": 50, "speed_delay": 0.03, "step_delay": 0.15,
             "cooldown": 1.0},
    "max": {"name": "Maximum Power", "lift_range": 55, "axis_range": 60, "speed_delay": 0.01, "step_delay": 0.08,
            "cooldown": 0.5}
}

AI_MODEL = {
    'prototxt_path': "/home/legendeltax/servo_control/models/MobileNetSSD_deploy.prototxt",
    'model_path': "/home/legendeltax/servo_control/models/MobileNetSSD_deploy.caffemodel",
    'classes': ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train",
                "tvmonitor"],
    'person_class_id': 15,
    # --- İSTEDİĞİN GİBİ GÜNCELLENDİ ---
    'confidence_threshold': 0.5  # %50 ve üzeri güvenle insan olarak kabul et.
}

DISTANCE_CALIBRATION = {
    'person_height_cm': 170, 'camera_focal_length': 600,
    'min_distance_cm': 50, 'max_distance_cm': 300,
    'target_distance_cm': 100
}

VIDEO = {'width': 640, 'height': 480, 'fps_delay': 0.033}