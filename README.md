# REX-Pi5 — Raspberry Pi 5 Tabanlı Web Kontrollü Dört Ayaklı Robot

Bu proje, Raspberry Pi 5 üzerinde çalışan web tabanlı bir kontrol arayüzü ile dört ayaklı bir robotu (örümcek tipi) kontrol etmenizi sağlar. Servo motorlar PCA9685 modülü ile sürülür. Arayüzde canlı video akışı, hareket komutları, servo kontrolü ve yol takibi gibi özellikler bulunur.

---

## 🔧 Donanım Gereksinimleri

- Raspberry Pi 5 (veya 4)
- PCA9685 Servo Driver
- 8× SG90 veya MG996R Servo Motor
- Kamera Modülü (USB veya CSI)
- Güç kaynağı (5V 4A önerilir)
- Wi-Fi bağlantısı

---

## 📦 Gerekli Python Kütüphaneleri

Python 3.7+ kullanmanız önerilir.

### Kurulum:

```bash
pip install -r requirements.txt

rbtormck/
│
├── web.py                 # Flask tabanlı web sunucu
├── video.py               # Kamera, görüntü işleme, takip
├── movement/              # Servo hareketleri (mock / gerçek)
│   └── rex.py             # Gerçek veya sahte servo sürümü
├── static/                # CSS, JS, stil dosyaları
│   ├── css/styles.css
│   ├── js/app.js
│   └── path_draw.png      # Çizilen yürüyüş yolu
├── templates/
│   └── index.html         # Web arayüzü
├── config.json            # Ayar dosyası (ilk açılışta oluşur)
├── models_cfg.py          # DNN model yolu konfigürasyonu
└── realworld_py.py        # Yürüyüş yolu haritası çizici
