# REX-Pi5 — Web-Controlled Quadruped Robot Powered by Raspberry Pi 5

**REX-Pi5** is a web-controlled quadruped robot project based on the Raspberry Pi 5. It uses a web-based interface to control walking, leaning, and other movements. The servos are driven via a PCA9685 controller. The interface supports live video streaming, servo control via sliders or raw commands, and movement path visualization.

---

## 🔧 Hardware Requirements

- Raspberry Pi 5 (or Raspberry Pi 4)
- PCA9685 16-channel Servo Driver
- 8× SG90 or MG996R Servo Motors
- USB or CSI Camera Module
- Stable 5V 4A power source
- Wi-Fi connection

---

## 📦 Python Library Requirements

Recommended: Python 3.7+

### Installation:

```bash
pip install -r requirements.txt
```

---

## 📁 Project Structure

```
rbtormck/
│
├── web.py                 # Flask-based web server
├── video.py               # Camera, tracking, vision processing
├── movement/              # Servo movement logic (mock/real)
│   └── rex.py             # Main robot movement implementation
├── static/                # CSS, JS, and static assets
│   ├── css/styles.css
│   ├── js/app.js
│   └── path_draw.png      # Movement path overlay
├── templates/
│   └── index.html         # Main UI layout
├── config.json            # Runtime config file (auto-generated)
├── models_cfg.py          # DNN model path definitions
└── realworld_py.py        # Real-world path drawer
```

---

## ✅ Features

- Full web interface (control via browser)
- Live video feed with tracking
- Path visualization system
- Manual and slider-based servo control
- Smooth directional movements
- Leaning and centering operations
- Adjustable stance height
