# ======================================
#  web.py – Flask server for REX Pi 5
# ======================================
"""
• Klasik gait rotaları (/forward, /back, /left, /right, /center)
• /servo?cmd="0:120 6:30"   → ham servo yaz
• /cfg  (GET/POST)          → JSON konfigürasyon
• /trim?d=±1                → tüm kalça trim’ini adım adım ayarla
• Kamera işleme toggles     → /track /gamma /hist /autostep
• Yüz kaydetme & dedektör modu                → /facesave /detmode
• MJPEG akışı               → /stream.mjpg
"""

from pathlib import Path
from flask import Flask, send_from_directory, Response, request, jsonify

import movement               # servo & gait mantığı
import video                  # kamera + takip (security’e delege)
import security               # yüz fotoğrafı kaydetme bayrağı

APP_ROOT   = Path(__file__).parent
STATIC_DIR = APP_ROOT / "static"
app        = Flask(__name__, static_folder=str(STATIC_DIR))

# ───────────────────────── UI root ──────────────────────────
@app.get("/")
def root():
    return send_from_directory(app.static_folder, "index.html")

# ───────────────────────── MJPEG stream ─────────────────────
@app.get("/stream.mjpg")
def stream():
    def gen():
        while True:
            buf = video.next_frame()
            if buf is None:
                break
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   buf +
                   b"\r\n")
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ───────────────────────── Config (/cfg) ────────────────────
@app.route("/cfg", methods=["GET", "POST"])
def cfg():
    if request.method == "POST":
        movement.rex.cfg.update(request.get_json(force=True) or {})
        movement.rex.save_cfg()
    return jsonify(movement.rex.cfg)

# ───────────────────────── Quick-trim helper ────────────────
@app.get("/trim")
def trim():
    """/trim?d=1 → tüm hip trim +1; d=-1 → -1"""
    try:
        delta = int(request.args.get("d", "0"))
    except ValueError:
        delta = 0
    if delta:
        movement.rex.adjust_trim(delta)
    return jsonify({"trim": movement.rex.cfg["trim"]})

# ───────────────────────── Raw servo ────────────────────────
@app.get("/servo")
def servo():
    cmd = request.args.get("cmd", "")
    if cmd:
        movement.raw_servo_cmd(cmd)
    return "OK"

# ───────────────────────── Gait routes ──────────────────────
@app.get("/forward")
@app.get("/back")
@app.get("/left")
@app.get("/right")
@app.get("/center")
def gait():
    actions = {
        "/forward": movement.rex.forward,
        "/back"   : movement.rex.back,
        "/left"   : movement.rex.turn_left,
        "/right"  : movement.rex.turn_right,
        "/center" : movement.rex.center_servos,
    }
    actions[request.path]()
    return "OK"

# ───────────────────────── Lean helper ──────────────────────
@app.get("/lean")
def lean():
    dir_ = request.args.get("dir", "")
    getattr(movement.rex, f"lean_{dir_}", lambda: None)()
    return "OK"

# ───────────────────────── Toggles (cam proc) ───────────────
@app.get("/track")
def tog_track():
    video.TRACK = bool(int(request.args.get("v", "0")))
    return "ON" if video.TRACK else "OFF"

@app.get("/autostep")
def tog_step():
    video.AUTOSTEP = video.APPROACH = bool(int(request.args.get("v", "0")))
    return "ON" if video.AUTOSTEP else "OFF"

@app.get("/gamma")
def tog_gamma():
    video.AUTO_GAMMA = bool(int(request.args.get("v", "0")))
    return "ON" if video.AUTO_GAMMA else "OFF"

@app.get("/hist")
def tog_hist():
    video.AUTO_HIST = bool(int(request.args.get("v", "0")))
    return "ON" if video.AUTO_HIST else "OFF"

# ───────────────────────── Face-save & mode ─────────────────
@app.get("/facesave")
def tog_facesave():
    security.SAVE_FACE_IMAGES = bool(int(request.args.get("v", "0")))
    return "ON" if security.SAVE_FACE_IMAGES else "OFF"

@app.get("/detmode")          # m = person | face | both
def detector_mode():
    m = request.args.get("m", "both")
    if m not in {"person", "face", "both"}:
        return "ERR", 400
    video.DETECT_MODE = m
    return m

# ───────────────────────── Target list / select ─────────────
@app.get("/targets")
def targets():
    return jsonify(video.get_targets())

@app.get("/target")
def target():
    tid = request.args.get("id")
    video.set_target(int(tid) if tid else None)
    return "OK"

# ───────────────────────── Status JSON ──────────────────────
@app.get("/status")
def status():
    return jsonify({
        "track"        : video.TRACK,
        "autostep"     : video.AUTOSTEP,
        "gamma"        : video.AUTO_GAMMA,
        "hist"         : video.AUTO_HIST,
        "facesave"     : security.SAVE_FACE_IMAGES,
        "detector_mode": video.DETECT_MODE,
        "stance"       : movement.rex.cfg.get("stance_height", 60)
    })

# ───────────────────────── Run ──────────────────────────────
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
