# movement_web.py – Flask server w/ Face‑Save + Detector‑Mode toggles
# =====================================================================
from pathlib import Path
from flask import Flask, send_from_directory, Response, request, jsonify

import movement               # REX gait & servo logic
import video                  # camera + tracking (delegates IDs to security)
import security               # photo save flag

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
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n"
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ───────────────────────── Stance cfg ───────────────────────
@app.route("/cfg", methods=["GET", "POST"])
def cfg():
    if request.method == "POST":
        movement.rex.cfg.update(request.get_json(force=True) or {})
    return jsonify(movement.rex.cfg)

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
    d = request.args.get("dir", "")
    getattr(movement.rex, f"lean_{d}", lambda: None)()
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

# ───────────────────────── NEW: Face‑save toggle ────────────
@app.get("/facesave")
def tog_facesave():
    security.SAVE_FACE_IMAGES = bool(int(request.args.get("v", "0")))
    return "ON" if security.SAVE_FACE_IMAGES else "OFF"

# ───────────────────────── NEW: Detector‑mode toggle ────────
#   m = person | face | both
@app.get("/detmode")
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

# ───────────────────────── Run ───────────────────────────────
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=80)
