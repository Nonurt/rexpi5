from flask import Flask, render_template, Response, jsonify
from video import generate_frames
from controller import controller
from movement_gaits import forward, back, left, right, center, dance, lean_left, lean_right

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/start_camera')
def start_camera():
    if controller.start_camera():
        return "Camera started"
    return "Failed to start camera"

@app.route('/stop_camera')
def stop_camera():
    controller.stop_camera()
    return "Camera stopped"

@app.route('/api/toggle_tracking', methods=['POST'])
def toggle_tracking():
    controller.tracking_enabled = not controller.tracking_enabled
    return jsonify({'tracking': controller.tracking_enabled})

@app.route('/forward')
def do_forward():
    forward()
    return "Forward"

@app.route('/back')
def do_back():
    back()
    return "Back"

@app.route('/left')
def do_left():
    left()
    return "Left"

@app.route('/right')
def do_right():
    right()
    return "Right"

@app.route('/center')
def do_center():
    center()
    return "Center"

@app.route('/dance')
def do_dance():
    dance()
    return "Dance"

@app.route('/lean_left')
def do_lean_left():
    lean_left()
    return "Lean Left"

@app.route('/lean_right')
def do_lean_right():
    lean_right()
    return "Lean Right"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)