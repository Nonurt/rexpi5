from flask import Flask, render_template, request, jsonify, Response
import time
import threading
import cv2
import numpy as np

app = Flask(__name__)

# These will be initialized by main.py
controller = None
ai_model = None
camera_tracker = None


def generate_frames():
    """Generate video frames for streaming"""
    while True:
        if controller.current_frame is not None:
            try:
                ret, buffer = cv2.imencode('.jpg', controller.current_frame)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception as e:
                print(f"[ERROR] Frame encoding error: {e}")
        time.sleep(0.033)  # ~30 FPS


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/start_camera', methods=['POST'])
def start_camera():
    if controller.start_camera():
        return jsonify({'status': 'success', 'message': 'Camera started successfully'})
    else:
        return jsonify({'status': 'error', 'message': 'Failed to start camera'})


@app.route('/api/toggle_tracking', methods=['POST'])
def toggle_tracking():
    controller.tracking_enabled = not controller.tracking_enabled
    status = "enabled" if controller.tracking_enabled else "disabled"
    return jsonify({'status': 'success', 'message': f'Human tracking {status}', 'tracking': controller.tracking_enabled})


@app.route('/api/toggle_camera_tracking', methods=['POST'])
def toggle_camera_tracking():
    controller.camera_tracking = not controller.camera_tracking
    status = "enabled" if controller.camera_tracking else "disabled"
    return jsonify({'status': 'success', 'message': f'Camera tracking {status}', 'camera_tracking': controller.camera_tracking})


@app.route('/api/set_power_mode', methods=['POST'])
def set_power_mode():
    data = request.get_json()
    mode = data.get('mode', 'medium')
    if controller.set_power_mode(mode):
        settings = controller.get_power_settings()
        return jsonify({
            'status': 'success',
            'message': f'Power mode set to {settings["name"]}',
            'power_mode': mode,
            'settings': settings
        })
    else:
        return jsonify({'status': 'error', 'message': 'Invalid power mode'})


@app.route('/api/manual_move', methods=['POST'])
def manual_move():
    data = request.get_json()
    action = data.get('action')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    action_map = {
        'forward': controller.spider_walk_forward,
        'left': controller.spider_turn_left,
        'right': controller.spider_turn_right,
        'back': controller.spider_back_away,
        'stance': controller.spider_stance_position,
        'defensive': controller.spider_defensive_posture,
        'attack': controller.spider_attack_posture,
        'crouch': controller.spider_crouch_low,
    }

    try:
        if action in action_map:
            threading.Thread(target=action_map[action], daemon=True).start()
            return jsonify({'status': 'success', 'message': f'Executing {action} movement'})
        else:
            return jsonify({'status': 'error', 'message': 'Invalid action'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Movement error: {str(e)}'})


@app.route('/api/camera_control', methods=['POST'])
def camera_control():
    data = request.get_json()
    action = data.get('action')
    try:
        if action == 'center':
            controller.set_camera_position(pan_angle=90, tilt_angle=90)
        elif action == 'left':
            new_pan = max(controller.camera_limits['pan_min'], controller.camera_pan_angle - 10)
            controller.set_camera_position(pan_angle=new_pan)
        elif action == 'right':
            new_pan = min(controller.camera_limits['pan_max'], controller.camera_pan_angle + 10)
            controller.set_camera_position(pan_angle=new_pan)
        elif action == 'up':
            new_tilt = max(controller.camera_limits['tilt_min'], controller.camera_tilt_angle - 10)
            controller.set_camera_position(tilt_angle=new_tilt)
        elif action == 'down':
            new_tilt = min(controller.camera_limits['tilt_max'], controller.camera_tilt_angle + 10)
            controller.set_camera_position(tilt_angle=new_tilt)
        else:
            return jsonify({'status': 'error', 'message': 'Invalid camera action'})

        return jsonify({
            'status': 'success',
            'message': f'Camera moved {action}',
            'pan': controller.camera_pan_angle,
            'tilt': controller.camera_tilt_angle
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Camera control error: {str(e)}'})


@app.route('/api/roi_settings', methods=['POST'])
def update_roi_settings():
    data = request.get_json()
    if 'stop_distance' in data:
        controller.roi_stop_distance = max(30, min(200, int(data['stop_distance'])))
    if 'min_distance' in data:
        controller.roi_min_distance = max(20, min(100, int(data['min_distance'])))
    if 'enabled' in data:
        controller.roi_enabled = bool(data['enabled'])

    return jsonify({
        'status': 'success',
        'message': 'ROI settings updated',
        'settings': {
            'stop_distance': controller.roi_stop_distance,
            'min_distance': controller.roi_min_distance,
            'enabled': controller.roi_enabled
        }
    })


@app.route('/api/leg_control', methods=['POST'])
def leg_control():
    data = request.get_json()
    action = data.get('action')
    leg_group = data.get('leg_group', 'all')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        action_map = {
            'lift': controller.lift_legs,
            'lower': controller.lower_legs,
            'spread': controller.spread_legs,
            'center': controller.center_legs
        }
        if action in action_map:
            threading.Thread(target=action_map[action], args=(leg_group,), daemon=True).start()
            return jsonify({'status': 'success', 'message': f'{action} {leg_group} legs'})
        else:
            return jsonify({'status': 'error', 'message': 'Invalid leg action'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Leg control error: {str(e)}'})


@app.route('/api/gait_control', methods=['POST'])
def gait_control():
    data = request.get_json()
    gait_type = data.get('gait_type')
    direction = data.get('direction', 'forward')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        gait_map = {
            'front_lift_walk': controller.front_lift_gait,
            'rear_drive_walk': controller.rear_drive_gait,
            'alternating_pairs': controller.alternating_pairs_gait
        }
        if gait_type in gait_map:
            threading.Thread(target=gait_map[gait_type], args=(direction,), daemon=True).start()
            return jsonify({'status': 'success', 'message': f'Executing {gait_type} gait {direction}'})
        else:
            return jsonify({'status': 'error', 'message': 'Invalid gait type'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Gait control error: {str(e)}'})


@app.route('/api/camera_settings', methods=['POST'])
def update_camera_settings():
    data = request.get_json()
    if 'smooth_delay' in data:
        controller.camera_smooth_delay = max(0.01, min(0.2, float(data['smooth_delay'])))
    if 'dead_zone_radius' in data:
        controller.dead_zone_radius = max(20, min(150, int(data['dead_zone_radius'])))
    if 'step_size' in data:
        controller.camera_step_size = max(1, min(10, int(data['step_size'])))

    return jsonify({
        'status': 'success',
        'message': 'Camera settings updated',
        'settings': {
            'smooth_delay': controller.camera_smooth_delay,
            'dead_zone_radius': controller.dead_zone_radius,
            'step_size': controller.camera_step_size
        }
    })


@app.route('/api/status', methods=['GET'])
def get_status():
    return jsonify({
        'tracking_enabled': controller.tracking_enabled,
        'camera_tracking': controller.camera_tracking,
        'human_detected': controller.human_detected,
        'human_distance': controller.human_distance,
        'roi_zone': controller.last_roi_zone,
        'power_mode': controller.power_mode,
        'walking': controller.walking,
        'camera_running': controller.is_camera_running,
        'camera_position': {
            'pan': controller.camera_pan_angle,
            'tilt': controller.camera_tilt_angle
        },
        'roi_settings': {
            'stop_distance': controller.roi_stop_distance,
            'min_distance': controller.roi_min_distance,
            'enabled': controller.roi_enabled
        }
    })


@app.route('/api/stop_all', methods=['POST'])
def stop_all():
    controller.tracking_enabled = False
    controller.camera_tracking = False
    controller.walking = False
    threading.Thread(target=controller.spider_stance_position, daemon=True).start()
    return jsonify({'status': 'success', 'message': 'All functions stopped - Emergency stop activated'})


@app.route('/api/advanced_move', methods=['POST'])
def advanced_move():
    data = request.get_json()
    action = data.get('action')
    steps = data.get('steps', 1)
    distance = data.get('distance', 0)

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        if action == 'adaptive_walk':
            direction = data.get('direction', 'forward')
            threading.Thread(target=controller.spider_adaptive_walk, args=(direction, distance), daemon=True).start()
        elif action == 'wave_gait':
            threading.Thread(target=controller.spider_wave_gait, daemon=True).start()
        elif action == 'creep_gait':
            threading.Thread(target=controller.spider_creep_gait, daemon=True).start()
        elif action == 'multi_step':
            direction = data.get('direction', 'forward')
            threading.Thread(target=controller.spider_walk_algorithm_v2, args=(direction, steps), daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid advanced action'})
        return jsonify({'status': 'success', 'message': f'Executing {action}'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Advanced movement error: {str(e)}'})


@app.route('/api/rex_movement', methods=['POST'])
def rex_movement():
    data = request.get_json()
    action = data.get('action')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        action_map = {
            'rex_forward': controller.rex_forward_gait,
            'rex_backward': controller.rex_backward_gait,
            'rex_left': controller.rex_turn_left,
            'rex_right': controller.rex_turn_right,
            'rex_stabilize': controller.rex_stabilize,
            'rex_lean_left': controller.rex_lean_left,
            'rex_lean_right': controller.rex_lean_right,
            'rex_lean_forward': controller.rex_lean_forward,
            'rex_lean_back': controller.rex_lean_back
        }
        if action in action_map:
            threading.Thread(target=action_map[action], daemon=True).start()
            return jsonify({'status': 'success', 'message': f'Executing REX {action}'})
        else:
            return jsonify({'status': 'error', 'message': 'Invalid REX action'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'REX movement error: {str(e)}'})


@app.route('/api/rex_stance_control', methods=['POST'])
def rex_stance_control():
    data = request.get_json()
    height = data.get('height', 60)

    if hasattr(controller, 'rex_stance_height'):
        controller.rex_stance_height = max(30, min(70, int(height)))
    else:
        controller.rex_stance_height = 60

    return jsonify({
        'status': 'success',
        'message': f'REX stance height set to {controller.rex_stance_height}',
        'height': controller.rex_stance_height
    })


def init_web_interface(ctrl, model=None, tracker=None):
    """Initialize the web interface with controller reference"""
    global controller, ai_model, camera_tracker
    controller = ctrl
    ai_model = model
    camera_tracker = tracker
    app.config['controller'] = controller
