# === GEREKLİ MODÜLLER ===
from flask import Flask, jsonify, request                     # Flask API fonksiyonları için
import threading                                              # Paralel yürüyüş/servo hareketleri için

from web import app                                           # Flask uygulaması tanımı (web.py içinden geliyor)
from your_controller_module import HumanTrackingServoController  # controller sınıfın (doğru dosya ismini yaz)


from flask import Flask, jsonify, request
import threading
from web import app
from your_controller_module import HumanTrackingServoController  # Replace with your actual controller module

# Initialize controller
controller = HumanTrackingServoController()

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
    return jsonify(
        {'status': 'success', 'message': f'Human tracking {status}', 'tracking': controller.tracking_enabled})


@app.route('/api/toggle_camera_tracking', methods=['POST'])
def toggle_camera_tracking():
    controller.camera_tracking = not controller.camera_tracking
    status = "enabled" if controller.camera_tracking else "disabled"
    return jsonify(
        {'status': 'success', 'message': f'Camera tracking {status}', 'camera_tracking': controller.camera_tracking})


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

    try:
        if action == 'forward':
            threading.Thread(target=controller.spider_walk_forward, daemon=True).start()
        elif action == 'left':
            threading.Thread(target=controller.spider_turn_left, daemon=True).start()
        elif action == 'right':
            threading.Thread(target=controller.spider_turn_right, daemon=True).start()
        elif action == 'back':
            threading.Thread(target=controller.spider_back_away, daemon=True).start()
        elif action == 'stance':
            threading.Thread(target=controller.spider_stance_position, daemon=True).start()
        elif action == 'defensive':
            threading.Thread(target=controller.spider_defensive_posture, daemon=True).start()
        elif action == 'attack':
            threading.Thread(target=controller.spider_attack_posture, daemon=True).start()
        elif action == 'crouch':
            threading.Thread(target=controller.spider_crouch_low, daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid action'})

        return jsonify({'status': 'success', 'message': f'Executing {action} movement'})
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
    """Specific leg movement control"""
    data = request.get_json()
    action = data.get('action')
    leg_group = data.get('leg_group', 'all')  # front, rear, left, right, all

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        if action == 'lift':
            threading.Thread(target=controller.lift_legs, args=(leg_group,), daemon=True).start()
        elif action == 'lower':
            threading.Thread(target=controller.lower_legs, args=(leg_group,), daemon=True).start()
        elif action == 'spread':
            threading.Thread(target=controller.spread_legs, args=(leg_group,), daemon=True).start()
        elif action == 'center':
            threading.Thread(target=controller.center_legs, args=(leg_group,), daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid leg action'})

        return jsonify({'status': 'success', 'message': f'{action} {leg_group} legs'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Leg control error: {str(e)}'})


@app.route('/api/gait_control', methods=['POST'])
def gait_control():
    """Advanced gait pattern control"""
    data = request.get_json()
    gait_type = data.get('gait_type')
    direction = data.get('direction', 'forward')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        if gait_type == 'front_lift_walk':
            threading.Thread(target=controller.front_lift_gait, args=(direction,), daemon=True).start()
        elif gait_type == 'rear_drive_walk':
            threading.Thread(target=controller.rear_drive_gait, args=(direction,), daemon=True).start()
        elif gait_type == 'alternating_pairs':
            threading.Thread(target=controller.alternating_pairs_gait, args=(direction,), daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid gait type'})

        return jsonify({'status': 'success', 'message': f'Executing {gait_type} gait {direction}'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Gait control error: {str(e)}'})


@app.route('/api/camera_settings', methods=['POST'])
def update_camera_settings():
    """Update camera control settings"""
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
    """Emergency stop all functions"""
    controller.tracking_enabled = False
    controller.camera_tracking = False
    controller.walking = False

    # Return to stance position
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
    """REX Quad style movement control"""
    data = request.get_json()
    action = data.get('action')

    if controller.walking:
        return jsonify({'status': 'error', 'message': 'Robot is currently walking'})

    try:
        if action == 'rex_forward':
            threading.Thread(target=controller.rex_forward_gait, daemon=True).start()
        elif action == 'rex_backward':
            threading.Thread(target=controller.rex_backward_gait, daemon=True).start()
        elif action == 'rex_left':
            threading.Thread(target=controller.rex_turn_left, daemon=True).start()
        elif action == 'rex_right':
            threading.Thread(target=controller.rex_turn_right, daemon=True).start()
        elif action == 'rex_stabilize':
            threading.Thread(target=controller.rex_stabilize, daemon=True).start()
        elif action == 'rex_lean_left':
            threading.Thread(target=controller.rex_lean_left, daemon=True).start()
        elif action == 'rex_lean_right':
            threading.Thread(target=controller.rex_lean_right, daemon=True).start()
        elif action == 'rex_lean_forward':
            threading.Thread(target=controller.rex_lean_forward, daemon=True).start()
        elif action == 'rex_lean_back':
            threading.Thread(target=controller.rex_lean_back, daemon=True).start()
        else:
            return jsonify({'status': 'error', 'message': 'Invalid REX action'})

        return jsonify({'status': 'success', 'message': f'Executing REX {action}'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'REX movement error: {str(e)}'})


@app.route('/api/rex_stance_control', methods=['POST'])
def rex_stance_control():
    """REX stance height control"""
    data = request.get_json()
    height = data.get('height', 60)  # Default stance height

    # Update controller's stance height for REX functions
    if hasattr(controller, 'rex_stance_height'):
        controller.rex_stance_height = max(30, min(70, int(height)))
    else:
        controller.rex_stance_height = 60

    return jsonify({
        'status': 'success',
        'message': f'REX stance height set to {controller.rex_stance_height}',
        'height': controller.rex_stance_height
    })