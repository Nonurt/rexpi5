#!/usr/bin/env python3
import time
import threading
from flask import Flask
from camera_tracker import CameraTracker
from ai_model import AIModel
from rex_gait import RexGait
from web_interface import app, init_web_interface
import os


class MainController:
    def __init__(self):
        # Initialize components
        print("[SYSTEM] Initializing components...")

        # Initialize AI Model first
        self.ai_model = AIModel()

        # Initialize Servo Controller
        self.servo_controller = RexGait()

        # Add attributes expected by camera_tracker
        self.power_mode = "medium"
        self.camera_smooth_delay = 0.05
        self.camera_limits = {
            'pan_min': 30,
            'pan_max': 150,
            'tilt_min': 30,
            'tilt_max': 150
        }
        self.camera_pan_angle = 90
        self.camera_tilt_angle = 90

        # Initialize Camera Tracker
        self.camera_tracker = CameraTracker(self)

        # Configure web interface
        init_web_interface(self)

        # System status
        self.system_ready = False
        self.shutdown_flag = False

    def start_system(self):
        """Start all system components"""
        if self.system_ready:
            print("[SYSTEM] System already running")
            return True

        print("[SYSTEM] Starting system components...")

        try:
            # Verify model files exist
            if not os.path.exists("models/MobileNetSSD_deploy.prototxt"):
                raise FileNotFoundError("Missing model prototxt file")
            if not os.path.exists("models/MobileNetSSD_deploy.caffemodel"):
                raise FileNotFoundError("Missing model weights file")

            # Start camera
            if not self.camera_tracker.start_camera():
                raise RuntimeError("Failed to start camera")

            # Initialize servos to default position
            self.servo_controller.init_camera_position()
            self.servo_controller.spider_stance_position()

            self.system_ready = True
            print("[SYSTEM] All components started successfully")
            return True

        except Exception as e:
            print(f"[ERROR] System startup failed: {e}")
            self.system_ready = False
            return False

    def stop_system(self):
        """Gracefully shutdown all components"""
        print("[SYSTEM] Shutting down components...")
        self.shutdown_flag = True

        # Stop tracking first
        self.camera_tracker.tracking_enabled = False
        self.camera_tracker.camera_tracking = False

        # Stop camera
        self.camera_tracker.stop_camera()

        # Return servos to neutral position
        self.servo_controller.spider_stance_position()
        self.servo_controller.set_camera_position(90, 90)

        print("[SYSTEM] Shutdown complete")

    def system_status(self):
        """Get current system status"""
        return {
            'system_ready': self.system_ready,
            'camera_running': getattr(self.camera_tracker, 'is_camera_running', False),
            'ai_model_loaded': self.ai_model.net is not None,
            'tracking_active': getattr(self.camera_tracker, 'tracking_enabled', False),
            'camera_tracking': getattr(self.camera_tracker, 'camera_tracking', False),
            'servos_active': not self.servo_controller.walking,
            'human_detected': getattr(self.camera_tracker, 'human_detected', False),
            'human_distance': getattr(self.camera_tracker, 'human_distance', 0),
            'power_mode': self.power_mode
        }


def run_flask_app():
    """Run the Flask web interface"""
    print("[WEB] Starting web interface...")
    # Ensure templates folder exists
    if not os.path.exists("templates"):
        os.makedirs("templates")
        # Create basic index.html if missing
        if not os.path.exists("templates/index.html"):
            with open("templates/index.html", "w") as f:
                f.write("""<!DOCTYPE html>
<html>
<head>
    <title>REX Control Interface</title>
</head>
<body>
    <h1>REX Robot Control Interface</h1>
    <div id="status">
        <p>System Status: <span id="system-status">Loading...</span></p>
    </div>
</body>
</html>""")

    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)


def main():
    try:
        # Create and initialize main controller
        controller = MainController()

        # Start system components
        if not controller.start_system():
            raise RuntimeError("System initialization failed")

        # Start web interface in a separate thread
        web_thread = threading.Thread(target=run_flask_app, daemon=True)
        web_thread.start()

        # Main loop
        print("\n[SYSTEM] === SYSTEM READY ===")
        print("Control interface available at: http://localhost:5000")

        while not controller.shutdown_flag:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutdown requested by user")
    except Exception as e:
        print(f"\n[ERROR] System failure: {e}")
    finally:
        if 'controller' in locals():
            controller.stop_system()
        print("[SYSTEM] Exiting")


if __name__ == "__main__":
    main()