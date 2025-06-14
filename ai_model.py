import cv2
import numpy as np


class AIModel:
    def __init__(self):
        self.net = None
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]

        # Distance calculation parameters
        self.distance_calibration = {
            'person_height_cm': 170,  # Average person height
            'camera_focal_length': 600,  # Camera focal length in pixels
            'min_distance_cm': 50,  # Minimum reliable detection distance
            'max_distance_cm': 300,  # Maximum reliable detection distance
            'target_distance_cm': 100  # Ideal tracking distance
        }

        self.init_model()

    def init_model(self):
        """Initialize the AI detection model"""
        try:
            # Model file paths (update these paths as needed)
            prototxt_path = "models/MobileNetSSD_deploy.prototxt"
            model_path = "models/MobileNetSSD_deploy.caffemodel"

            print("[AI] Loading object detection model...")
            self.net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)

            # Try to use CUDA if available
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                print("[AI] Using CUDA acceleration")
            except:
                print("[AI] CUDA not available, using CPU")

            print("[AI] Model loaded successfully")
        except Exception as e:
            print(f"[ERROR] Failed to load AI model: {e}")
            self.net = None

    def detect_humans(self, frame):
        """
        Detect humans in a frame using the AI model
        Args:
            frame: Input image frame (BGR format)
        Returns:
            tuple: (processed_frame, detections)
                   processed_frame: Frame with detection visualizations
                   detections: List of detected humans with metadata
        """
        if self.net is None:
            return frame, []

        (h, w) = frame.shape[:2]
        processed_frame = frame.copy()
        detections = []

        # Prepare the frame for the neural network
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                     0.007843, (300, 300), 127.5)

        # Perform detection
        self.net.setInput(blob)
        network_detections = self.net.forward()

        # Process detections
        for i in range(network_detections.shape[2]):
            confidence = network_detections[0, 0, i, 2]
            class_id = int(network_detections[0, 0, i, 1])

            # Only process person detections with sufficient confidence
            if class_id == 15 and confidence > 0.3:  # Class 15 is 'person'
                box = network_detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Calculate bounding box center and dimensions
                center_x = (startX + endX) // 2
                center_y = (startY + endY) // 2
                bbox_width = endX - startX
                bbox_height = endY - startY

                # Calculate distance to person
                distance = self.calculate_distance(bbox_height)

                # Store detection information
                detection = {
                    'bbox': (startX, startY, endX, endY),
                    'center': (center_x, center_y),
                    'confidence': confidence,
                    'distance': distance,
                    'width': bbox_width,
                    'height': bbox_height
                }
                detections.append(detection)

                # Draw bounding box and info on frame
                cv2.rectangle(processed_frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
                cv2.circle(processed_frame, (center_x, center_y), 5, (0, 255, 0), -1)

                # Display confidence and distance
                text = f"Person: {confidence:.2f} ({distance}cm)"
                cv2.putText(processed_frame, text, (startX, startY - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return processed_frame, detections

    def calculate_distance(self, bbox_height):
        """
        Calculate distance to detected person based on bounding box height
        Args:
            bbox_height: Height of the detected person's bounding box in pixels
        Returns:
            int: Estimated distance in centimeters
        """
        if bbox_height <= 0:
            return 0

        # Distance calculation formula:
        # distance = (real_height * focal_length) / apparent_height
        distance = (self.distance_calibration['person_height_cm'] *
                    self.distance_calibration['camera_focal_length']) / bbox_height

        # Constrain distance to reasonable limits
        distance = max(self.distance_calibration['min_distance_cm'],
                       min(self.distance_calibration['max_distance_cm'], distance))

        return int(distance)

    def update_calibration(self, new_calibration):
        """
        Update distance calculation parameters
        Args:
            new_calibration: Dictionary with calibration values:
                            {
                                'person_height_cm': float,
                                'camera_focal_length': float,
                                'min_distance_cm': float,
                                'max_distance_cm': float
                            }
        """
        for key, value in new_calibration.items():
            if key in self.distance_calibration:
                self.distance_calibration[key] = value
        print("[AI] Calibration updated:", self.distance_calibration)