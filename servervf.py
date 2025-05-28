import os
import cv2
import numpy as np
import time
import json
from flask import Flask, request, Response, jsonify, render_template
import torch
from PIL import Image
import io
import base64
import threading
import socket
import logging
import argparse

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

app = Flask(__name__)

# Global variables
latest_frame = None
latest_detections = []
focused_object = None
model = None

# Function to load YOLOv5 model
def load_yolo_model():
    try:
        logger.info("Loading YOLOv5 model...")
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        logger.info("YOLOv5 model loaded successfully")
        return model
    except Exception as e:
        logger.error(f"Error loading YOLOv5 model: {e}")
        return None

# Function to send commands to the robot arm
def send_to_arm(command_dict):
    global ARM_HOST, ARM_PORT
    try:
        # Convert dict to JSON string
        command_json = json.dumps(command_dict)
        logger.info(f"Sending to arm: {command_json}")
        
        # Create a socket connection
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)  # 2 second timeout
            s.connect((ARM_HOST, ARM_PORT))
            s.sendall((command_json + '\n').encode('utf-8'))  # Add newline for ESP32 command parsing
            
            # Wait for response
            response = s.recv(1024)
            response_text = response.decode('utf-8')
            logger.info(f"Arm response: {response_text}")
            
            return True, response_text
    except socket.timeout:
        logger.error(f"Timeout connecting to arm at {ARM_HOST}:{ARM_PORT}")
        return False, "Connection timeout"
    except ConnectionRefusedError:
        logger.error(f"Connection refused by arm at {ARM_HOST}:{ARM_PORT}")
        return False, "Connection refused"
    except Exception as e:
        logger.error(f"Error communicating with arm: {e}")
        return False, str(e)

@app.route('/')
def index():
    """Serve the main page with video stream"""
    return render_template('index.html')

@app.route('/upload', methods=['POST'])
def upload_frame():
    """Receive frame from Raspberry Pi and process it"""
    global latest_frame, latest_detections, model, focused_object
    
    try:
        # Get frame from request
        file = request.files.get('frame')
        if not file:
            return jsonify({'error': 'No frame provided'}), 400
            
        # Read image from request
        img_bytes = file.read()
        nparr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if img is None or img.size == 0:
            return jsonify({'error': 'Invalid image data'}), 400
            
        # Make sure model is loaded
        if model is None:
            return jsonify({'error': 'Object detection model not loaded'}), 500
        
        # Process with YOLO
        results = model(img)
        
        # Store detection results
        latest_detections = []
        for pred in results.xyxy[0].cpu().numpy():
            x1, y1, x2, y2, conf, cls = pred
            label = model.names[int(cls)]
            
            # Only include objects with reasonable confidence
            if conf > 0.4:  # Adjustable confidence threshold
                latest_detections.append({
                    'label': label,
                    'confidence': float(conf),
                    'bbox': [int(x1), int(y1), int(x2), int(y2)]
                })
        
        # Draw detections on image
        img_with_boxes = img.copy()
        for det in latest_detections:
            x1, y1, x2, y2 = det['bbox']
            label = f"{det['label']} {det['confidence']:.2f}"
            
            # Highlight focused object differently
            color = (0, 255, 0)  # Green for regular detections
            thickness = 2
            
            # Check if this object is the focused one
            if focused_object and focused_object['label'] == det['label']:
                # Calculate centers
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                focused_center_x = (focused_object['bbox'][0] + focused_object['bbox'][2]) // 2
                focused_center_y = (focused_object['bbox'][1] + focused_object['bbox'][3]) // 2
                
                # If centers are close, consider it the same object
                if abs(center_x - focused_center_x) < 50 and abs(center_y - focused_center_y) < 50:
                    color = (0, 0, 255)  # Red for focused object
                    thickness = 3
                    
                    # Draw target at center
                    cv2.drawMarker(img_with_boxes, (center_x, center_y), 
                                 (255, 0, 0), markerType=cv2.MARKER_CROSS, 
                                 markerSize=20, thickness=2)
                    
                    # Update focused object position
                    focused_object['bbox'] = [int(x1), int(y1), int(x2), int(y2)]
            
            # Draw bounding box
            cv2.rectangle(img_with_boxes, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(img_with_boxes, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Add timestamp to image
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(img_with_boxes, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Convert to JPG and store as latest frame
        _, buffer = cv2.imencode('.jpg', img_with_boxes)
        latest_frame = buffer.tobytes()
        
        return jsonify({
            'success': True,
            'detections': latest_detections,
            'count': len(latest_detections)
        })
    
    except Exception as e:
        logger.error(f"Error processing frame: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/video_feed')
def video_feed():
    """Generate video feed for the web interface"""
    def generate():
        global latest_frame
        while True:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
            else:
                # Placeholder image when no frames are available
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(img, "Waiting for camera frames...", (50, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                _, buffer = cv2.imencode('.jpg', img)
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/latest_detections')
def get_latest_detections():
    """Return the latest detections as JSON"""
    global latest_detections
    return jsonify(latest_detections)

@app.route('/status')
def get_status():
    """Return system status"""
    global latest_detections, ARM_HOST, ARM_PORT
    
    # Check arm connection
    arm_status = "Unknown"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1)
            result = s.connect_ex((ARM_HOST, ARM_PORT))
            arm_status = "Connected" if result == 0 else "Disconnected"
    except:
        arm_status = "Error"
    
    return jsonify({
        'arm_controller': {
            'status': arm_status,
            'host': ARM_HOST,
            'port': ARM_PORT
        },
        'object_detection': {
            'model': 'YOLOv5s',
            'detections': len(latest_detections)
        },
        'system': {
            'time': time.strftime("%Y-%m-%d %H:%M:%S")
        }
    })

@app.route('/arm_control', methods=['POST'])
def arm_control():
    """Handle robot arm control commands"""
    global focused_object
    
    try:
        data = request.json
        command = data.get('command')
        
        if not command:
            return jsonify({'success': False, 'error': 'No command provided'}), 400
        
        # Handle different commands
        if command == 'focus':
            # Focus on a specific object
            x = data.get('x')
            y = data.get('y')
            label = data.get('label')
            
            if not all([x, y, label]):
                return jsonify({'success': False, 'error': 'Missing parameters'}), 400
            
            # Find the detection with this label
            for det in latest_detections:
                if det['label'] == label:
                    focused_object = det
                    logger.info(f"Focused on object: {label} at ({x}, {y})")
                    break
            
            return jsonify({'success': True, 'message': f'Focused on {label} at ({x}, {y})'})
        
        elif command == 'move_to':
            # Move arm to coordinates
            x = data.get('x')
            y = data.get('y')
            
            if not all([x, y]):
                return jsonify({'success': False, 'error': 'Missing coordinates'}), 400
            
            # Send command to arm
            success, response = send_to_arm({
                'command': 'move_to',
                'x': int(x),
                'y': int(y)
            })
            
            if success:
                return jsonify({'success': True, 'message': f'Moving to ({x}, {y})'})
            else:
                return jsonify({'success': False, 'error': response}), 500
        
        elif command == 'grab':
            # Move to and grab an object
            x = data.get('x')
            y = data.get('y')
            label = data.get('label')
            
            if not all([x, y]):
                return jsonify({'success': False, 'error': 'Missing coordinates'}), 400
            
            # Send command to arm
            success, response = send_to_arm({
                'command': 'grab',
                'x': int(x),
                'y': int(y)
            })
            
            if success:
                return jsonify({'success': True, 'message': f'Grabbing object at ({x}, {y})'})
            else:
                return jsonify({'success': False, 'error': response}), 500
        
        elif command in ['up', 'down', 'left', 'right', 'center', 'home']:
            # Simple directional commands
            success, response = send_to_arm({
                'command': command
            })
            
            if success:
                return jsonify({'success': True, 'message': f'Sent {command} command'})
            else:
                return jsonify({'success': False, 'error': response}), 500
        
        elif command == 'gripperOpen':
            # Open gripper
            success, response = send_to_arm({
                'command': 'gripper',
                'action': 'open'
            })
            
            if success:
                return jsonify({'success': True, 'message': 'Gripper opened'})
            else:
                return jsonify({'success': False, 'error': response}), 500
        
        elif command == 'gripperClose':
            # Close gripper
            success, response = send_to_arm({
                'command': 'gripper',
                'action': 'close'
            })
            
            if success:
                return jsonify({'success': True, 'message': 'Gripper closed'})
            else:
                return jsonify({'success': False, 'error': response}), 500
        
        else:
            return jsonify({'success': False, 'error': f'Unknown command: {command}'}), 400
    
    except Exception as e:
        logger.error(f"Error handling arm command: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

def main():
    global model, ARM_HOST, ARM_PORT
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Flask server for robot arm control')
    parser.add_argument('--host', default='0.0.0.0', help='Host to run the Flask server on')
    parser.add_argument('--port', type=int, default=5000, help='Port to run the Flask server on')
    parser.add_argument('--arm-host', default='192.168.110.114', help='Robot arm controller host')
    parser.add_argument('--arm-port', type=int, default=8888, help='Robot arm controller port')
    parser.add_argument('--debug', action='store_true', help='Run Flask in debug mode')
    args = parser.parse_args()
    
    # Set global variables
    ARM_HOST = args.arm_host
    ARM_PORT = args.arm_port
    
    # Create templates directory if it doesn't exist
    if not os.path.exists('templates'):
        os.makedirs('templates')
        logger.info("Created templates directory")
    
    # Load YOLOv5 model
    model = load_yolo_model()
    if model is None:
        logger.error("Failed to load YOLOv5 model. Exiting.")
        return
    
    # Run the Flask app
    logger.info(f"Starting Flask server on {args.host}:{args.port}")
    logger.info(f"Robot arm controller configured at {ARM_HOST}:{ARM_PORT}")
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)

if __name__ == '__main__':
    main()