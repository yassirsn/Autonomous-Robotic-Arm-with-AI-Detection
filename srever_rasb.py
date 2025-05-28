import cv2
from picamera2 import Picamera2
import time
import requests
import argparse
import numpy as np

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Raspberry Pi Camera Client for YOLO Object Detection')
    parser.add_argument('--server', type=str, required=True,
                        help='Server URL (e.g., http://192.168.139.220:5000)')
    parser.add_argument('--fps', type=int, default=10, help='Target FPS (default: 10)')
    parser.add_argument('--color-fix', choices=['rgb', 'bgr', 'hsv', 'auto'], default='auto',
                        help='Color correction method (default: auto)')
    args = parser.parse_args()
    
    # Initialize PiCamera2
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        raw={"size": (1640, 1232)},
        main={"format": 'RGB888', "size": (640, 480)}
    ))
    
    # Start the camera
    picam2.start()
    
    # Allow camera to warm up
    print("Camera warming up...")
    time.sleep(2)
    print(f"Starting to send frames to server: {args.server}")
    server_url = f"{args.server}/upload"
    frame_interval = 1.0 / args.fps  # Time between frames for target FPS
    
    try:
        while True:
            start_time = time.time()
            
            # Capture frame
            img = picam2.capture_array()
            
            # Apply color correction based on the selected method
            if args.color_fix == 'rgb':
                # Explicitly convert to RGB (although it should already be RGB888)
                img_corrected = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            elif args.color_fix == 'bgr':
                # No conversion needed, just pass BGR directly to OpenCV
                img_corrected = img
            elif args.color_fix == 'hsv':
                # Try HSV conversion for more dramatic color correction
                img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
                # You can adjust these values to correct the color tint
                img_hsv[:, :, 0] = (img_hsv[:, :, 0] + 10) % 180  # Hue shift
                img_hsv[:, :, 1] = np.clip(img_hsv[:, :, 1] * 1.2, 0, 255)  # Saturation boost
                img_corrected = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
            else:  # 'auto' or any other value
                # Try the standard conversion which works in most cases
                img_corrected = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            
            # Compress the image for faster transmission
            _, img_encoded = cv2.imencode('.jpg', img_corrected, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            # Send to server
            try:
                response = requests.post(
                    server_url,
                    files={'frame': ('frame.jpg', img_encoded.tobytes(), 'image/jpeg')},
                    timeout=5
                )
                
                if response.status_code == 200:
                    result = response.json()
                    detections = result.get('detections', [])
                    print(f"Sent frame, detected {len(detections)} objects")
                    
                    # Optional: Show labels of detected objects
                    if detections:
                        labels = [f"{d['label']} ({d['confidence']:.2f})" for d in detections]
                        print(f"Detected: {', '.join(labels)}")
                else:
                    print(f"Error from server: {response.status_code} {response.text}")
            
            except requests.exceptions.RequestException as e:
                print(f"Connection error: {e}")
            
            # Calculate time spent and sleep if needed
            elapsed = time.time() - start_time
            sleep_time = max(0, frame_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
            actual_fps = 1.0 / (time.time() - start_time)
            print(f"Current FPS: {actual_fps:.1f}")
            
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        # Cleanup
        picam2.stop()
        picam2.close()
        print("Client stopped.")

if __name__ == "__main__":
    main()