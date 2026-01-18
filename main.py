import cv2
import cvzone
import math
import time
import mediapipe as mp
from ultralytics import YOLO
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import torch
import serial
# --- 1. SETUP SERIAL CONNECTION (Before the main loop) ---
# REPLACE 'COM3' with your actual port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Mac/Linux)
# Ensure baud rate (9600) matches your Arduino sketch
try:
    arduino = serial.Serial('COM3', 9600, timeout=1) 
    time.sleep(2) # Give Arduino 2 seconds to reset/wake up
    print("Connected to Arduino!")
except:
    print("WARNING: Arduino not connected. Check COM port.")
    arduino = None

# --- 1. CONFIGURATION & SETUP ---
# Check for GPU (from your YOLO code)
print(f"Is CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA device: {torch.cuda.get_device_name(0)}")
else:
    print("CUDA is not available. Running on CPU.")


# Paths
yolo_model_path = "./yolov8n.pt"      # Your YOLO model
hand_model_path = './hand_landmarker.task' # Your MediaPipe model

# Initialize YOLO
yolo_model = YOLO(yolo_model_path)

# Initialize MediaPipe Variables
latest_mp_result = None

def save_result(result, output_image: mp.Image, timestamp_ms: int):
    global latest_mp_result
    latest_mp_result = result

# --- 2. HELPER FUNCTIONS (Hand Tracking) ---

def get_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def count_fingers(detection_result):
    left_hand_count = None
    right_hand_count = None
    
    if detection_result.hand_landmarks:
        for hand, hand_info in zip(detection_result.hand_landmarks, detection_result.handedness):
            label = hand_info[0].category_name
            wrist = hand[0]
            current_hand_fingers = 0
            
            # Thumb Logic (Distance Method)
            thumb_tip_dist = get_distance(wrist, hand[4])
            thumb_ip_dist = get_distance(wrist, hand[3])
            
            if thumb_tip_dist > thumb_ip_dist * 1.1:
                current_hand_fingers += 1
                
            # Other 4 Fingers
            finger_tips = [8, 12, 16, 20]
            finger_pips = [6, 10, 14, 18]
            
            for tip, pip in zip(finger_tips, finger_pips):
                if get_distance(wrist, hand[tip]) > get_distance(wrist, hand[pip]):
                    current_hand_fingers += 1
            
            if label == "Left":
                left_hand_count = current_hand_fingers
            elif label == "Right":
                right_hand_count = current_hand_fingers
                    
    return left_hand_count, right_hand_count

def draw_hand_landmarks(image, detection_result):
    # Standard connections
    HAND_CONNECTIONS = [

        (0, 1), (1, 2), (2, 3), (3, 4),   # Thumb

        (0, 5), (5, 6), (6, 7), (7, 8),   # Index

        (5, 9), (9, 10), (10, 11), (11, 12), # Middle

        (9, 13), (13, 14), (14, 15), (15, 16), # Ring

        (13, 17), (17, 18), (18, 19), (19, 20), # Pinky

        (0, 17) # Palm base to Pinky base
    ]
    
    if detection_result.hand_landmarks:
        for hand_landmarks in detection_result.hand_landmarks:
            height, width, _ = image.shape
            points = []
            for landmark in hand_landmarks:
                x = int(landmark.x * width)
                y = int(landmark.y * height)
                points.append((x, y))

            # Draw Connections
            # We must manually map connections since we aren't using mp_drawing.draw_landmarks
            # A simple loop over the predefined connections:
            for connection in HAND_CONNECTIONS:
                start_idx = connection[0]
                end_idx = connection[1]
                cv2.line(image, points[start_idx], points[end_idx], (0, 255, 0), 2)

            # Draw Joints
            for point in points:
                cv2.circle(image, point, 4, (0, 0, 255), -1)
    return image

# --- 3. MAIN EXECUTION ---
def main():
    # Setup MediaPipe
    base_options = python.BaseOptions(model_asset_path=hand_model_path)
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.LIVE_STREAM,
        num_hands=2,
        result_callback=save_result)

    # Webcam Setup
    cap = cv2.VideoCapture(0)
    cap.set(3, 1280) # Width
    cap.set(4, 720)  # Height

    # Start MediaPipe Context
    with vision.HandLandmarker.create_from_options(options) as landmarker:
        print("System Ready. Showing YOLO + Hand Tracking.")

    # Initialize "Previous State" variables to track changes
        prev_left_grasp = False   # <--- ADD THIS
        prev_right_grasp = False  # <--- ADD THIS

        while True:
            # 1. READ FRAME
            success, raw_frame = cap.read()
            if not success:
                print("ERROR: Camera failed to return a frame. Check index or connection.")
                break

            # 2. FLIP (Optional: Mirror Effect)
            # We flip the raw frame so BOTH models see the mirrored version
            #raw_frame = cv2.flip(raw_frame, 1)

            # 3. GOAL A: CREATE A COPY FOR DRAWING
            # 'raw_frame' is clean for AI processing.
            # 'img_display' is where we draw boxes and skeletons.
            img_display = raw_frame.copy()

            # --- PROCESS A: YOLO OBJECT DETECTION ---
            # 1. Define Screen Center & Zone Size
            height, width, _ = img_display.shape
            center_x, center_y = width // 2, height // 2
            pixel_threshold = 250  # <--- ADJUST THIS to make zone bigger/smaller
            
            # (Optional) Draw the Focus Zone so you can see it
            cv2.circle(img_display, (center_x, center_y), pixel_threshold, (0, 255, 0), 2)

            # ... (YOLO inference) ...
            yolo_results = yolo_model(raw_frame, stream=True, verbose=False)
            
            for r in yolo_results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    
                    # 2. Calculate Object Center
                    obj_cx = (x1 + x2) // 2
                    obj_cy = (y1 + y2) // 2
                    
                    # 3. Calculate Distance from Screen Center
                    distance = math.sqrt((center_x - obj_cx)**2 + (center_y - obj_cy)**2)
                    
                    # 4. FILTER: Only show if inside the zone
                    if distance < pixel_threshold:
                        
                        # Draw Box (Only executes for center objects)
                        cv2.rectangle(img_display, (x1, y1), (x2, y2), (255, 0, 255), 3)
                        
                        # Draw Confidence
                        conf = math.ceil((box.conf[0] * 100)) / 100
                        cls = int(box.cls[0])
                        current_class = yolo_model.names[cls]
                        cvzone.putTextRect(img_display, f'{current_class} {conf}', 
                                         (max(0, x1), max(35, y1)), scale=1, thickness=1)
                        
                        # (Optional) Draw a line to the center to show "Lock"
                        cv2.line(img_display, (center_x, center_y), (obj_cx, obj_cy), (0, 255, 255), 2)

            # --- PROCESS B: MEDIAPIPE HANDS ---
            # Convert raw frame to RGB for MediaPipe
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB))
            
            # Send to Async Detector
            frame_timestamp_ms = int(time.time() * 1000)
            landmarker.detect_async(mp_image, frame_timestamp_ms)

            # Check for results and Draw on DISPLAY image
            if latest_mp_result:
                # 1. Initialize Default Values (Reset every frame)
                left_grasp = False   # <--- ADD THIS
                right_grasp = False  # <--- ADD THIS

                # Draw skeleton
                img_display = draw_hand_landmarks(img_display, latest_mp_result)
                
                # Count fingers
                left_count, right_count = count_fingers(latest_mp_result)

                if left_count is not None and left_count == 0:
                    left_grasp = True
                
                if right_count is not None and right_count == 0: 
                    right_grasp = True

                # --- 2. SEND SIGNAL TO ARDUINO ---
                if arduino:
                    # LEFT HAND LOGIC
                    if left_grasp != prev_left_grasp:
                        try:
                            if left_grasp:
                                print("Sending: Left Close")
                                arduino.write(b"LC\n")
                            else:
                                print("Sending: Left Open")
                                arduino.write(b"LO\n")
                            
                            # Give Arduino time to process!
                            time.sleep(0.05) 
                            
                        except serial.SerialTimeoutException:
                            print("Warning: Arduino write timed out. Skipping.")
                        
                        prev_left_grasp = left_grasp

                    # RIGHT HAND LOGIC
                    if right_grasp != prev_right_grasp:
                        try:
                            if right_grasp:
                                print("Sending: Right Close")
                                arduino.write(b"RC\n")
                            else:
                                print("Sending: Right Open")
                                arduino.write(b"RO\n")
                                
                            # Give Arduino time to process!
                            time.sleep(0.05) 

                        except serial.SerialTimeoutException:
                            print("Warning: Arduino write timed out. Skipping.")
                            
                        prev_right_grasp = right_grasp
                
                # 3. Display Hand Counts & Grasp Status
                # Note: I adjusted the coordinates so the text doesn't overlap
                
                # LEFT HAND INFO (Top Left)
                cv2.putText(img_display, f"Left Count: {left_count}", (10, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                
                # Show Grasp status below the count
                color_left = (0, 255, 0) if left_grasp else (0, 0, 255) # Green if Grasping
                cv2.putText(img_display, f"L Grasp: {left_grasp}", (10, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, color_left, 2)
                
                # RIGHT HAND INFO (Top Right side)
                width = img_display.shape[1]
                cv2.putText(img_display, f"Right Count: {right_count}", (width - 350, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                color_right = (0, 255, 0) if right_grasp else (0, 0, 255)
                cv2.putText(img_display, f"R Grasp: {right_grasp}", (width - 350, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, color_right, 2)


            # 4. SHOW FINAL IMAGE
            cv2.imshow("YOLO + Hand Tracking", img_display)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()