import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2
import time

# --- Configuration ---
model_path = './hand_landmarker.task'

# --- 1. Custom Drawing Function (Uses OpenCV instead of MediaPipe Utils) ---
# This avoids the "mediapipe.framework" import error completely.
def draw_landmarks_on_image(rgb_image, detection_result):
    annotated_image = rgb_image.copy()
    
    # These are the standard connections between finger joints
    HAND_CONNECTIONS = [
        (0, 1), (1, 2), (2, 3), (3, 4),   # Thumb
        (0, 5), (5, 6), (6, 7), (7, 8),   # Index
        (5, 9), (9, 10), (10, 11), (11, 12), # Middle
        (9, 13), (13, 14), (14, 15), (15, 16), # Ring
        (13, 17), (17, 18), (18, 19), (19, 20), # Pinky
        (0, 17) # Palm base to Pinky base
    ]

    # Check if any hands were detected
    if detection_result.hand_landmarks:
        # Loop through each detected hand
        for hand_landmarks in detection_result.hand_landmarks:
            
            # Get image dimensions to convert normalized coordinates to pixels
            height, width, _ = annotated_image.shape
            
            # Store pixel coordinates of all 21 points
            points = []
            for landmark in hand_landmarks:
                x = int(landmark.x * width)
                y = int(landmark.y * height)
                points.append((x, y))

            # Draw Lines (Connections)
            for connection in HAND_CONNECTIONS:
                start_idx = connection[0]
                end_idx = connection[1]
                cv2.line(annotated_image, points[start_idx], points[end_idx], (0, 255, 0), 2)

            # Draw Points (Joints)
            for point in points:
                cv2.circle(annotated_image, point, 4, (0, 0, 255), -1) # Red dots

    return annotated_image

# --- Global variable to store results ---
latest_result = None

# --- Callback Function ---
def save_result(result, output_image: mp.Image, timestamp_ms: int):
    global latest_result
    latest_result = result

def count_fingers(detection_result):
    # Tips of fingers: Index(8), Middle(12), Ring(16), Pinky(20)
    # We check if the TIP is higher (smaller Y value) than the PIP joint (lower knuckle)
    tips = [8, 12, 16, 20]
    pips = [6, 10, 14, 18]
    
    fingers_up = 0
    
    # Check if hand is detected
    if detection_result.hand_landmarks:
        hand = detection_result.hand_landmarks[0] # Get first hand
        
        # 1. Check Thumb (Logic is different: check X for left/right movement)
        # Assuming right hand facing camera: Tip(4) should be to the left of IP(3)
        if hand[4].x < hand[3].x: 
            fingers_up += 1
            
        # 2. Check other 4 fingers (Check Y for up/down movement)
        for i in range(len(tips)):
            if hand[tips[i]].y < hand[pips[i]].y: # Remember: Y decreases as you go UP
                fingers_up += 1
                
    return fingers_up

# --- Main Execution ---
def main():
    base_options = python.BaseOptions(model_asset_path=model_path)
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.LIVE_STREAM,
        num_hands=2,
        result_callback=save_result)

    cap = cv2.VideoCapture(0)

    # Start the Landmarker
    with vision.HandLandmarker.create_from_options(options) as landmarker:
        print("Camera started. Press 'q' to exit.")
        
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                continue

            # Convert BGR to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            
            # Timestamp required for LIVE_STREAM mode
            frame_timestamp_ms = int(time.time() * 1000)

            # Detect Async
            landmarker.detect_async(mp_image, frame_timestamp_ms)

            # Draw latest landmarks if available
            if latest_result:
                frame = draw_landmarks_on_image(frame, latest_result)

                # Count fingers and print to console
                count = count_fingers(latest_result)
                print(f"Fingers up: {count}")
                
                # Simple Logic for Robot Arm
                if count == 0:
                    cv2.putText(frame, "GRAB (Fist)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                elif count == 5:
                    cv2.putText(frame, "RELEASE (Open)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('Hand Tracking (Custom Draw)', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()