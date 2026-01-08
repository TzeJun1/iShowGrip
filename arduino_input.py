import serial
import time
import cv2
import mediapipe as mp

# Setup Serial Communication (Check your COM port in Arduino IDE)
# Change 'COM3' to your actual port (e.g., '/dev/ttyUSB0' on Linux)
try:
    esp32 = serial.Serial(port='COM5', baudrate=115200, timeout=0.1)
except:
    print("Could not connect to ESP32. Check COM port.")

def send_data(servo_count, angle):
    """Sends data in format 'count,angle\n' (e.g., '3,90')"""
    data = f"{servo_count},{angle}\n"
    esp32.write(data.encode())
    print(f"Sent: {data.strip()}")

# --- MODE 1: Manual Input ---
print("Mode 1: Manual Input")
val = int(input("Enter angle (0-90): "))
servos = int(input("How many servos to move (1-3)? "))
send_data(servos, val)

time.sleep(2)

# --- MODE 2: Computer Vision (Hand Tracking) ---
print("Mode 2: Hand Gesture Control. Press 'q' to quit.")
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, img = cap.read()
    if not success: break

    img = cv2.flip(img, 1)
    results = hands.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    
    finger_count = 0

    if results.multi_hand_landmarks:
        for hand_lms in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, hand_lms, mp_hands.HAND_CONNECTIONS)
            
            # Simple finger counting logic (checking if tips are above knuckles)
            tips = [8, 12, 16, 20] # Index, Middle, Ring, Pinky
            for tip in tips:
                if hand_lms.landmark[tip].y < hand_lms.landmark[tip-2].y:
                    finger_count += 1
            # Thumb (different logic for horizontal movement)
            if hand_lms.landmark[4].x > hand_lms.landmark[3].x:
                finger_count += 1

        # Logic: Rotate all servos to 90 degrees based on finger count
        # If 2 fingers are up, 2 servos move.
        send_data(finger_count, 90)

    cv2.putText(img, f"Fingers: {finger_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
    cv2.imshow("Hand Tracker", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()