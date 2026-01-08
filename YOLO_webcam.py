from ultralytics import YOLO
import cv2
import cvzone
import math
import torch

# Check for GPU
print(f"Is CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA device: {torch.cuda.get_device_name(0)}")
else:
    print("CUDA is not available. Running on CPU.")

model = YOLO("./YOLO-weights/yolo12n.pt")

# Webcam
# cap = cv2.VideoCapture(0)
# cap.set(3, 1240)
# cap.set(4, 720)

# Video
cap = cv2.VideoCapture("./Videos/bikes.mp4")

while True:
    success, img = cap.read()
    results = model(img, stream=True)
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # Bounding Box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

            # Confidence
            conf = math.ceil((box.conf[0] * 100)) / 100
            cls = int(box.cls[0])
            cvzone.putTextRect(img, f'{model.names[cls]} {conf}', (max(0, x1), max(35, y1)), scale=1,
                               thickness=1, offset=3)
            
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break