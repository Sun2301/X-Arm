#!/usr/bin/env python3
# detect_pixel2real.py
# Detection YOLO + conversion pixel -> coordonnees reelles via homographie

import cv2
import numpy as np
from ultralytics import YOLO

# --- Configuration ---
CAMERA_INDEX = 1
MODEL_PATH = "Rom/models/best.pt"
CONFIDENCE_THRESHOLD = 0.5

# --- Chargement homographie ---
try:
    H = np.load('homography.npy')
    print("Homographie chargee.")
except FileNotFoundError:
    print("Erreur : homography.npy introuvable.")
    print("Lance d'abord compute_homography.py.")
    exit(1)

# --- Chargement modele YOLO ---
try:
    model = YOLO(MODEL_PATH)
    print("Modele YOLO charge.")
except Exception as e:
    print("Erreur : impossible de charger le modele YOLO : " + str(e))
    exit(1)

# --- Conversion pixel -> reel ---
def pixel2real(u, v, H):
    pt = np.array([[[u, v]]], dtype=np.float32)
    result = cv2.perspectiveTransform(pt, H)
    return result[0][0]

# --- Camera ---
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Erreur : impossible d'ouvrir la camera (index " + str(CAMERA_INDEX) + ")")
    exit(1)

print("Detection en cours. Appuie sur 'q' pour quitter.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_display = frame.copy()

    # --- Inference YOLO ---
    results = model(frame, verbose=False)

    for result in results:
        for box in result.boxes:
            confidence = float(box.conf[0])
            if confidence < CONFIDENCE_THRESHOLD:
                continue

            # Bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            class_id = int(box.cls[0])
            class_name = model.names[class_id]

            # Centre de la bounding box
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # Conversion pixel -> reel
            rx, ry = pixel2real(cx, cy, H)

            # Affichage
            cv2.rectangle(frame_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame_display, (cx, cy), 5, (0, 255, 0), -1)

            label_class = class_name + " " + str(round(confidence, 2))
            label_pixel = "px (" + str(cx) + ", " + str(cy) + ")"
            label_real = "reel (" + str(round(rx, 3)) + ", " + str(round(ry, 3)) + ") m"

            cv2.putText(frame_display, label_class,
                        (x1, y1 - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame_display, label_pixel,
                        (x1, y1 - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(frame_display, label_real,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            print("Detecte : " + class_name + " | px (" + str(cx) + ", " + str(cy) + ") -> reel (" + str(round(rx, 4)) + ", " + str(round(ry, 4)) + ") m")

    cv2.imshow("detect_pixel2real", frame_display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Fin.")