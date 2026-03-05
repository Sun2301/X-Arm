#!/usr/bin/env python3
# test_pixel2real.py
# Test d'intégration : clique sur l'image pour convertir pixel → coordonnées réelles

import cv2
import numpy as np

CAMERA_INDEX = 1

# --- Charger l'homographie ---
try:
    H = np.load('homography.npy')
    print("Homographie chargée.")
except FileNotFoundError:
    print("Erreur : homography.npy introuvable.")
    print("Lance d'abord compute_homography.py.")
    exit(1)

# --- Fonction de conversion ---
def pixel2real(u, v, H):
    pt = np.array([[[u, v]]], dtype=np.float32)
    result = cv2.perspectiveTransform(pt, H)
    return result[0][0]

# --- Caméra ---
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
if not cap.isOpened():
    print(f"Erreur : impossible d'ouvrir la caméra (index {CAMERA_INDEX})")
    exit(1)

print("Clique sur l'image pour voir la conversion pixel → réel.")
print("Appuie sur 'q' pour quitter.")

click_pos = [None]

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        click_pos[0] = (x, y)

cv2.namedWindow("pixel2real")
cv2.setMouseCallback("pixel2real", on_mouse)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_display = frame.copy()

    if click_pos[0] is not None:
        u, v = click_pos[0]
        rx, ry = pixel2real(u, v, H)

        cv2.circle(frame_display, (u, v), 6, (0, 255, 0), -1)
        cv2.putText(frame_display,
                    f"pixel ({u}, {v})",
                    (u + 10, v - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame_display,
                    f"reel ({rx:.3f}, {ry:.3f}) m",
                    (u + 10, v),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        print(f"pixel ({u}, {v}) → réel ({rx:.4f}, {ry:.4f}) mètres")

    cv2.imshow("pixel2real", frame_display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Fin.")
