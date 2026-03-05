#!/usr/bin/env python3
# aruco_detector.py
# Test de détection ArUco — affiche les coordonnées pixels des marqueurs

import cv2
import numpy as np

# --- Configuration ---
CAMERA_INDEX = 1          # Changer si nécessaire
ARUCO_DICT   = cv2.aruco.DICT_4X4_50
MARKER_IDS   = [0, 1, 2, 3]

# --- Initialisation ---
aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters()
detector     = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
if not cap.isOpened():
    print(f"Erreur : impossible d'ouvrir la caméra (index {CAMERA_INDEX})")
    exit(1)

print("Caméra ouverte. Appuie sur 'q' pour quitter, 's' pour sauvegarder les coordonnées.")
print("-" * 60)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erreur : impossible de lire l'image")
        break

    # Détection des marqueurs
    corners, ids, rejected = detector.detectMarkers(frame)

    # Dessiner les marqueurs détectés
    frame_display = frame.copy()

    detected = {}

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame_display, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in MARKER_IDS:
                # Centre du marqueur = moyenne des 4 coins
                center = corners[i][0].mean(axis=0)
                cx, cy = int(center[0]), int(center[1])
                detected[int(marker_id)] = (cx, cy)

                # Afficher le centre et l'ID sur l'image
                cv2.circle(frame_display, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(frame_display, f"ID {marker_id}: ({cx}, {cy})",
                            (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Afficher le statut dans le terminal
    print(f"\rDétectés : {sorted(detected.keys())} / {MARKER_IDS}", end="")

    # Vérifier si les 4 marqueurs sont tous détectés
    if all(m in detected for m in MARKER_IDS):
        cv2.putText(frame_display, "4/4 MARQUEURS OK", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        print(f"\r4/4 marqueurs détectés — coordonnées pixels :          ")
        for mid in sorted(detected.keys()):
            print(f"  Marqueur {mid} : pixel {detected[mid]}")
    else:
        manquants = [m for m in MARKER_IDS if m not in detected]
        cv2.putText(frame_display, f"Marqueurs manquants : {manquants}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("ArUco Detection", frame_display)

    key = cv2.waitKey(1) & 0xFF

    # Sauvegarder les coordonnées pixels si 4 marqueurs détectés
    if key == ord('s'):
        if all(m in detected for m in MARKER_IDS):
            pixels = np.array([detected[m] for m in sorted(detected.keys())],
                               dtype=np.float32)
            np.save('pixels_aruco.npy', pixels)
            print("\nCoordonnées pixels sauvegardées dans pixels_aruco.npy")
            print("Contenu :")
            for i, mid in enumerate(sorted(detected.keys())):
                print(f"  Marqueur {mid} : {pixels[i]}")
        else:
            print("\nImpossible de sauvegarder — tous les marqueurs ne sont pas détectés")

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("\nFin.")
