#!/usr/bin/env python3
# compute_homography.py
# Calcule et sauvegarde la matrice d'homographie
# À exécuter une seule fois après avoir collecté les coordonnées réelles

import cv2
import numpy as np

# --- À REMPLIR : coordonnées (x, y) réelles depuis MoveIt ---
# Ordre : marqueur 0, marqueur 1, marqueur 2, marqueur 3
# Unité : mètres (ce que MoveIt retourne)

real_coords = np.array([
    [0.0, 0.0],   # Marqueur 0 — remplacer par (x, y) réel
    [0.0, 0.0],   # Marqueur 1 — remplacer par (x, y) réel
    [0.0, 0.0],   # Marqueur 2 — remplacer par (x, y) réel
    [0.0, 0.0],   # Marqueur 3 — remplacer par (x, y) réel
], dtype=np.float32)

# --- Charger les pixels sauvegardés par aruco_detector.py ---
try:
    pixels = np.load('pixels_aruco.npy')
    print("Pixels chargés :")
    for i, p in enumerate(pixels):
        print(f"  Marqueur {i} : pixel {p}")
except FileNotFoundError:
    print("Erreur : pixels_aruco.npy introuvable.")
    print("Lance d'abord aruco_detector.py et appuie sur 's' pour sauvegarder.")
    exit(1)

# --- Calculer l'homographie ---
H, status = cv2.findHomography(pixels, real_coords)

if H is None:
    print("Erreur : impossible de calculer l'homographie.")
    exit(1)

print("\nHomographie calculée avec succès.")
print("Matrice H :")
print(H)

# --- Sauvegarder ---
np.save('homography.npy', H)
print("\nHomographie sauvegardée dans homography.npy")

# --- Test rapide ---
print("\nTest de validation :")
for i in range(4):
    pt = np.array([[[pixels[i][0], pixels[i][1]]]], dtype=np.float32)
    result = cv2.perspectiveTransform(pt, H)
    pred = result[0][0]
    real = real_coords[i]
    erreur = np.linalg.norm(pred - real) * 100  # en cm
    print(f"  Marqueur {i} : prédit {pred} | réel {real} | erreur {erreur:.2f} cm")
