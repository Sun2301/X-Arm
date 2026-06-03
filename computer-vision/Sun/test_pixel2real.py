import cv2
import numpy as np

# ==========================================
# PARAMÈTRES DE CONFIGURATION
# ==========================================

# 1. Bornes HSV pour la FACE SUPÉRIEURE du cube (à ajuster avec ton script de calibration)
# Rappel : ces valeurs doivent cibler la face claire, pas l'ombre ni les côtés.
HSV_LOWER = np.array([85, 80, 150])
HSV_UPPER = np.array([110, 200, 255])

# 2. Ta matrice d'homographie (Remplace ce tableau par ta vraie matrice H calculée via ArUco)
# H = np.array([[...], [...], [...]])

# ==========================================
# FONCTIONS DE TRAITEMENT
# ==========================================

def get_top_face_centroid(frame, x1, y1, x2, y2, lower_hsv, upper_hsv):
    """
    Isole la face supérieure du cube dans la bounding box YOLO
    et retourne le centroïde exact de cette face dans l'image principale.
    """
    # Sécurité : s'assurer que les coordonnées ne sortent pas de l'image
    h, w = frame.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)

    # 1. Découper (crop) la zone de la bounding box YOLO
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None

    # 2. Convertir le crop en HSV et créer le masque
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Nettoyage morphologique pour enlever le bruit
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 3. Trouver les contours sur le masque
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Garder le plus grand contour (la face supérieure)
    cnt = max(contours, key=cv2.contourArea)

    # 4. Calculer le centroïde via les moments mathématiques
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None

    # 5. Translater les coordonnées du crop vers les coordonnées de l'image complète
    cx = x1 + int(M["m10"] / M["m00"])
    cy = y1 + int(M["m01"] / M["m00"])

    return cx, cy

def pixel_to_real(cx, cy, H):
    """
    Applique l'homographie pour convertir un pixel (cx, cy) en coordonnées robot réelles (X, Y).
    """
    point_2d = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
    point_3d = cv2.perspectiveTransform(point_2d, H)
    return point_3d[0][0][0], point_3d[0][0][1]

# ==========================================
# BOUCLE PRINCIPALE (detect_pixel2real.py)
# ==========================================

def main():
    cap = cv2.VideoCapture(0)

    # Charger le modèle YOLO ici si nécessaire
    # model = load_yolo_model(...)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erreur de flux vidéo.")
            break

        # ----------------------------------------------------
        # ÉTAPE 1 : INSTRUCTION YOLO
        # Remplace cette partie par ton inférence YOLO actuelle
        # results = model(frame)
        # bboxes = results.get_boxes() # format [[x1, y1, x2, y2], ...]
        # ----------------------------------------------------
        
        # Bounding box fictive pour que le code tourne sans YOLO si tu testes
        bboxes = [] 

        for bbox in bboxes:
            # Récupérer les coordonnées de la bounding box YOLO
            x1, y1, x2, y2 = map(int, bbox[:4])

            # Dessiner la boîte YOLO d'origine en rouge (pour voir le décalage)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

            # ----------------------------------------------------
            # ÉTAPE 2 : CORRECTION HSV DU CENTROÏDE
            # ----------------------------------------------------
            centroid = get_top_face_centroid(frame, x1, y1, x2, y2, HSV_LOWER, HSV_UPPER)

            if centroid:
                cx, cy = centroid

                # Dessiner le NOUVEAU centroïde parfait en vert vif
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"Centre HSV ({cx},{cy})", (cx + 10, cy), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # ----------------------------------------------------
                # ÉTAPE 3 : HOMOGRAPHIE (Décommenter quand H est défini)
                # ----------------------------------------------------
                # real_x, real_y = pixel_to_real(cx, cy, H)
                # print(f"px ({cx}, {cy}) -> reel ({real_x:.4f}, {real_y:.4f}) m")
                
                # C'est ici que tu envoies (real_x, real_y) au X-Arm
                
            else:
                # FALLBACK de sécurité : Si l'éclairage foire et que HSV ne trouve rien,
                # on utilise ton astuce empirique (le tiers supérieur de la boîte)
                fallback_cx = int((x1 + x2) / 2)
                fallback_cy = int(y1 + (y2 - y1) / 3)
                
                cv2.circle(frame, (fallback_cx, fallback_cy), 5, (0, 165, 255), -1) # Orange
                cv2.putText(frame, "Fallback YOLO", (fallback_cx + 10, fallback_cy), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

        # Affichage du résultat
        cv2.imshow("Detection Hybride: YOLO + Segmentation HSV", frame)

        # Quitter avec 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()







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