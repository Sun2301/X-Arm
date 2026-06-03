#!/usr/bin/env python3
"""
detect_pixel2real.py — Détection YOLO + HSV + Homographie
Suit exactement le README: Étapes 1-4

Flux: YOLO → Crop BB → HSV (face du dessus) → Centroïde → Homographie → Coords réelles
"""

import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path
from Arm_Lib import Arm_Device

dofbot=Arm_Device()
dofbot.Arm_serial_servo_write6(92,107,5,5,90,30,500)

# ==========================================
# ⚙️ CONFIGURATION À MODIFIER
# ==========================================

# 1️⃣ INDEX DE CAMÉRA
CAMERA_INDEX = 1  # Modifie si besoin (0, 1, 2...)

# 2️⃣ BORNES HSV (à obtenir via test_HSV calibrate())
# Lance d'abord: python3 test_HSV, puis calibrate()
# Copie les valeurs obtenues ici

HSV_LOWER = np.array([89, 146, 0])
HSV_UPPER = np.array([99, 255, 255])
# À calibrer!

# ==========================================
# ÉTAPE 2 — Fonction de centroïde précis
# ==========================================

def get_top_face_centroid(frame, x1, y1, x2, y2):
    """
    Étape 2 du README: Segmente la face du dessus du cube.
    Retourne le centroïde précis (cx, cy) dans l'image complète.
    """
    # 1. Crop la bounding box YOLO
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None

    # 2. Segmentation HSV (face du dessus = plus claire)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    # 3. Nettoyage morphologique
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 4. Contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    cnt = max(contours, key=cv2.contourArea)

    # 5. Centroïde
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None

    # Remettre dans les coordonnées de l'image complète
    cx = x1 + int(M["m10"] / M["m00"])
    cy = y1 + int(M["m01"] / M["m00"])
    return cx, cy


def pixel_to_real(cx, cy, H):
    """
    Étape 4 du README: Applique l'homographie.
    Convertit les coordonnées pixel (cx, cy) en coordonnées réelles (x_reel, y_reel).
    """
    point_pixel = np.array([[cx, cy]], dtype=np.float32).reshape(-1, 1, 2)
    point_reel = cv2.perspectiveTransform(point_pixel, H)
    x_reel, y_reel = point_reel[0][0]
    return x_reel, y_reel


# ==========================================
# BOUCLE PRINCIPALE
# ==========================================

def main():
    # --- Charger le modèle YOLO ---
    model_path = Path("/home/pi/X-Arm/computer-vision/Rom/models/best.pt")
    if not model_path.exists():
        print(f"❌ Modèle YOLO introuvable: {model_path}")
        print("   Télécharge best.pt depuis Colab et place-le dans Rom/models/")
        exit(1)
    
    print(f"✓ Modèle YOLO chargé: {model_path}")
    model = YOLO(str(model_path))
    model.conf = 0.5
    
    # --- Charger l'homographie ---
    try:
        H = np.load("homography.npy")
        print("✓ Homographie chargée: homography.npy")
    except FileNotFoundError:
        print("❌ Homographie introuvable: homography.npy")
        print("   Lance compute_homography.py avec les marqueurs ArUco")
        exit(1)
    
    # --- Initialiser la caméra ---
    print(f"\n🎥 Ouverture de la caméra (index {CAMERA_INDEX})...")
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    if not cap.isOpened():
        print(f"❌ Impossible d'accéder à la caméra (index {CAMERA_INDEX})!")
        print("   Modifie CAMERA_INDEX en haut du fichier (essaie 0, 1, 2...)")
        exit(1)
    
    print("✓ Caméra ouverte")
    print("\n📋 Contrôles:")
    print("   - Appuye 'q' pour quitter")
    print("   - Appuye 's' pour sauvegarder une capture")
    print("=" * 70)
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Erreur de flux vidéo")
            break
        
        frame_count += 1
        h, w = frame.shape[:2]
        
        # --- ÉTAPE 1 : DÉTECTION YOLO ---
        results = model(frame, verbose=False)
        
        # Mapping des classes
        class_names = {0: "Blue", 1: "Green", 2: "Red"}
        
        # --- ÉTAPE 2-4 : Pour chaque cube détecté ---
        for result in results:
            boxes = result.boxes
            
            for box in boxes:
                # Coordonnées YOLO
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = class_names.get(cls_id, "Unknown")
                
                # Dessiner la BB YOLO (rouge)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f"{cls_name} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # --- ÉTAPE 2 : Segmentation HSV ---
                centroid = get_top_face_centroid(frame, x1, y1, x2, y2)
                
                if centroid:
                    cx, cy = centroid
                    
                    # Dessiner le centroïde HSV (vert)
                    cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                    cv2.putText(frame, f"HSV ({cx},{cy})", (cx + 15, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # --- ÉTAPE 4 : Homographie ---
                    x_real, y_real = pixel_to_real(cx, cy, H)
                    
                    # Afficher coords réelles
                    cv2.putText(frame, f"Reel: ({x_real:.4f}, {y_real:.4f}) m", 
                                (cx + 15, cy + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    print(f"[Frame {frame_count}] {cls_name}: pixel ({cx}, {cy}) -> reel ({x_real:.4f}, {y_real:.4f}) m")
                
                else:
                    # Fallback (si HSV ne trouve rien)
                    fallback_cx = int((x1 + x2) / 2)
                    fallback_cy = int(y1 + (y2 - y1) / 3)
                    
                    cv2.circle(frame, (fallback_cx, fallback_cy), 8, (0, 165, 255), -1)
                    cv2.putText(frame, "Fallback", (fallback_cx + 15, fallback_cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
                    
                    x_real, y_real = pixel_to_real(fallback_cx, fallback_cy, H)
                    print(f"[Frame {frame_count}] {cls_name} (FALLBACK): pixel ({fallback_cx}, {fallback_cy}) -> reel ({x_real:.4f}, {y_real:.4f}) m")
        
        # Affichage
        cv2.imshow("YOLO + HSV + Homographie", frame)
        
        # Contrôles
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("\n✓ Arrêt...")
            break
        elif key == ord('s'):
            filename = f"capture_{frame_count}.png"
            cv2.imwrite(filename, frame)
            print(f"✓ Capture sauvegardée: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    print("✓ Fin du programme")


if __name__ == "__main__":
    main()
