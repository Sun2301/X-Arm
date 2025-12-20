#!/usr/bin/env python3
"""
Détection en temps réel avec YOLO et webcam
Affiche les détections en direct dans une fenêtre
"""

import cv2
import torch
from ultralytics import YOLO
from pathlib import Path
import warnings

# Désactiver les avertissements
warnings.filterwarnings('ignore')

# Charger le modèle
model_path = Path(__file__).parent / "models" / "best.pt"

if not model_path.exists():
    print(f" Modèle non trouvé: {model_path}")
    exit(1)

print(f" Modèle chargé: {model_path}")
model = YOLO(str(model_path))

# Configuration YOLO
model.conf = 0.5
model.verbose = False

# Mapping des classes avec couleurs
class_names = {0: "Blue", 1: "Green", 2: "Red"}
class_colors = {0: (255, 0, 0), 1: (0, 255, 0), 2: (0, 0, 255)}  # BGR

# Initialiser la webcam
print("\n Ouverture de la webcam...")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print(" Impossible d'accéder à la webcam!")
    exit(1)

# Configuration webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

print(" Webcam ouverte")
print("\n Contrôles:")
print("  - 'q' = quitter")
print("  - 's' = sauvegarder une capture")
print("\n" + "="*50 + "\n")

frame_count = 0
save_dir = Path(__file__).parent / "webcam_results"
save_dir.mkdir(exist_ok=True)

try:
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print(" Erreur de lecture de la webcam")
            break
        
        frame_count += 1
        
        # Inférence
        with torch.no_grad():
            results = model(frame, verbose=False, conf=0.5)
        
        boxes_count = 0
        
        # Traiter les résultats
        if results and len(results) > 0:
            result = results[0]
            
            if result.boxes is not None and len(result.boxes) > 0:
                boxes = result.boxes
                boxes_count = len(boxes)
                
                for box in boxes:
                    # Coordonnées
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = class_names.get(class_id, "Unknown")
                    color = class_colors.get(class_id, (0, 255, 255))
                    
                    # Dessiner la boîte
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
                    # Label
                    label = f"{class_name} {conf:.2f}"
                    label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    
                    # Fond du texte
                    cv2.rectangle(
                        frame,
                        (x1, y1 - label_size[1] - 5),
                        (x1 + label_size[0], y1),
                        color,
                        -1
                    )
                    
                    # Texte
                    cv2.putText(
                        frame,
                        label,
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2
                    )
        
        # Statistiques
        stats_text = f"Frame: {frame_count} | Detections: {boxes_count}"
        cv2.putText(
            frame,
            stats_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        # Afficher
        cv2.imshow("YOLO Detection - Blue/Green/Red (q=quit, s=save)", frame)
        
        # Gestion des touches
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\n Arrêt...")
            break
        elif key == ord('s'):
            filename = save_dir / f"capture_{frame_count}.jpg"
            cv2.imwrite(str(filename), frame)
            print(f" Capture sauvegardée: {filename}")

except KeyboardInterrupt:
    print("\n  Interruption utilisateur")
finally:
    cap.release()
    cv2.destroyAllWindows()
    print(f" Webcam fermée")
    print(f"Captures sauvegardées dans: {save_dir}")
