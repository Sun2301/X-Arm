#!/usr/bin/env python3
"""
Détection en temps réel avec YOLO et webcam
Enregistre les détections dans un fichier vidéo
"""

import cv2
import torch
from ultralytics import YOLO
from pathlib import Path
import numpy as np
import warnings
import os

# Désactiver l'affichage Qt
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

# Désactiver les avertissements
warnings.filterwarnings('ignore')

# Charger le modèle
model_path = Path(__file__).parent / "models" / "best.pt"

if not model_path.exists():
    print(f" Modèle non trouvé: {model_path}")
    exit(1)

print(f"✓ Modèle chargé: {model_path}")
model = YOLO(str(model_path))

# Désactiver les plots YOLO
model.conf = 0.5
model.verbose = False
model.max_det = 300

# Mapping des classes avec couleurs
class_names = {0: "Blue", 1: "Green", 2: "Red"}
class_colors = {0: (255, 0, 0), 1: (0, 255, 0), 2: (0, 0, 255)}  # BGR format

# Initialiser la webcam
print("\n🎥 Ouverture de la webcam...")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ Impossible d'accéder à la webcam!")
    exit(1)

# Configuration webcam
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Configuration du writer vidéo
output_path = Path(__file__).parent / "webcam_results" / "detection_output.mp4"
output_path.parent.mkdir(exist_ok=True)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 30
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

writer = cv2.VideoWriter(str(output_path), fourcc, fps, (frame_width, frame_height))

print("✓ Webcam ouverte")
print(f"📹 Enregistrement dans: {output_path}")
print("\nAppuie sur Ctrl+C pour arrêter l'enregistrement...")
print("="*50)

frame_count = 0
detections_count = 0

try:
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("❌ Erreur de lecture de la webcam")
            break
        
        frame_count += 1
        
        # Inférence directe
        with torch.no_grad():
            results = model(frame, verbose=False, conf=0.5)
        
        boxes_count = 0
        
        # Traiter les résultats
        if results and len(results) > 0:
            result = results[0]
            
            # Obtenir les boîtes
            if result.boxes is not None and len(result.boxes) > 0:
                boxes = result.boxes
                boxes_count = len(boxes)
                detections_count += boxes_count
                
                for box in boxes:
                    # Coordonnées
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = class_names.get(class_id, "Unknown")
                    color = class_colors.get(class_id, (0, 255, 255))
                    
                    # Dessiner la boîte
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    
                    # Texte avec label et confiance
                    label = f"{class_name} {conf:.2f}"
                    label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    
                    # Fond pour le texte
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
        
        # Afficher les stats
        stats_text = f"Frame: {frame_count} | Détections: {boxes_count}"
        cv2.putText(
            frame,
            stats_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )
        
        # Écrire la frame dans la vidéo
        writer.write(frame)
        
        # Afficher la progression
        if frame_count % 30 == 0:
            print(f"✓ Frames traitées: {frame_count} | Total détections: {detections_count}")

except KeyboardInterrupt:
    print("\n⚠️  Arrêt demandé par l'utilisateur...")
finally:
    cap.release()
    writer.release()
    print(f"\n✓ Webcam fermée")
    print(f"✓ Vidéo sauvegardée: {output_path}")
    print(f"📊 Statistiques:")
    print(f"   - Frames traitées: {frame_count}")
    print(f"   - Total détections: {detections_count}")
