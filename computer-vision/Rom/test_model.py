#!/usr/bin/env python3
"""
Script pour tester le modèle YOLO entraîné
"""

from ultralytics import YOLO
from pathlib import Path
import cv2

# Charger le modèle entraîné
model_path = Path(__file__).parent / "models" / "best.pt"

if not model_path.exists():
    print(f"Modèle non trouvé: {model_path}")
    print("Télécharge best.pt depuis Colab et place-le dans le dossier 'models/'")
    exit(1)

print(f"Modèle chargé: {model_path}")
model = YOLO(str(model_path))

# Test 1: Sur une image unique
print("\n=== Test sur une image ===")
test_image = "yolo_dataset/images/test"  # Dossier de test

results = model.predict(
    source=test_image,
    save=True,
    conf=0.5,
    device='cpu'  # CPU par défaut
)

print(f" Résultats sauvegardés dans runs/detect/predict")

# Test 2: Afficher les statistiques
print("\n=== Métriques du modèle ===")
metrics = model.val(
    data="yolo_dataset/data.yaml",
    split="test",
    device='cpu'
)

print(f"mAP50: {metrics.box.map50:.3f}")
print(f"mAP50-95: {metrics.box.map:.3f}")

# Test 3: Sur une webcam (optionnel)
print("\n=== Prédiction en temps réel (webcam) ===")
print("Appuie sur 'q' pour quitter")

# Décommenter pour webcam:
# results = model.predict(source=0, save=False, conf=0.5)
