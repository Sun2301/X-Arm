# ========================================
# ENTRAÎNEMENT YOLO X-ARM SUR GOOGLE COLAB
# ========================================
# 
# Instructions:
# 1. Ouvrir dans Google Colab: https://colab.research.google.com
# 2. Runtime > Change runtime type > GPU (T4)
# 3. Exécuter les cellules une par une
#

# ========== CELLULE 1: Vérifier le GPU ==========
import torch
print(f"GPU disponible: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU: {torch.cuda.get_device_name(0)}")
else:
    print("Activer le GPU: Runtime > Change runtime type > GPU")

# ========== CELLULE 2: Installer Ultralytics YOLO ==========
!pip install ultralytics

# ========== CELLULE 3: Uploader le dataset ==========
# Option A: Depuis Google Drive
from google.colab import drive
drive.mount('/content/drive')
# Puis compresser ton dossier yolo_dataset en .zip et le mettre dans Drive

# Option B: Upload direct (si petit dataset < 100MB)
from google.colab import files
import zipfile

print("Upload ton fichier yolo_dataset.zip")
uploaded = files.upload()

# Décompresser
for filename in uploaded.keys():
    if filename.endswith('.zip'):
        with zipfile.ZipFile(filename, 'r') as zip_ref:
            zip_ref.extractall('.')
        print(f"Dataset extrait: {filename}")

# ========== CELLULE 4: Vérifier la structure du dataset ==========
!ls -la yolo_dataset/
!cat yolo_dataset/data.yaml

# ========== CELLULE 5: Corriger les chemins dans data.yaml ==========
# Le chemin absolu ne marchera pas sur Colab, on va le corriger
import yaml

with open('yolo_dataset/data.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Mettre le chemin relatif
config['path'] = '/content/yolo_dataset'

with open('yolo_dataset/data.yaml', 'w') as f:
    yaml.dump(config, f, default_flow_style=False)

print("✓ data.yaml mis à jour pour Colab")
!cat yolo_dataset/data.yaml

# ========== CELLULE 6: ENTRAÎNEMENT ==========
from ultralytics import YOLO

# Créer le modèle (yolov8n = nano, rapide)
model = YOLO('yolov8n.pt')

# Entraîner
results = model.train(
    data='yolo_dataset/data.yaml',
    epochs=100,              # Nombre d'epochs
    imgsz=640,               # Taille des images
    batch=16,                # Batch size (augmente si GPU puissant)
    device=0,                # GPU 0
    project='runs/xarm',     # Dossier des résultats
    name='train_blue_green_red',
    patience=20,             # Early stopping
    save=True,
    plots=True
)

# ========== CELLULE 7: Validation sur test set ==========
# Tester le meilleur modèle
metrics = model.val(
    data='yolo_dataset/data.yaml',
    split='test'
)

print(f"\n📊 Résultats sur le test set:")
print(f"  - mAP50: {metrics.box.map50:.3f}")
print(f"  - mAP50-95: {metrics.box.map:.3f}")

# ========== CELLULE 8: Télécharger le modèle entraîné ==========
from google.colab import files
import shutil

# Compresser les résultats
shutil.make_archive('model_trained', 'zip', 'runs/xarm/train_blue_green_red')

# Télécharger
files.download('model_trained.zip')
print("✓ Modèle téléchargé! Décompresse-le sur ton PC.")

# Le fichier important est: weights/best.pt

# ========== CELLULE 9 (Optionnel): Tester une prédiction ==========
from ultralytics import YOLO
from PIL import Image

# Charger le meilleur modèle
model = YOLO('runs/xarm/train_blue_green_red/weights/best.pt')

# Prédire sur une image de test
results = model.predict(
    source='yolo_dataset/images/test',  # Dossier de test
    save=True,                          # Sauvegarder les résultats
    conf=0.5                            # Seuil de confiance
)

print("✓ Prédictions sauvegardées dans runs/detect/predict")
