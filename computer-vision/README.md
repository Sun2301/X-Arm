# Détection de Couleurs YOLO - X-Arm AI

Système de détection en temps réel des couleurs (Blue, Green, Red) utilisant YOLO v8, entraîné sur un dataset customisé pour le bras robotique X-Arm.

## Résultats du Modèle

- **mAP50**: 0.884 (88.4% de précision)
- **mAP50-95**: 0.704 (70.4% de robustesse)
- **Modèle**: YOLOv8n (nano)
- **Framework**: Ultralytics YOLO

---

## Structure du Projet

```
computer-vision/
├── models/
│   └── best.pt                    # Modèle entraîné (téléchargé de Colab)
├── yolo_dataset/
│   ├── data.yaml                  # Configuration du dataset
│   ├── images/
│   │   ├── train/                 # Images d'entraînement (70%)
│   │   ├── val/                   # Images de validation (30%)
│   │   └── test/                  # Images de test
│   └── labels/
│       ├── train/                 # Annotations d'entraînement (.txt)
│       ├── val/                   # Annotations de validation (.txt)
│       └── test/                  # Annotations de test (.txt)
├── imageDataset/                  # Dataset brut (Edge Impulse)
│   ├── training/                  # Images d'entraînement
│   ├── testing/                   # Images de test
│   └── info.labels                # Métadonnées
├── webcam_results/                # Captures et vidéos
├── convert_to_yolo.py             # Script de conversion
├── test_model.py                  # Test sur images statiques
├── webcam_simple.py               # Détection en temps réel
└── README.md                       # Ce fichier
```

---

## Installation

### 1. Activer l'environnement virtuel

```bash
cd /home/romaric/Documents/ProjetX_Arm
source ./env/bin/activate
```

### 2. Vérifier les dépendances

Les packages essentiels sont déjà installés :

- `ultralytics` (YOLO)
- `torch` & `torchvision`
- `opencv-python` (cv2)

Pour vérifier :

```bash
/home/romaric/Documents/ProjetX_Arm/env/bin/pip list | grep -E "ultralytics|torch|opencv"
```

---

## Préparation du Dataset

### Convertir le Dataset Edge Impulse en Format YOLO

Si vous avez un nouveau dataset Edge Impulse, convertissez-le :

```bash
cd /home/romaric/Documents/ProjetX_Arm/X-Arm/computer-vision
/home/romaric/Documents/ProjetX_Arm/env/bin/python convert_to_yolo.py
```

**Ce script fait :**

- Convertit les annotations Edge Impulse en format YOLO
- Divise les données : 70% train, 30% val, 100% test
- Normalise les bounding boxes
- Crée `data.yaml` pour la configuration

**Résultat attendu :**

```
Structure du dataset YOLO:
   yolo_dataset/
   ├── data.yaml
   ├── images/
   │   ├── train/  (X images)
   │   ├── val/    (X images)
   │   └── test/   (X images)
   └── labels/
       ├── train/  (X fichiers .txt)
       ├── val/    (X fichiers .txt)
       └── test/   (X fichiers .txt)
```

---

## Entraîner le Modèle

### Sur Google Colab (Recommandé)

**Avantages :** GPU gratuit, pas de configuration locale

1. **Préparer le dataset :**

   ```bash
   cd /home/romaric/Documents/ProjetX_Arm/X-Arm/computer-vision
   zip -r yolo_dataset.zip yolo_dataset/
   ```

2. **Ouvrir Google Colab :** https://colab.research.google.com

3. **Créer un notebook et exécuter :**

   - Voir le fichier `train_yolo_colab.py` pour les instructions détaillées

4. **Runtime > Change runtime type > GPU (T4)**

5. **Résultats :** Télécharger `model_trained.zip` et extraire `weights/best.pt`

### Localement (Si GPU disponible)

```bash
cd /home/romaric/Documents/ProjetX_Arm/X-Arm/computer-vision

/home/romaric/Documents/ProjetX_Arm/env/bin/python -c "
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
results = model.train(
    data='yolo_dataset/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,  # GPU 0 (modifier selon votre config)
    patience=20
)
"
```

---

## Tester le Modèle

### 1. Test sur Images Statiques

```bash
cd /home/romaric/Documents/ProjetX_Arm/X-Arm/computer-vision
/home/romaric/Documents/ProjetX_Arm/env/bin/python test_model.py
```

**Ce script :**

- Charge le modèle `models/best.pt`
- Teste sur le dossier `yolo_dataset/images/test/`
- Affiche les métriques (mAP50, mAP50-95)
- Sauvegarde les résultats dans `runs/detect/predict/`

**Output :**

```
mAP50: 0.884
mAP50-95: 0.704
✓ Résultats sauvegardés dans runs/detect/predict
```

### 2. Détection Temps Réel avec Webcam

```bash
cd /home/romaric/Documents/ProjetX_Arm/X-Arm/computer-vision

# Configurer le serveur graphique
export QT_QPA_PLATFORM=xcb
export DISPLAY=:0

# Lancer le script
/home/romaric/Documents/ProjetX_Arm/env/bin/python webcam_simple.py
```

**Fonctionnalités :**

- Affichage temps réel de la webcam
- Détection Blue/Green/Red avec boîtes colorées
- Affichage de la confiance et du nombre de détections
- Appuie `s` pour sauvegarder une capture
- Appuie `q` pour quitter

**Résultats :** Captures sauvegardées dans `webcam_results/`

---

## Utilisation du Modèle dans Votre Code

### Exemple Python Simple

```python
from ultralytics import YOLO
from pathlib import Path
import cv2

# Charger le modèle
model = YOLO('models/best.pt')

# Sur une image
results = model.predict(source='path/to/image.jpg', conf=0.5)

# Sur la webcam
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, verbose=False, conf=0.5)

    # Traiter les détections
    if results[0].boxes:
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            class_id = int(box.cls[0])

            class_names = {0: "Blue", 1: "Green", 2: "Red"}
            print(f"Détecté: {class_names[class_id]} (confiance: {conf:.2f})")

    cv2.imshow("YOLO", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### Pour Contrôler le Bras Robotique

```python
from ultralytics import YOLO
import cv2

model = YOLO('models/best.pt')

# Mapping classe -> action robot
actions = {
    0: "move_left",    # Blue
    1: "move_center",  # Green
    2: "move_right"    # Red
}

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    results = model(frame, verbose=False, conf=0.5)

    if results[0].boxes:
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            action = actions[class_id]

            # TODO: Envoyer commande au robot X-Arm
            print(f"Action: {action}")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## Améliorer les Performances

### Si le modèle sous-performe :

1. **Augmenter le nombre d'epochs :**

   ```bash
   epochs=200  # Au lieu de 100
   ```

2. **Utiliser un modèle plus puissant :**

   ```bash
   model=yolov8s.pt  # Small (au lieu de Nano)
   # ou
   model=yolov8m.pt  # Medium (plus lent mais plus précis)
   ```

3. **Augmenter la taille des images :**

   ```bash
   imgsz=768  # Au lieu de 640
   ```

4. **Ajouter plus de données :**
   - Plus d'images dans `imageDataset/training/`
   - Relancer la conversion et le réentraînement

---

## Dépannage

### Erreur : "Modèle non trouvé"

```
 Modèle non trouvé: models/best.pt
```

**Solution :** Téléchargez `best.pt` depuis Colab et placez-le dans le dossier `models/`

### Erreur : "Impossible d'accéder à la webcam"

```
 Impossible d'accéder à la webcam!
```

**Solutions :**

- Vérifiez que votre webcam est connectée
- Testez : `ls -la /dev/video*`
- Changez l'index : `cv2.VideoCapture(0)` → `cv2.VideoCapture(1)`

### Erreur Qt/Wayland

```
qt.qpa.plugin: Could not load the Qt platform plugin
```

**Solution :**

```bash
export QT_QPA_PLATFORM=xcb
export DISPLAY=:0
python webcam_simple.py
```

### Lenteur de la Détection

- Le CPU est lent pour l'inférence temps réel
- Solutions :
  - Utiliser un GPU
  - Réduire la taille : `imgsz=320`
  - Utiliser un modèle plus petit : `yolov8n.pt`

---

## Références

- **YOLO Documentation :** https://docs.ultralytics.com/
- **Google Colab :** https://colab.research.google.com/
- **OpenCV :** https://opencv.org/

---

## Contributeurs

- Romaric - Développement initial du pipeline YOLO
- Équipe X-Arm AI

---

## Changelog

### v1.0 (20 Décembre 2025)

- Dataset YOLO converti (Edge Impulse → YOLO)
- Modèle entraîné avec mAP50: 0.884
- Détection temps réel en webcam
- Documentation complète

---

**Prêt à utiliser.**
