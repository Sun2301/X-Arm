#!/usr/bin/env python3
"""
Script pour convertir le dataset Edge Impulse au format YOLO
"""

import json
import os
import shutil
import random
from pathlib import Path

def convert_bbox_to_yolo(bbox, img_width, img_height):
    """
    Convertit les bounding boxes du format Edge Impulse au format YOLO
    Edge Impulse: x, y, width, height (pixels absolus)
    YOLO: x_center, y_center, width, height (normalisés entre 0 et 1)
    """
    x = bbox['x']
    y = bbox['y']
    w = bbox['width']
    h = bbox['height']
    
    # Calculer le centre de la boîte
    x_center = (x + w / 2) / img_width
    y_center = (y + h / 2) / img_height
    
    # Normaliser width et height
    width_norm = w / img_width
    height_norm = h / img_height
    
    return x_center, y_center, width_norm, height_norm

def create_yolo_dataset():
    """
    Crée la structure de données YOLO à partir du dataset Edge Impulse
    """
    # Chemins
    base_dir = Path(__file__).parent
    source_dir = base_dir / "imageDataset"
    yolo_dir = base_dir / "yolo_dataset"
    
    # Créer la structure YOLO avec train, val et test
    train_images_dir = yolo_dir / "images" / "train"
    val_images_dir = yolo_dir / "images" / "val"
    test_images_dir = yolo_dir / "images" / "test"
    train_labels_dir = yolo_dir / "labels" / "train"
    val_labels_dir = yolo_dir / "labels" / "val"
    test_labels_dir = yolo_dir / "labels" / "test"
    
    for directory in [train_images_dir, val_images_dir, test_images_dir, 
                      train_labels_dir, val_labels_dir, test_labels_dir]:
        directory.mkdir(parents=True, exist_ok=True)
    
    # Charger les informations des labels
    with open(source_dir / "info.labels", 'r') as f:
        data = json.load(f)
    
    # Mapping des classes
    classes = {"Blue": 0, "Green": 1, "Red": 2}
    
    # Dimensions des images (à ajuster selon vos images)
    # Pour Edge Impulse, souvent 320x320 ou 512x512
    # On va détecter automatiquement à partir de la première image
    img_width = 512
    img_height = 512
    
    print("Conversion du dataset en cours...")
    print(f"Classes détectées: {classes}")
    
    # Séparer les données en train, val et test
    training_items = [item for item in data['files'] if item['category'] == 'training']
    testing_items = [item for item in data['files'] if item['category'] == 'testing']
    
    # Mélanger les données d'entraînement pour diviser en train/val
    random.seed(42)  # Pour avoir toujours la même division
    random.shuffle(training_items)
    
    # Diviser training en 70% train et 30% val
    split_idx = int(len(training_items) * 0.7)
    train_items = training_items[:split_idx]
    val_items = training_items[split_idx:]
    
    print(f"\nRépartition des données:")
    print(f"  - Train: {len(train_items)} images")
    print(f"  - Val: {len(val_items)} images")
    print(f"  - Test: {len(testing_items)} images")
    
    processed_count = {"train": 0, "val": 0, "test": 0}
    
    # Traiter tous les ensembles
    for split_name, items in [("train", train_items), ("val", val_items), ("test", testing_items)]:
        for item in items:
            # Déterminer si c'est training ou testing
            image_path = source_dir / item['path']
            
            if not image_path.exists():
                print(f"Image non trouvée: {image_path}")
                continue
            
            # Obtenir les dimensions réelles de l'image
            try:
                from PIL import Image
                with Image.open(image_path) as img:
                    img_width, img_height = img.size
            except:
                print(f" Impossible de lire les dimensions de {image_path}, utilisation des valeurs par défaut")
            
            # Déterminer les dossiers de destination selon le split
            if split_name == "train":
                dest_images_dir = train_images_dir
                dest_labels_dir = train_labels_dir
            elif split_name == "val":
                dest_images_dir = val_images_dir
                dest_labels_dir = val_labels_dir
            else:  # test
                dest_images_dir = test_images_dir
                dest_labels_dir = test_labels_dir
        
            # Nouveau nom de fichier (simplifié)
            image_name = image_path.name
            label_name = image_path.stem + ".txt"
            
            # Copier l'image
            shutil.copy2(image_path, dest_images_dir / image_name)
            
            # Créer le fichier de labels YOLO
            label_file_path = dest_labels_dir / label_name
            
            with open(label_file_path, 'w') as f:
                # Traiter chaque bounding box
                for bbox in item.get('boundingBoxes', []):
                    label = bbox['label']
                    
                    # Ignorer les labels inconnus
                    if label not in classes:
                        print(f"Classe inconnue ignorée: {label} dans {image_name}")
                        continue
                    
                    class_id = classes[label]
                    x_center, y_center, width, height = convert_bbox_to_yolo(bbox, img_width, img_height)
                    
                    # Écrire au format YOLO: class_id x_center y_center width height
                    f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
            
            processed_count[split_name] += 1
    
    print(f"\n Conversion terminée!")
    print(f"  - Images d'entraînement (train): {processed_count['train']}")
    print(f"  - Images de validation (val): {processed_count['val']}")
    print(f"  - Images de test: {processed_count['test']}")
    
    # Créer le fichier data.yaml
    yaml_content = f"""# Configuration du dataset YOLO pour X-Arm
path: {yolo_dir.absolute()}  # chemin du dataset
train: images/train  # chemin des images d'entraînement (relatif à 'path')
val: images/val  # chemin des images de validation (relatif à 'path')
test: images/test  # chemin des images de test (relatif à 'path')

# Classes
nc: 3  # nombre de classes
names: ['Blue', 'Green', 'Red']  # noms des classes
"""
    
    yaml_path = yolo_dir / "data.yaml"
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    
    print(f"\n Fichier de configuration créé: {yaml_path}")
    print(f"\n Structure du dataset YOLO:")
    print(f"   {yolo_dir}/")
    print(f"   ├── data.yaml")
    print(f"   ├── images/")
    print(f"   │   ├── train/  ({processed_count['train']} images)")
    print(f"   │   ├── val/    ({processed_count['val']} images)")
    print(f"   │   └── test/   ({processed_count['test']} images)")
    print(f"   └── labels/")
    print(f"       ├── train/  ({processed_count['train']} fichiers .txt)")
    print(f"       ├── val/    ({processed_count['val']} fichiers .txt)")
    print(f"       └── test/   ({processed_count['test']} fichiers .txt)")
    
    
    print(f"\n Prêt à entraîner avec YOLO!")
    print(f"   Entraînement: yolo train data={yaml_path} model=yolov8n.pt epochs=100")
    print(f"   Test: yolo val data={yaml_path} model=runs/detect/train/weights/best.pt split=test")

if __name__ == "__main__":
    create_yolo_dataset()
