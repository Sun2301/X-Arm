# Pick and Place - X-Arm

Système complet de pick-and-place en un seul fichier.
Détecte un cube coloré par segmentation HSV, calcule sa position 3D, et commande le bras.

---

## Pipeline

```
pixel (u, v)
     ↓  K + Z_CAM
Point 3D repère caméra (Xc, Yc, Zc)
     ↓  R_cb @ P_cam + T_cb
Point 3D repère base (Xb, Yb, Zb)
     ↓  ikpy + arm.urdf
Angles servos
     ↓
Pick and place
```

---

## Fichiers nécessaires

| Fichier | Description |
|---|---|
| `pick_and_place.py` | Script principal |
| `calibrate_camera.py` | Calibration caméra (donne K) |
| `arm.urdf` | Modèle du bras pour ikpy |
| `camera_matrix.npy` | Généré par calibrate_camera.py |

---

## Étape 1 — Calibration de la caméra (trouver K)

K est la matrice intrinsèque de la caméra. Elle est nécessaire pour convertir
un pixel en point 3D dans le repère caméra.

### 1.1 Préparer le damier
- Imprimer le damier OpenCV : https://docs.opencv.org/4.x/pattern.png
- Imprimer en A4
- Coller sur quelque chose de rigide et plat (carton, livre)

### 1.2 Capturer les images
```bash
python3 calibrate_camera.py --capture
```
- Appuyer sur **ESPACE** pour capturer quand le damier est détecté (contours verts affichés)
- Capturer **15 à 20 images** sous différents angles et distances
- Appuyer sur **Q** pour terminer

### 1.3 Tester les images capturées (optionnel)
```bash
python3 calibrate_camera.py --test ./calib_imgs/*.jpg
```
Affiche OK ou ECHEC pour chaque image. Minimum 10 images valides recommandé.

### 1.4 Calculer K
```bash
python3 calibrate_camera.py --calibrate
```

**Résultat attendu dans le terminal :**
```
==================================================
  RESULTATS DE CALIBRATION
==================================================
  Erreur de reprojection : 0.42 px
  (< 0.5 px = excellent | < 1.0 px = acceptable)

  fx = 847.3241
  fy = 849.1023
  cx = 318.4412
  cy = 241.8876

  A copier dans pick_and_place.py :

K = np.array([[847.3241,       0, 318.4412],
              [      0, 849.1023, 241.8876],
              [      0,       0,      1]])
==================================================
```

### 1.5 Copier K dans pick_and_place.py
Remplacer les valeurs provisoires :
```python
# AVANT
K = np.array([[600.0,   0.0, 320.0],
              [  0.0, 600.0, 240.0],
              [  0.0,   0.0,   1.0]])

# APRES (exemple avec vos valeurs)
K = np.array([[847.3241,       0, 318.4412],
              [      0, 849.1023, 241.8876],
              [      0,       0,      1]])
```

---

## Étape 2 — Mesurer Z_CAM

Z_CAM est la distance entre la caméra et la surface de la table,
mesurée le long de l'axe optique de la caméra.

- Si la caméra est **parfaitement verticale** : Z_CAM = distance caméra-table à la règle
- Si la caméra est **inclinée** : Z_CAM = distance le long de l'axe de la caméra (un peu plus grande)

Mesurer à la règle et mettre à jour dans `pick_and_place.py` :
```python
Z_CAM = 0.40   # remplacer par votre valeur en metres
```

---

## Étape 3 — Calibration HSV (si les couleurs ne sont pas bien détectées)

### Sur image statique
```bash
python3 pick_and_place.py --tune --image image.jpg
```

### Sur webcam live
```bash
python3 pick_and_place.py --tune
```

Ajuster les sliders jusqu'à ce que seul le cube apparaisse en blanc dans le masque.
Les valeurs finales s'affichent dans le terminal à la sortie (Q).

Copier les valeurs dans `pick_and_place.py` dans la section `HSV_RANGES`.

---

## Étape 4 — Test sans bras

### Sur image statique
```bash
python3 pick_and_place.py --test --image image.jpg
```

**Résultat attendu dans le terminal :**
```
=======================================================
  MODE TEST - Image : image.jpg
=======================================================

[TEST] Resolution image : 512x512 px
[TEST] Z_CAM configure  : 0.40 m

[TEST] 1 cube(s) detecte(s) :

  Cube 1 - Blue
    Pixel centroide    : (226, 289)
    Aire               : 51537 px2
    Repere camera (m)  : Xc=-0.0213  Yc=0.0326  Zc=0.4000
    Repere BASE   (m)  : Xb=0.1234  Yb=0.1456  Zb=0.0521
```

**Vérification :** Mesurer à la règle la distance entre le cube et la base du bras.
Comparer avec Xb et Yb. Si cohérent, la calibration est correcte.

### Sur webcam live (sans bras)
```bash
python3 pick_and_place.py --test
```

---

## Étape 5 — Mode réel avec bras

```bash
python3 pick_and_place.py
```

Le bras va en position home, attend de détecter un cube, puis exécute la séquence.

**Séquence pick-and-place :**
1. Transit au-dessus de l'objet (Z = 0.12 m)
2. Descente sur l'objet (Z = Z_PICK = 0.02 m)
3. Fermeture du gripper
4. Remontée (Z = 0.12 m)
5. Transit vers le point de dépôt
6. Descente au point de dépôt
7. Ouverture du gripper
8. Retour home

**Arrêt propre :** Ctrl+C

---

## Paramètres à adapter

Dans `pick_and_place.py` :

```python
# Caméra
CAMERA_INDEX = 1       # index de ta caméra (0 = webcam interne, 1 = externe)
Z_CAM = 0.40           # distance caméra-table en mètres (à mesurer)

# Cube
Z_PICK = 0.02          # moitié de la hauteur du cube (cube 4cm => 0.02m)

# Point de dépôt (repère base du bras, en mètres)
PLACE_X      =  0.00
PLACE_Y      =  0.12
PLACE_Z_LIFT =  0.12
PLACE_Z_DROP =  0.075

# Gripper
GRIPPER_OPEN   = 15
GRIPPER_CLOSED = 112
```

---

## Erreurs fréquentes

### `ikpy non installe`
```bash
pip install ikpy
```

### `URDF introuvable : arm.urdf`
Vérifier que `arm.urdf` est dans le même dossier que `pick_and_place.py`.

### `Impossible d'ouvrir la camera 1`
Changer `CAMERA_INDEX = 0` ou vérifier que la caméra est branchée.

### Cube non détecté
Lancer `--tune` pour ajuster les plages HSV selon l'éclairage actuel.

### Erreur de reprojection > 1.0 px lors de la calibration
Recapturer les images : meilleur éclairage, damier bien à plat, plus d'angles différents.

### Le bras va au mauvais endroit
1. Vérifier que K est correct (erreur reprojection < 1.0 px)
2. Vérifier Z_CAM (mesurer à la règle)
3. Tester en mode `--test --image` et comparer Xb/Yb avec la position réelle du cube

---

## Structure des fichiers

```
.
├── pick_and_place.py      # script principal
├── calibrate_camera.py    # calibration K
├── arm.urdf               # modèle du bras
├── camera_matrix.npy      # K sauvegardée (généré par calibrate_camera.py)
├── dist_coeffs.npy        # distorsion (généré par calibrate_camera.py)
└── calib_imgs/            # images de calibration (généré par --capture)
```
