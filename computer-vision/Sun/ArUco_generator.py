import cv2
import os

# Configuration
OUTPUT_DIR = 'aruco_markers'
MARKER_SIZE = 400  # Taille du marqueur en pixels
QUIET_ZONE = 50    # Bordure blanche (quiet zone)
NUM_MARKERS = 4
DICT_TYPE = cv2.aruco.DICT_4X4_50

# Créer le répertoire de sortie
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Obtenir le dictionnaire ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_TYPE)

# Générer les marqueurs
for marker_id in range(NUM_MARKERS):
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE)
    
    # Ajouter la quiet zones
    marker_with_border = cv2.copyMakeBorder(
        marker_image, 
        QUIET_ZONE, QUIET_ZONE, QUIET_ZONE, QUIET_ZONE,
        cv2.BORDER_CONSTANT, 
        value=255
    )
    
    # Save le marqueur
    output_path = os.path.join(OUTPUT_DIR, f'marker_{marker_id}.png')
    cv2.imwrite(output_path, marker_with_border)
    print(f' Marqueur {marker_id} généré: {output_path}')