#!/usr/bin/env python3
"""
calibrate_camera.py
-------------------
Calibration de la matrice intrinseque K par damier de calibration.

Usage :
    # Capturer des images depuis la webcam (appuyer sur ESPACE pour capturer, Q pour finir)
    python3 calibrate_camera.py --capture

    # Calibrer depuis des images deja capturees dans le dossier ./calib_imgs/
    python3 calibrate_camera.py --calibrate

    # Tout en un (capture + calibration directe)
    python3 calibrate_camera.py --capture --calibrate

Resultat :
    Affiche fx, fy, cx, cy dans le terminal
    Sauvegarde camera_matrix.npy et dist_coeffs.npy

Dependances : opencv-python, numpy
"""

import cv2
import numpy as np
import argparse
import os
import glob

# ==========================================
# CONFIGURATION DU DAMIER
# ==========================================
# Nombre de coins INTERIEURS du damier (pas le nombre de cases)
# Ex : damier 9x6 cases => 8x5 coins interieurs
CHESSBOARD_W = 9   # coins en largeur
CHESSBOARD_H = 6   # coins en hauteur
SQUARE_SIZE  = 0.025  # taille d'une case en metres (ex: 2.5 cm)

CALIB_DIR    = "./calib_imgs"
CAMERA_INDEX = 0


# ==========================================
# CAPTURE DES IMAGES
# ==========================================
def capture_images(camera_index=CAMERA_INDEX, save_dir=CALIB_DIR, n_target=20):
    """
    Ouvre la webcam et permet de capturer des images du damier.
    ESPACE : capturer une image
    Q      : quitter
    """
    os.makedirs(save_dir, exist_ok=True)
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Impossible d'ouvrir la camera {camera_index}")

    count = 0
    print(f"\n[CALIB] Capture d'images - objectif : {n_target} images")
    print("[CALIB] ESPACE = capturer | Q = quitter\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (CHESSBOARD_W, CHESSBOARD_H), None
        )

        if found:
            cv2.drawChessboardCorners(display, (CHESSBOARD_W, CHESSBOARD_H), corners, found)
            cv2.putText(display, "Damier detecte - ESPACE pour capturer",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Damier non detecte",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"Images capturees : {count}/{n_target}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        cv2.imshow("Calibration - Capture", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' ') and found:
            path = os.path.join(save_dir, f"calib_{count:03d}.jpg")
            cv2.imwrite(path, frame)
            count += 1
            print(f"[CALIB] Image {count} sauvegardee : {path}")
            if count >= n_target:
                print("[CALIB] Nombre cible atteint.")
                break

    cap.release()
    cv2.destroyAllWindows()
    print(f"[CALIB] {count} images capturees dans {save_dir}/\n")
    return count


# ==========================================
# CALIBRATION
# ==========================================
def calibrate(image_dir=CALIB_DIR):
    """
    Calcule K et les coefficients de distorsion depuis les images capturees.
    """
    # Points 3D du damier dans son repere (Z=0, plan du damier)
    objp = np.zeros((CHESSBOARD_H * CHESSBOARD_W, 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_W, 0:CHESSBOARD_H].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    obj_points = []   # points 3D monde
    img_points = []   # points 2D image

    images = glob.glob(os.path.join(image_dir, "*.jpg"))
    if not images:
        raise FileNotFoundError(f"Aucune image trouvee dans {image_dir}/")

    print(f"[CALIB] {len(images)} images trouvees, traitement en cours...\n")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    img_shape = None

    for path in sorted(images):
        img  = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]  # (w, h)

        found, corners = cv2.findChessboardCorners(
            gray, (CHESSBOARD_W, CHESSBOARD_H), None
        )

        if found:
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            obj_points.append(objp)
            img_points.append(corners_refined)
            print(f"  OK  : {os.path.basename(path)}")
        else:
            print(f"  SKIP: {os.path.basename(path)} (damier non detecte)")

    if len(obj_points) < 5:
        raise RuntimeError("Pas assez d'images valides (minimum 5). Recapturer.")

    print(f"\n[CALIB] Calibration sur {len(obj_points)} images valides...")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_shape, None, None
    )

    # ---- Resultats ----
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    print("\n" + "="*50)
    print("  RESULTATS DE CALIBRATION")
    print("="*50)
    print(f"  Erreur de reprojection : {ret:.4f} px")
    print(f"  (< 0.5 px = excellent | < 1.0 px = acceptable)\n")
    print(f"  fx = {fx:.4f}")
    print(f"  fy = {fy:.4f}")
    print(f"  cx = {cx:.4f}")
    print(f"  cy = {cy:.4f}")
    print(f"\n  Matrice K complete :")
    print(f"  {K}")
    print(f"\n  Coefficients distorsion : {dist.ravel()}")
    print("="*50)

    print("\n[CALIB] A copier dans pick_and_place.py :")
    print(f"""
K = np.array([[{fx:.4f},       0, {cx:.4f}],
              [      0, {fy:.4f}, {cy:.4f}],
              [      0,       0,      1]])
""")

    # Sauvegarde
    np.save("camera_matrix.npy", K)
    np.save("dist_coeffs.npy",   dist)
    print("[CALIB] Sauvegardes : camera_matrix.npy  dist_coeffs.npy")

    return K, dist


# ==========================================
# MAIN
# ==========================================
def test_on_images(image_paths):
    """
    Teste la detection du damier sur des images statiques fournies.
    Affiche chaque image avec les coins detectes (ou un message d'echec).
    Utile pour verifier que les images sont exploitables avant calibration.

    Usage :
        python3 calibrate_camera.py --test img1.jpg img2.jpg ...
        python3 calibrate_camera.py --test ./mes_photos/*.jpg
    """
    if not image_paths:
        print("[TEST] Aucune image fournie. Usage : --test img1.jpg img2.jpg ...")
        return

    print(f"\n[TEST] Verification de {len(image_paths)} image(s)\n")
    ok_count = 0

    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  ERREUR : impossible de lire {path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (CHESSBOARD_W, CHESSBOARD_H), None
        )

        display = img.copy()
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners  = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, (CHESSBOARD_W, CHESSBOARD_H), corners, found)
            cv2.putText(display, "OK - Damier detecte",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            print(f"  OK   : {path}")
            ok_count += 1
        else:
            cv2.putText(display, "ECHEC - Damier non detecte",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            print(f"  ECHEC: {path}  (verifier eclairage, angle, taille damier)")

        # Redimensionner si trop grande pour l'ecran
        h, w = display.shape[:2]
        if w > 1280:
            scale   = 1280 / w
            display = cv2.resize(display, (1280, int(h * scale)))

        cv2.imshow(f"Test - {path}", display)
        print(f"         -> Appuyer sur une touche pour passer a la suivante")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print(f"\n[TEST] Resultat : {ok_count}/{len(image_paths)} images valides")
    if ok_count < 5:
        print("[TEST] ATTENTION : moins de 5 images valides - recapturer avec un meilleur eclairage.")
    elif ok_count < 10:
        print("[TEST] Correct, mais 15-20 images valides donnent une meilleure calibration.")
    else:
        print("[TEST] Bon nombre d'images - tu peux lancer --calibrate.")


def main():
    parser = argparse.ArgumentParser(description="Calibration camera par damier")
    parser.add_argument("--capture",   action="store_true",
                        help="Capturer des images depuis la webcam")
    parser.add_argument("--calibrate", action="store_true",
                        help="Calibrer depuis les images dans ./calib_imgs/")
    parser.add_argument("--test",      nargs="+", metavar="IMAGE",
                        help="Tester la detection du damier sur des images statiques")
    parser.add_argument("--camera",    type=int, default=CAMERA_INDEX,
                        help="Index camera (defaut 0)")
    parser.add_argument("--n",         type=int, default=20,
                        help="Nombre d'images a capturer (defaut 20)")
    args = parser.parse_args()

    if not args.capture and not args.calibrate and not args.test:
        parser.print_help()
        return

    if args.test:
        test_on_images(args.test)

    if args.capture:
        capture_images(camera_index=args.camera, n_target=args.n)

    if args.calibrate:
        calibrate()


if __name__ == "__main__":
    main()
    
    
    
    
print("a")