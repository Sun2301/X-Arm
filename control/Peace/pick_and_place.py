#!/usr/bin/env python3
"""
pick_and_place.py
-----------------
Systeme complet pick-and-place en un seul fichier.

Pipeline :
    1. Detection du cube par segmentation HSV (OpenCV)
    2. Deprojection pixel -> 3D repere camera  (K + Z_CAM)
    3. Transformation camera -> base du bras   (R_cb, T_cb)
    4. Cinematique inverse                      (ikpy)
    5. Sequence pick-and-place                  (Arm_Device)

Usage :
    # Mode camera live (normal)
    python3 pick_and_place.py

    # Mode test sur image statique (pas de bras requis)
    python3 pick_and_place.py --test --image chemin/vers/image.jpg

    # Mode test webcam sans bras (affiche detections uniquement)
    python3 pick_and_place.py --test

    # Mode calibration HSV interactive (webcam)
    python3 pick_and_place.py --tune

    # Mode calibration HSV interactive (image statique)
    python3 pick_and_place.py --tune --image chemin/vers/image.jpg

Dependances : opencv-python, numpy, ikpy
"""

import cv2
import numpy as np
import argparse
import time

# ==========================================
# CONFIGURATION CAMERA
# ==========================================

CAMERA_INDEX = 1

# Matrice intrinseque K - obtenue avec calibrate_camera.py
# REMPLACER ces valeurs apres calibration
K = np.array([[600.0,   0.0, 320.0],
              [  0.0, 600.0, 240.0],
              [  0.0,   0.0,   1.0]], dtype=np.float64)

# Distance camera-table le long de l'axe optique (metres)
# MESURER physiquement avec une regle apres avoir monte la camera
Z_CAM = 0.40   # valeur arbitraire - A REMPLACER

# ==========================================
# TRANSFORMATION CAMERA -> BASE DU BRAS
# ==========================================

T_cb = np.array([0.002, 0.139, 0.153])

R_cb = np.array([[ 0.99983586, -0.01807169,  0.00129312],
                 [-0.01734706, -0.93425894,  0.35617316],
                 [-0.00522854, -0.35613713, -0.93441908]])

# ==========================================
# CONVERSION PIXEL -> REPERE BASE
# ==========================================

def pixel_to_camera(cx, cy):
    """
    Convertit un pixel (cx, cy) en point 3D dans le repere camera.
    Utilise K et Z_CAM (distance camera-table fixe).

    Formule :
        Xc = (u - cx_K) / fx * Z_CAM
        Yc = (v - cy_K) / fy * Z_CAM
        Zc = Z_CAM
    """
    fx = K[0, 0]
    fy = K[1, 1]
    cx_K = K[0, 2]
    cy_K = K[1, 2]

    Xc = (cx - cx_K) / fx * Z_CAM
    Yc = (cy - cy_K) / fy * Z_CAM
    Zc = Z_CAM

    return np.array([Xc, Yc, Zc])


def camera_to_base(P_cam):
    """
    Transforme un point 3D du repere camera vers le repere base du bras.
    P_base = R_cb @ P_cam + T_cb
    """
    return R_cb @ P_cam + T_cb


def pixel_to_base(cx, cy):
    """Pipeline complet : pixel -> repere base."""
    P_cam  = pixel_to_camera(cx, cy)
    P_base = camera_to_base(P_cam)
    return P_base, P_cam

# ==========================================
# DETECTION HSV
# ==========================================

HSV_RANGES = {
    "Red": [
        (np.array([0,   100,  50]),  np.array([10,  255, 255])),
        (np.array([170, 100,  50]),  np.array([180, 255, 255])),
    ],
    "Green": [
        (np.array([35,  60,  40]),   np.array([85,  255, 255])),
    ],
    "Blue": [
        (np.array([85,  80,  50]),   np.array([135, 255, 255])),
    ],
}

MIN_AREA    = 500
MAX_AREA    = 200_000
BLUR_KERNEL = 7
MORPH_K     = 11


def build_mask(hsv, color):
    """
    Construit le masque binaire pour la couleur donnee.
    Applique un hull convexe pour couvrir toutes les faces du cube.
    """
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, lo, hi)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_K, MORPH_K))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hull_mask = np.zeros_like(mask)
    for cnt in contours:
        if cv2.contourArea(cnt) > MIN_AREA:
            hull = cv2.convexHull(cnt)
            cv2.drawContours(hull_mask, [hull], -1, 255, -1)

    return hull_mask


def detect_cubes(frame):
    """
    Detecte tous les cubes visibles dans la frame.
    Retourne une liste de dicts :
        {color, contour, bbox(x,y,w,h), centroid(cx,cy), area}
    """
    blurred = cv2.GaussianBlur(frame, (BLUR_KERNEL, BLUR_KERNEL), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    detections = []

    for color in HSV_RANGES:
        mask = build_mask(hsv, color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (MIN_AREA < area < MAX_AREA):
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            M  = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"]) if M["m00"] else x + w // 2
            cy = int(M["m01"] / M["m00"]) if M["m00"] else y + h // 2

            detections.append({
                "color":    color,
                "contour":  cnt,
                "bbox":     (x, y, w, h),
                "centroid": (cx, cy),
                "area":     area,
            })

    detections.sort(key=lambda d: d["area"], reverse=True)
    return detections


def annotate_frame(frame, detections):
    """Dessine les detections sur la frame."""
    COLOR_BGR = {"Red": (0, 0, 255), "Green": (0, 255, 0), "Blue": (255, 0, 0)}
    annotated = frame.copy()

    for det in detections:
        bgr        = COLOR_BGR.get(det["color"], (255, 255, 255))
        x, y, w, h = det["bbox"]
        cx, cy     = det["centroid"]

        cv2.drawContours(annotated, [det["contour"]], -1, bgr, 2)
        cv2.rectangle(annotated, (x, y), (x+w, y+h), bgr, 2)
        cv2.circle(annotated, (cx, cy), 5, (255, 255, 255), -1)
        cv2.putText(annotated,
                    f"{det['color']} ({int(det['area'])} px2)",
                    (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, bgr, 2)

    return annotated

# ==========================================
# CINEMATIQUE INVERSE (ikpy)
# ==========================================

def load_chain(urdf_path="arm.urdf"):
    try:
        import ikpy.chain
        chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["base_link"],
        )
        print(f"[IK] Chaine chargee : {len(chain.links)} liens")
        return chain
    except ImportError:
        print("[IK] ikpy non installe - mode test uniquement")
        return None
    except FileNotFoundError:
        print(f"[IK] URDF introuvable : {urdf_path} - mode test uniquement")
        return None


RA2DE = 180.0 / np.pi
DE2RA = np.pi  / 180.0


def get_current_seed(arm, chain):
    angles_deg = [arm.Arm_serial_servo_read(i) for i in range(1, 6)]
    angles_rad = [
        (a - 90.0) * DE2RA if a is not None else 0.0
        for a in angles_deg
    ]
    seed = [0.0] + angles_rad + [0.0] * (len(chain.links) - 6)
    return seed


def compute_ik(chain, x, y, z, seed=None):
    """
    Calcule les angles servos pour atteindre (x, y, z) dans le repere base.
    Retourne (joints_rad, joints_deg).
    """
    if seed is None:
        seed = [0.0] * len(chain.links)

    result = chain.inverse_kinematics(
        target_position=[x, y, z],
        initial_position=seed,
        orientation_mode=None,
    )

    joints_rad    = list(result[1:6])
    joints_rad[4] = -np.pi / 2.0

    joints_deg = [round(j * RA2DE + 90.0, 2) for j in joints_rad]

    full = [0.0] + joints_rad
    T    = chain.forward_kinematics(full)
    pos  = T[:3, 3]
    err  = np.linalg.norm(pos - np.array([x, y, z]))

    print(f"[IK] cible    : ({x:.4f}, {y:.4f}, {z:.4f}) m")
    print(f"[IK] angles   : {joints_deg}")
    print(f"[IK] FK retour: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m")
    print(f"[IK] erreur   : {err:.5f} m")

    return joints_rad, joints_deg

# ==========================================
# SECURITE SERVOS
# ==========================================

LIMITS = [
    (0, 180),  # j1
    (0, 180),  # j2
    (0, 180),  # j3
    (0, 180),  # j4
    (0, 270),  # j5
]

def check_limits(joints_deg):
    for i, (val, (lo, hi)) in enumerate(zip(joints_deg, LIMITS)):
        if not (lo <= val <= hi):
            print(f"[ARM] ERREUR joint{i+1}={val} hors limites [{lo},{hi}]")
            return False
    return True

# ==========================================
# COMMANDES BRAS
# ==========================================

GRIPPER_OPEN   = 15
GRIPPER_CLOSED = 112
MOVE_TIME      = 1500
HOME_TIME      = 2000
PLACE_TIME     = 1500
HOME_JOINTS    = [90, 90, 90, 90, 90]

# Position de capture - le bras revient ici avant chaque detection.
# C'est depuis cette position que R_cb/T_cb sont valables (calibration faite ici).
CAPTURE_JOINTS = [90, 99.3, 9, 91, 174]
CAPTURE_TIME   = 1000   # ms

# Position de depot
PLACE_X      =  0.00
PLACE_Y      =  0.12
PLACE_Z_LIFT =  0.12
PLACE_Z_DROP =  0.075

# Hauteur de saisie = moitie de la hauteur du cube (cube 4cm => 2cm)
Z_PICK = 0.02


def go_home(arm):
    print("[ARM] Retour home...")
    j1, j2, j3, j4, j5 = HOME_JOINTS
    arm.Arm_serial_servo_write6(j1, j2, j3, j4, j5, GRIPPER_OPEN, HOME_TIME)
    time.sleep(HOME_TIME / 1000.0 + 0.5)


def go_to_capture_position(arm):
    """
    Ramene le bras a la position fixe de capture.
    R_cb et T_cb ne sont valables QUE depuis cette position exacte -
    le bras doit y revenir avant chaque detection de cube.
    """
    print("[ARM] Retour position de capture...")
    j1, j2, j3, j4, j5 = CAPTURE_JOINTS
    arm.Arm_serial_servo_write6(j1, j2, j3, j4, j5, GRIPPER_OPEN, CAPTURE_TIME)
    time.sleep(CAPTURE_TIME / 1000.0 + 0.5)


def move_to(arm, joints_deg, gripper=GRIPPER_OPEN, duration=MOVE_TIME):
    if not check_limits(joints_deg):
        return False
    j1, j2, j3, j4, j5 = joints_deg
    arm.Arm_serial_servo_write6(j1, j2, j3, j4, j5, gripper, duration)
    time.sleep(duration / 1000.0 + 0.3)
    return True


def open_gripper(arm):
    arm.Arm_serial_servo_write(6, GRIPPER_OPEN, 600)
    time.sleep(0.8)


def close_gripper(arm):
    arm.Arm_serial_servo_write(6, GRIPPER_CLOSED, 600)
    time.sleep(0.8)

# ==========================================
# SEQUENCE PICK AND PLACE
# ==========================================

def pick_and_place(arm, chain, x_obj, y_obj):
    """
    Sequence complete 8 etapes.
    x_obj, y_obj : coordonnees dans le repere BASE du bras (metres).
    """
    Z_TRANSIT = PLACE_Z_LIFT
    seed = get_current_seed(arm, chain)

    # 1. Transit au-dessus de l'objet
    print("\n[SEQ] 1. Transit au-dessus de l'objet...")
    _, jd = compute_ik(chain, x_obj, y_obj, Z_TRANSIT, seed)
    open_gripper(arm)
    if not move_to(arm, jd):
        print("[SEQ] Position transit inaccessible - abandon.")
        go_home(arm)
        return False

    # 2. Descente sur l'objet
    print("[SEQ] 2. Descente sur l'objet...")
    seed = get_current_seed(arm, chain)
    _, jd = compute_ik(chain, x_obj, y_obj, Z_PICK, seed)
    if not move_to(arm, jd):
        print("[SEQ] Position pick inaccessible - abandon.")
        go_home(arm)
        return False

    # 3. Saisie
    print("[SEQ] 3. Saisie...")
    close_gripper(arm)

    # 4. Remontee
    print("[SEQ] 4. Remontee avec objet...")
    seed = get_current_seed(arm, chain)
    _, jd = compute_ik(chain, x_obj, y_obj, Z_TRANSIT, seed)
    move_to(arm, jd)

    # 5. Transit vers depot
    print("[SEQ] 5. Transit vers depot...")
    seed = get_current_seed(arm, chain)
    _, jd = compute_ik(chain, PLACE_X, PLACE_Y, PLACE_Z_LIFT, seed)
    if not move_to(arm, jd, gripper=GRIPPER_CLOSED, duration=PLACE_TIME):
        print("[SEQ] Position depot transit inaccessible.")
        go_home(arm)
        return False

    # 6. Descente depot
    print("[SEQ] 6. Descente depot...")
    seed = get_current_seed(arm, chain)
    _, jd = compute_ik(chain, PLACE_X, PLACE_Y, PLACE_Z_DROP, seed)
    move_to(arm, jd, gripper=GRIPPER_CLOSED, duration=PLACE_TIME)

    # 7. Relache
    print("[SEQ] 7. Relachement...")
    open_gripper(arm)

    # 8. Retour home
    go_home(arm)

    print("[SEQ] Pick and place termine avec succes.")
    return True

# ==========================================
# MODE TEST (sans bras physique)
# ==========================================

def test_on_image(image_path):
    """
    Teste la detection et la conversion pixel->base sur une image statique.
    Aucun bras requis.
    """
    print(f"\n{'='*55}")
    print(f"  MODE TEST - Image : {image_path}")
    print(f"{'='*55}\n")

    frame = cv2.imread(image_path)
    if frame is None:
        print(f"[TEST] Impossible de lire l'image : {image_path}")
        return

    print(f"[TEST] Resolution image : {frame.shape[1]}x{frame.shape[0]} px")
    print(f"[TEST] Z_CAM configure  : {Z_CAM} m\n")

    detections = detect_cubes(frame)

    if not detections:
        print("[TEST] Aucun cube detecte.")
        print("[TEST] Verifier les plages HSV ou lancer --tune pour calibrer.")
    else:
        print(f"[TEST] {len(detections)} cube(s) detecte(s) :\n")
        for i, det in enumerate(detections):
            cx, cy = det["centroid"]
            P_base, P_cam = pixel_to_base(cx, cy)

            print(f"  Cube {i+1} - {det['color']}")
            print(f"    Pixel centroide    : ({cx}, {cy})")
            print(f"    Aire               : {int(det['area'])} px2")
            print(f"    Repere camera (m)  : Xc={P_cam[0]:.4f}  Yc={P_cam[1]:.4f}  Zc={P_cam[2]:.4f}")
            print(f"    Repere BASE   (m)  : Xb={P_base[0]:.4f}  Yb={P_base[1]:.4f}  Zb={P_base[2]:.4f}")
            print()

    annotated = annotate_frame(frame, detections)
    cv2.imshow("Test Detection", annotated)
    print("[TEST] Appuyer sur une touche pour fermer.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def test_live_no_arm(camera_index=CAMERA_INDEX):
    """
    Mode test live : affiche les detections en temps reel sans bras.
    """
    print("\n[TEST LIVE] Detection en temps reel - sans bras")
    print("[TEST LIVE] Q pour quitter\n")

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[TEST LIVE] Impossible d'ouvrir la camera {camera_index}")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detections = detect_cubes(frame)
        annotated  = annotate_frame(frame, detections)

        for det in detections:
            cx, cy = det["centroid"]
            P_base, P_cam = pixel_to_base(cx, cy)
            print(f"[DETECT] {det['color']:6s} | pixel=({cx:4d},{cy:4d}) | "
                  f"cam=({P_cam[0]:.3f},{P_cam[1]:.3f},{P_cam[2]:.3f}) | "
                  f"base=({P_base[0]:.3f},{P_base[1]:.3f},{P_base[2]:.3f}) m")

        cv2.imshow("Test Live - Detection HSV", annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# ==========================================
# MODE CALIBRATION HSV INTERACTIF
# ==========================================

def tune_hsv(camera_index=CAMERA_INDEX, image_path=None):
    """
    Fenetre interactive pour ajuster les plages HSV.
    - Sans --image : utilise la webcam en live
    - Avec --image : utilise une image statique
    Q pour quitter - les valeurs finales s'affichent dans le terminal.
    """
    if image_path:
        frame_orig = cv2.imread(image_path)
        if frame_orig is None:
            print(f"[TUNE] Impossible de lire l'image : {image_path}")
            return
        print(f"[TUNE] Mode image statique : {image_path}")
    else:
        print(f"[TUNE] Mode webcam live (index {camera_index})")

    cv2.namedWindow("Tune HSV", cv2.WINDOW_NORMAL)
    defaults = {"H_lo": 85, "H_hi": 135, "S_lo": 80, "S_hi": 255,
                "V_lo": 50, "V_hi": 255}
    for name, val in defaults.items():
        hi = 180 if name.startswith("H") else 255
        cv2.createTrackbar(name, "Tune HSV", val, hi, lambda v: None)

    cap = None
    if not image_path:
        cap = cv2.VideoCapture(camera_index)
        if not cap.isOpened():
            print(f"[TUNE] Impossible d'ouvrir la camera {camera_index}")
            return

    print("[TUNE] Ajuster les sliders - Q pour quitter")
    print("[TUNE] Les valeurs finales s'affichent dans le terminal a la sortie\n")

    while True:
        if image_path:
            frame = frame_orig.copy()
        else:
            ret, frame = cap.read()
            if not ret:
                break

        h_lo = cv2.getTrackbarPos("H_lo", "Tune HSV")
        h_hi = cv2.getTrackbarPos("H_hi", "Tune HSV")
        s_lo = cv2.getTrackbarPos("S_lo", "Tune HSV")
        s_hi = cv2.getTrackbarPos("S_hi", "Tune HSV")
        v_lo = cv2.getTrackbarPos("V_lo", "Tune HSV")
        v_hi = cv2.getTrackbarPos("V_hi", "Tune HSV")

        lo = np.array([h_lo, s_lo, v_lo])
        hi = np.array([h_hi, s_hi, v_hi])

        hsv  = cv2.cvtColor(
            cv2.GaussianBlur(frame, (BLUR_KERNEL, BLUR_KERNEL), 0),
            cv2.COLOR_BGR2HSV
        )
        mask = cv2.inRange(hsv, lo, hi)

        print(f"\r  lo=({h_lo:3d},{s_lo:3d},{v_lo:3d})  "
              f"hi=({h_hi:3d},{s_hi:3d},{v_hi:3d})", end="", flush=True)

        h, w = frame.shape[:2]
        if w > 900:
            scale  = 900 / w
            frame  = cv2.resize(frame, (900, int(h * scale)))
            mask_r = cv2.resize(mask,  (900, int(h * scale)))
        else:
            mask_r = mask

        display = np.hstack([frame, cv2.cvtColor(mask_r, cv2.COLOR_GRAY2BGR)])
        cv2.imshow("Tune HSV", display)

        delay = 100 if image_path else 30
        if cv2.waitKey(delay) & 0xFF == ord('q'):
            break

    if cap:
        cap.release()
    cv2.destroyAllWindows()

    print(f"\n\n[TUNE] Valeurs finales a copier dans pick_and_place.py :")
    print(f"""
    "Blue": [
        (np.array([{h_lo}, {s_lo}, {v_lo}]), np.array([{h_hi}, {s_hi}, {v_hi}])),
    ],
""")

# ==========================================
# BOUCLE PRINCIPALE (mode reel avec bras)
# ==========================================

def run(camera_index=CAMERA_INDEX):
    """
    Boucle principale : detection -> conversion -> IK -> pick and place.
    """
    try:
        from Arm_Device import Arm_Device
    except ImportError:
        print("[RUN] Arm_Device non disponible - utiliser --test")
        return

    chain = load_chain()
    if chain is None:
        print("[RUN] Chaine IK non disponible - verifier arm.urdf et ikpy")
        return

    arm = Arm_Device()
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"[RUN] Impossible d'ouvrir la camera {camera_index}")
        return

    go_home(arm)
    go_to_capture_position(arm)
    print("\n[RUN] Systeme actif - Q pour quitter\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            detections = detect_cubes(frame)
            annotated  = annotate_frame(frame, detections)
            cv2.imshow("Pick and Place", annotated)

            if detections:
                det = detections[0]
                cx, cy = det["centroid"]

                P_base, P_cam = pixel_to_base(cx, cy)
                x_obj, y_obj, z_obj = P_base

                print(f"\n[RUN] Cube detecte : {det['color']}")
                print(f"[RUN] Pixel        : ({cx}, {cy})")
                print(f"[RUN] Repere cam   : ({P_cam[0]:.4f}, {P_cam[1]:.4f}, {P_cam[2]:.4f}) m")
                print(f"[RUN] Repere base  : ({x_obj:.4f}, {y_obj:.4f}, {z_obj:.4f}) m")

                cap.release()
                cv2.destroyAllWindows()

                success = pick_and_place(arm, chain, x_obj, y_obj)

                # Retour obligatoire a la position de capture avant la
                # prochaine detection - R_cb/T_cb ne sont valables que depuis
                # cette position precise.
                go_to_capture_position(arm)

                cap = cv2.VideoCapture(camera_index)
                if not success:
                    print("[RUN] Echec - nouvelle tentative...")

                time.sleep(0.5)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[RUN] Interruption manuelle.")

    finally:
        go_home(arm)
        cap.release()
        cv2.destroyAllWindows()
        print("[RUN] Arret propre.")

# ==========================================
# MAIN
# ==========================================

def main():
    parser = argparse.ArgumentParser(description="Pick and Place - Detection HSV + IK")
    parser.add_argument("--test",   action="store_true",
                        help="Mode test sans bras (live ou image)")
    parser.add_argument("--image",  type=str, default=None,
                        help="Image statique (avec --test ou --tune)")
    parser.add_argument("--tune",   action="store_true",
                        help="Calibration HSV interactive")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX,
                        help="Index camera (defaut 1)")
    args = parser.parse_args()

    if args.tune:
        tune_hsv(camera_index=args.camera, image_path=args.image)

    elif args.test:
        if args.image:
            test_on_image(args.image)
        else:
            test_live_no_arm(args.camera)

    else:
        run(args.camera)


if __name__ == "__main__":
    main()