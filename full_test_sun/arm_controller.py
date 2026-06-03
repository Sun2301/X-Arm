#!/usr/bin/env python3
"""
arm_controller.py
Controleur du bras : client ZMQ (REQ).
Demande une detection au detecteur, calcule l'IK, execute le pick and place.

ZMQ pattern : REQ (client)
"""

import numpy as np
import ikpy.chain
import time
import zmq
import json
from Arm_Device import Arm_Device

# ==========================================
# CONFIGURATION
# ==========================================

URDF_PATH      = "arm.urdf"
ZMQ_HOST       = "localhost"
ZMQ_PORT       = 5555

# Servos
GRIPPER_OPEN   = 15
GRIPPER_CLOSED = 112
MOVE_TIME      = 1500    # ms deplacement vers cible
HOME_TIME      = 2000    # ms deplacement vers home
PLACE_TIME     = 1500    # ms deplacement vers place

# Position home (neutres, servos en degres)
HOME_JOINTS    = [90, 90, 90, 90, 90]

# Position de depot (a gauche, sure)
PLACE_X        =  0.00
PLACE_Y        =  0.12
PLACE_Z_LIFT   =  0.12   # hauteur de transit avant de deposer
PLACE_Z_DROP   =  0.075  # hauteur de depot

# Hauteur fixe du flange pour la saisie sur table
Z_PICK         = 0.075   # metres (h_objet / 2 fixe)

RA2DE = 180.0 / np.pi
DE2RA = np.pi  / 180.0

# ==========================================
# CHAINE CINEMATIQUE
# ==========================================

def load_chain(urdf_path=URDF_PATH):
    chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path,
        base_elements=["base_link"],
    )
    print(f"[ARM] Chaine chargee : {len(chain.links)} liens")
    return chain


# ==========================================
# LECTURE POSITION ACTUELLE
# ==========================================

def get_current_seed(arm, chain):
    """
    Lit les angles physiques actuels (servos 1-5) et construit
    le vecteur initial_position pour ikpy.
    """
    angles_deg = [arm.Arm_serial_servo_read(i) for i in range(1, 6)]
    angles_rad = [
        (a - 90.0) * DE2RA if a is not None else 0.0
        for a in angles_deg
    ]
    # format ikpy : [base] + [5 joints] + [zeros restants]
    seed = [0.0] + angles_rad + [0.0] * (len(chain.links) - 6)
    return seed


# ==========================================
# IK
# ==========================================

def compute_ik(chain, x, y, z, seed=None):
    if seed is None:
        seed = [0.0] * len(chain.links)

    result = chain.inverse_kinematics(
        target_position=[x, y, z],
        initial_position=seed,
        orientation_mode=None,
    )

    joints_rad    = list(result[1:6])
    joints_rad[4] = -np.pi / 2.0          # joint5 force vers le haut

    joints_deg = [round(j * RA2DE + 90.0, 2) for j in joints_rad]

    # verification FK
    full = [0.0] + joints_rad
    T    = chain.forward_kinematics(full)
    pos  = T[:3, 3]
    err  = np.linalg.norm(pos - np.array([x, y, z]))

    print(f"[IK] cible=({x:.4f}, {y:.4f}, {z:.4f})")
    print(f"[IK] angles servo : {joints_deg}")
    print(f"[IK] FK result    : ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"[IK] erreur       : {err:.5f} m")

    return joints_rad, joints_deg


# ==========================================
# SECURITE
# ==========================================

LIMITS = [
    (0,   180),   # j1
    (0,   180),   # j2
    (0,   180),   # j3
    (0,   180),   # j4
    (0,   270),   # j5
]

def check_limits(joints_deg):
    for i, (val, (lo, hi)) in enumerate(zip(joints_deg, LIMITS)):
        if not (lo <= val <= hi):
            print(f"[ARM] ERREUR : joint{i+1}={val} hors limites [{lo}, {hi}]")
            return False
    return True


# ==========================================
# MOUVEMENTS
# ==========================================

def go_home(arm):
    print("[ARM] Retour home...")
    j1, j2, j3, j4, j5 = HOME_JOINTS
    arm.Arm_serial_servo_write6(j1, j2, j3, j4, j5, GRIPPER_OPEN, HOME_TIME)
    time.sleep(HOME_TIME / 1000.0 + 0.5)


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
    Sequence complete :
    1. Aller au-dessus de l'objet (z transit)
    2. Descendre au z de saisie
    3. Fermer le gripper
    4. Remonter
    5. Aller au-dessus du point de depot
    6. Deposer
    7. Ouvrir le gripper
    8. Retour home
    """

    Z_TRANSIT = PLACE_Z_LIFT    # hauteur de transit pick aussi

    seed = get_current_seed(arm, chain)

    # ---- 1. Transit au-dessus de l'objet ----
    print("\n[SEQ] Transit au-dessus de l'objet...")
    _, jd = compute_ik(chain, x_obj, y_obj, Z_TRANSIT, seed)
    open_gripper(arm)
    if not move_to(arm, jd):
        print("[SEQ] Position transit inaccessible, abandon.")
        go_home(arm)
        return False

    # ---- 2. Descente sur l'objet ----
    print("[SEQ] Descente sur l'objet...")
    seed = get_current_seed(arm, chain)
    _, jd = compute_ik(chain, x_obj, y_obj, Z_PICK, seed)
    if not move_to(arm, jd):
        print("[SEQ] Position pick inaccessible, abandon.")
        go_home(arm)
        return False

    # ---- 3. Saisie ----
    print("[SEQ] Saisie...")
    close_gripper(arm)

    # ---- 4. Remontee avec objet ----
    print("[SEQ] Remontee...")
    seed = get_current_seed(arm, chain)
    _, jd = compute_ik(chain, x_obj, y_obj, Z_TRANSIT, seed)
    move_to(arm, jd)

    # ---- 5. Transit vers le depot ----
    print("[SEQ] Deplacement vers depot...")
    seed = get_current_seed(arm, chain)
    _, jd = compute_ik(chain, PLACE_X, PLACE_Y, PLACE_Z_LIFT, seed)
    if not move_to(arm, jd, gripper=GRIPPER_CLOSED, duration=PLACE_TIME):
        print("[SEQ] Position depot transit inaccessible.")
        go_home(arm)
        return False

    # ---- 6. Descente depot ----
    print("[SEQ] Descente depot...")
    seed = get_current_seed(arm, chain)
    _, jd = compute_ik(chain, PLACE_X, PLACE_Y, PLACE_Z_DROP, seed)
    move_to(arm, jd, gripper=GRIPPER_CLOSED, duration=PLACE_TIME)

    # ---- 7. Relache ----
    print("[SEQ] Relache objet...")
    open_gripper(arm)

    # ---- 8. Retour home ----
    go_home(arm)

    print("[SEQ] Pick and place termine.")
    return True


# ==========================================
# CLIENT ZMQ
# ==========================================

def run_controller():
    chain = load_chain()
    arm   = Arm_Device()

    ctx    = zmq.Context()
    socket = ctx.socket(zmq.REQ)
    socket.connect(f"tcp://{ZMQ_HOST}:{ZMQ_PORT}")

    print(f"[ARM] Connecte au detecteur sur {ZMQ_HOST}:{ZMQ_PORT}")

    go_home(arm)

    try:
        while True:
            print("\n[ARM] Demande de detection...")
            socket.send_string("DETECT")

            resp = json.loads(socket.recv_string())

            if "error" in resp:
                print(f"[ARM] Detecteur : {resp['error']} — nouvelle tentative dans 2s")
                time.sleep(2.0)
                continue

            x_obj    = resp["x"]
            y_obj    = resp["y"]
            cls_name = resp["class"]

            print(f"[ARM] Objet recu : {cls_name} a ({x_obj:.4f}, {y_obj:.4f}) m")

            success = pick_and_place(arm, chain, x_obj, y_obj)

            if not success:
                print("[ARM] Echec pick and place, on continue...")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[ARM] Interruption manuelle.")
        socket.send_string("STOP")
        socket.recv_string()

    finally:
        go_home(arm)
        socket.close()
        ctx.term()
        print("[ARM] Arret propre.")


if __name__ == "__main__":
    run_controller()
