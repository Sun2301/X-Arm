#!/usr/bin/env python3
"""
detector.py
Serveur ZMQ (REP) de detection.
Attend une requete "DETECT" du controleur, capture des frames jusqu'a trouver
un objet, puis retourne ses coordonnees reelles en metres.

ZMQ pattern : REP (serveur)
"""

import cv2
import numpy as np
import zmq
import json
import time
from ultralytics import YOLO
from pathlib import Path

# ==========================================
# CONFIGURATION
# ==========================================

CAMERA_INDEX = 1
MODEL_PATH   = Path("/home/pi/X-Arm/computer-vision/Rom/models/best.pt")
HOMOGRAPHY   = Path("homography.npy")
ZMQ_PORT     = 5555
YOLO_CONF    = 0.5

HSV_LOWER = np.array([89, 146,   0])
HSV_UPPER = np.array([99, 255, 255])

CLASS_NAMES = {0: "Blue", 1: "Green", 2: "Red"}

# ==========================================
# DETECTION
# ==========================================

def get_top_face_centroid(frame, x1, y1, x2, y2):
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None

    hsv  = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    cnt = max(contours, key=cv2.contourArea)
    M   = cv2.moments(cnt)
    if M["m00"] == 0:
        return None

    cx = x1 + int(M["m10"] / M["m00"])
    cy = y1 + int(M["m01"] / M["m00"])
    return cx, cy


def pixel_to_real(cx, cy, H):
    pt    = np.array([[cx, cy]], dtype=np.float32).reshape(-1, 1, 2)
    pt_r  = cv2.perspectiveTransform(pt, H)
    x_r, y_r = pt_r[0][0]
    return float(x_r), float(y_r)


def detect_one(cap, model, H):
    """
    Capture des frames jusqu'a trouver un objet avec confiance suffisante.
    Retourne un dict {x, y, class_name} ou None si timeout.
    """
    deadline = time.time() + 10.0   # timeout 10 secondes

    while time.time() < deadline:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, verbose=False)

        for result in results:
            for box in result.boxes:
                conf   = float(box.conf[0])
                if conf < YOLO_CONF:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id   = int(box.cls[0])
                cls_name = CLASS_NAMES.get(cls_id, "Unknown")

                centroid = get_top_face_centroid(frame, x1, y1, x2, y2)

                if centroid:
                    cx, cy = centroid
                else:
                    cx = int((x1 + x2) / 2)
                    cy = int(y1 + (y2 - y1) / 3)

                x_real, y_real = pixel_to_real(cx, cy, H)

                print(f"[DETECT] {cls_name} conf={conf:.2f} "
                      f"pixel=({cx},{cy}) reel=({x_real:.4f}, {y_real:.4f}) m")

                return {"x": x_real, "y": y_real, "class": cls_name}

    print("[DETECT] Timeout : aucun objet detecte")
    return None


# ==========================================
# SERVEUR ZMQ
# ==========================================

def run_server():
    # --- modele YOLO ---
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"Modele YOLO introuvable : {MODEL_PATH}")
    model = YOLO(str(MODEL_PATH))

    # --- homographie ---
    if not HOMOGRAPHY.exists():
        raise FileNotFoundError(f"Homographie introuvable : {HOMOGRAPHY}")
    H = np.load(str(HOMOGRAPHY))

    # --- camera ---
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Impossible d'ouvrir la camera index={CAMERA_INDEX}")

    # --- ZMQ ---
    ctx    = zmq.Context()
    socket = ctx.socket(zmq.REP)
    socket.bind(f"tcp://*:{ZMQ_PORT}")

    print(f"[DETECTOR] Serveur ZMQ actif sur port {ZMQ_PORT}")
    print("[DETECTOR] En attente de requetes...")

    try:
        while True:
            msg = socket.recv_string()

            if msg == "DETECT":
                result = detect_one(cap, model, H)
                if result:
                    socket.send_string(json.dumps(result))
                else:
                    socket.send_string(json.dumps({"error": "no_object"}))

            elif msg == "STOP":
                socket.send_string(json.dumps({"status": "stopped"}))
                break

            else:
                socket.send_string(json.dumps({"error": "unknown_command"}))

    finally:
        cap.release()
        socket.close()
        ctx.term()
        print("[DETECTOR] Arret propre.")


if __name__ == "__main__":
    run_server()
