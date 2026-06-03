#!/usr/bin/env python3
"""
main.py
Lance le detecteur et le controleur dans deux processus separes.
Le detecteur demarre en premier, le controleur attend qu'il soit pret.

Usage :
    python3 main.py
    Ctrl+C pour arreter proprement.
"""

import subprocess
import sys
import time
import signal
import os

DETECTOR_SCRIPT    = "detector.py"
CONTROLLER_SCRIPT  = "arm_controller.py"
DETECTOR_WARMUP    = 3.0   # secondes d'attente avant de lancer le controleur


def main():
    procs = []

    def shutdown(sig=None, frame=None):
        print("\n[MAIN] Arret en cours...")
        for p in procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        time.sleep(2.0)
        for p in procs:
            if p.poll() is None:
                p.terminate()
        print("[MAIN] Arret propre.")
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    python = sys.executable

    # ---- 1. Lancer le detecteur ----
    print("[MAIN] Demarrage du detecteur...")
    det_proc = subprocess.Popen(
        [python, DETECTOR_SCRIPT],
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    procs.append(det_proc)

    # ---- 2. Attendre que le serveur ZMQ soit pret ----
    print(f"[MAIN] Attente {DETECTOR_WARMUP}s (demarrage serveur ZMQ)...")
    time.sleep(DETECTOR_WARMUP)

    if det_proc.poll() is not None:
        print("[MAIN] Le detecteur s'est arrete prematurement. Verifier les logs.")
        sys.exit(1)

    # ---- 3. Lancer le controleur ----
    print("[MAIN] Demarrage du controleur...")
    ctrl_proc = subprocess.Popen(
        [python, CONTROLLER_SCRIPT],
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    procs.append(ctrl_proc)

    # ---- 4. Surveiller les deux processus ----
    print("[MAIN] Systeme actif. Ctrl+C pour arreter.\n")
    while True:
        time.sleep(1.0)

        if det_proc.poll() is not None:
            print("[MAIN] Le detecteur s'est arrete de facon inattendue.")
            shutdown()

        if ctrl_proc.poll() is not None:
            print("[MAIN] Le controleur s'est arrete de facon inattendue.")
            shutdown()


if __name__ == "__main__":
    main()
