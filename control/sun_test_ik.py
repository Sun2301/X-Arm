#!/usr/bin/env python3
# test_ik_solver.py
import numpy as np
import ikpy.chain

RA2DE = 180.0 / np.pi


def load_chain(urdf_path):
    chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path,
        base_elements=["base_link"],
    )
    return chain


def compute_ik(chain, x, y, z, verbose=True):
    initial_position = [0.0] * len(chain.links)

    result = chain.inverse_kinematics(
        target_position=[x, y, z],
        initial_position=initial_position,
        orientation_mode=None,
    )

    joints_rad  = result[1:6]
    joints_deg  = [round(j * RA2DE + 90.0, 2) for j in joints_rad]

    if verbose:
        print(f"\nCible          : x={x:.4f}  y={y:.4f}  z={z:.4f}")
        print(f"Angles (rad)   : {[round(j, 4) for j in joints_rad]}")
        print(f"Angles servo   : j1={joints_deg[0]}  j2={joints_deg[1]}  "
              f"j3={joints_deg[2]}  j4={joints_deg[3]}  j5={joints_deg[4]}")

        # verification FK
        full = [0.0] + list(joints_rad)
        T    = chain.forward_kinematics(full)
        pos  = T[:3, 3]
        err  = np.linalg.norm(pos - np.array([x, y, z]))
        print(f"FK result      : x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}")
        print(f"Erreur (m)     : {err:.5f}")

    return joints_rad, joints_deg


def make_target(x, y, h):
    """
    x, y : coordonnees reelles retournees par l'homographie (metres)
    h     : hauteur de l'objet (metres), z fixe a h/2
    """
    z =  h   #h / 2.0
    return x, y, z


if __name__ == "__main__":
    chain = load_chain("/home/mr_sun/X-Arm/control/arm_info/urdf/arm.urdf")

    # remplace par tes vraies coordonnees homographie + hauteur objet
    """tests = [
        (0.00, 0.20, 0.04),
        (0.10, 0.15, 0.04),
        (-0.05, 0.18, 0.06),
    ]

    for x_obj, y_obj, h_obj in tests:
        x, y, z = make_target(x_obj, y_obj, h_obj)
        compute_ik(chain, x, y, z)
        print("-" * 50)"""
    
    x, y, z = make_target(-0.0006,0.2033 ,0.08)
    compute_ik(chain, x, y, z)
    print("-" * 50)