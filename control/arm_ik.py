"""
arm_ik.py
═══════════════════════════════════════════════════════════════════════════════
Cinématique Inverse (IK) — Bras robotique 5-DOF
Méthode : IK Numérique (Jacobien pseudo-inverse + Damped Least Squares)
          avec FK rigoureuse basée sur les transformations URDF complètes

Pourquoi numérique et pas analytique ?
──────────────────────────────────────
L'URDF de ce bras contient des offsets de rotation fixes sur chaque joint
(paramètre 'rpy'). En particulier, joint2 a rpy=[0, π/2, 0] ce qui change
complètement le plan de travail perçu par les joints suivants.
Ces offsets rendent la dérivation analytique (Al-Kashi) non triviale et
source d'erreurs importantes (jusqu'à 100mm testés). La méthode numérique
par Jacobien est donc plus fiable ici car elle utilise directement la FK
qui intègre tous ces offsets.

Structure du bras (5 joints, rotation autour de Z pour chaque joint) :
  Joint 1 : base rotative
  Joint 2 : épaule
  Joint 3 : coude
  Joint 4 : poignet pitch
  Joint 5 : poignet roll

Usage minimal :
    from arm_ik import compute_ik

    result = compute_ik(x=0.10, y=0.0, z=0.18)
    if result.success:
        print(result.as_degrees())   # angles en degrés → envoyer aux servos
        print(result.as_array())     # angles en radians
═══════════════════════════════════════════════════════════════════════════════
"""

import numpy as np
import warnings
from dataclasses import dataclass


# ─────────────────────────────────────────────────────────────────────────────
# PARAMÈTRES URDF DU BRAS
# ─────────────────────────────────────────────────────────────────────────────

# Chaque joint est défini par :
#   'xyz' : translation fixe (offset) du lien rigide qui précède ce joint (mètres)
#   'rpy' : rotation fixe (roll, pitch, yaw) de ce même lien rigide (radians)
#
# Ces valeurs sont extraites directement de l'URDF du bras.
# Elles représentent la géométrie FIXE (non articulée) entre deux joints.
# La rotation articulaire de chaque joint est toujours autour de l'axe Z local.

JOINTS_URDF = [
    {   # Joint 1 — Base rotative
        'xyz': [0.0,          0.0,          0.06605],
        'rpy': [-0.010805,    0.0,          0.0    ],
    },
    {   # Joint 2 — Epaule
        # rpy=[0, pi/2, 0] : le lien est incline a 90 degres autour de Y
        # C'est pourquoi une IK analytique simple echoue ici
        'xyz': [0.0,         -0.00031873,   0.04145],
        'rpy': [0.0,          np.pi / 2,    0.0    ],
    },
    {   # Joint 3 — Coude
        # xyz en X negatif : le segment part dans -X du frame local
        'xyz': [-0.08285,     0.0,          0.0    ],
        'rpy': [0.0,          0.0,          0.0    ],
    },
    {   # Joint 4 — Poignet pitch
        'xyz': [-0.08285,     0.0,          0.0    ],
        'rpy': [0.0,          0.0,          0.0083081],
    },
    {   # Joint 5 — Poignet roll / effecteur
        # rpy=[0, -pi/2, 0] : remise a plat du repere en bout de bras
        'xyz': [-0.07385,    -0.001,         0.0    ],
        'rpy': [0.0,         -np.pi / 2,    0.0    ],
    },
]

N_JOINTS = len(JOINTS_URDF)   # 5

# Limites articulaires (radians) extraites de l'URDF
JOINT_LIMITS = [
    (-np.pi / 2,  np.pi / 2),   # Joint 1
    (-np.pi / 2,  np.pi / 2),   # Joint 2
    (-np.pi / 2,  np.pi / 2),   # Joint 3
    (-np.pi / 2,  np.pi / 2),   # Joint 4
    (-np.pi / 2,  np.pi      ),  # Joint 5
]


# ─────────────────────────────────────────────────────────────────────────────
# STRUCTURE DE RETOUR
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class JointAngles:
    """
    Resultat retourne par compute_ik().

    Tous les angles sont stockes en RADIANS.
    Utilisez as_degrees() si vos servos attendent des degres.

    Attributs
    ---------
    theta1 ... theta5 : float
        Angles des joints 1 a 5 en radians.
    success : bool
        True si l'IK a converge avec une erreur acceptable.
    error_mm : float
        Erreur residuelle entre FK(angles trouves) et la cible (mm).
        Une bonne IK donne < 1 mm.
    message : str
        Description du resultat ou de la cause d'echec.
    """
    theta1:   float = 0.0
    theta2:   float = 0.0
    theta3:   float = 0.0
    theta4:   float = 0.0
    theta5:   float = 0.0
    success:  bool  = False
    error_mm: float = 999.0
    message:  str   = ""

    def as_array(self) -> np.ndarray:
        """Retourne les 5 angles en RADIANS sous forme de tableau numpy."""
        return np.array([self.theta1, self.theta2,
                         self.theta3, self.theta4, self.theta5])

    def as_degrees(self) -> np.ndarray:
        """Retourne les 5 angles convertis en DEGRES."""
        return np.degrees(self.as_array())

    def print_report(self):
        """Affiche un tableau lisible dans le terminal."""
        names = ["Base    ", "Epaule  ", "Coude   ", "Poignet ", "Effecteur"]
        print("\n+--------------------------------------------------+")
        print("|          RESULTAT IK -- bras 5-DOF               |")
        print("+-----------+------------+--------------------------+")
        print("|   Joint   |  Radians   |         Degres           |")
        print("+-----------+------------+--------------------------+")
        for name, rad in zip(names, self.as_array()):
            print(f"|  {name} |  {rad:+.4f}  |      {np.degrees(rad):+9.4f} deg      |")
        print("+-----------+------------+--------------------------+")
        status = "OK" if self.success else "ECHEC"
        print(f"|  Statut    : {status:<38s}|")
        print(f"|  Erreur FK : {self.error_mm:.4f} mm{' '*30}|")
        print(f"|  Info      : {self.message:<38s}|")
        print("+--------------------------------------------------+\n")


# ─────────────────────────────────────────────────────────────────────────────
# CINÉMATIQUE DIRECTE (FK)
# ─────────────────────────────────────────────────────────────────────────────

def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convertit les angles RPY en matrice de rotation 3x3.

    Convention : Rz(yaw) @ Ry(pitch) @ Rx(roll)
    C'est la convention standard utilisee dans les URDF ROS.
    """
    Rx = np.array([[1, 0,              0             ],
                   [0, np.cos(roll),  -np.sin(roll)  ],
                   [0, np.sin(roll),   np.cos(roll)  ]])
    Ry = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                   [ 0,             1, 0             ],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw),  np.cos(yaw), 0],
                   [0,            0,            1]])
    return Rz @ Ry @ Rx


def _rot_z(theta: float) -> np.ndarray:
    """
    Matrice de rotation articulaire autour de l'axe Z.

    Tous les joints de ce bras tournent autour de Z dans leur frame local.
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]])


def forward_kinematics(q: np.ndarray) -> tuple:
    """
    Cinematique directe complete basee sur les transformations URDF.

    Pour chaque joint i, la transformation totale est :
        T_i = T_{i-1} x T_fixe_i x T_articulaire_i(theta_i)
    ou :
        T_fixe_i      = translation(xyz_i) + rotation(rpy_i)  <- URDF, constant
        T_articulaire = rotation autour de Z de l'angle theta_i <- variable

    Paramètres
    ----------
    q : np.ndarray, shape (5,)
        Angles articulaires en radians [theta1, theta2, theta3, theta4, theta5].

    Retourne
    --------
    pos_effecteur : np.ndarray, shape (3,)
        Position (x, y, z) de l'effecteur en metres.
    frames : list of np.ndarray
        Positions (x, y, z) de chaque joint dans l'espace (utile pour debug).
    """
    # Matrice homogene 4x4 courante (part de l'identite = frame monde)
    T = np.eye(4)
    frames = []

    for jt, theta in zip(JOINTS_URDF, q):

        # -- Transformation fixe du lien rigide (offset URDF) ----------------
        # Chaque lien rigide a une orientation et une position de depart
        # definies dans l'URDF (independantes de l'angle articulaire)
        Tf = np.eye(4)
        Tf[:3, :3] = _rpy_to_rot(*jt['rpy'])   # orientation initiale du lien
        Tf[:3,  3] = jt['xyz']                  # decalage en translation
        T = T @ Tf

        # -- Rotation articulaire autour de Z du frame courant ---------------
        # Apres l'offset fixe, on applique la rotation du joint
        Tj = np.eye(4)
        Tj[:3, :3] = _rot_z(theta)
        T = T @ Tj

        # Enregistrer la position de ce joint pour reference
        frames.append(T[:3, 3].copy())

    # La position de l'effecteur est la translation de la matrice finale
    pos_effecteur = T[:3, 3]
    return pos_effecteur, frames


# ─────────────────────────────────────────────────────────────────────────────
# JACOBIEN NUMERIQUE
# ─────────────────────────────────────────────────────────────────────────────

def _jacobian(q: np.ndarray, eps: float = 1e-7) -> tuple:
    """
    Calcule le Jacobien de position par differences finies.

    Le Jacobien J (3 x N_JOINTS) relie les vitesses articulaires
    aux vitesses de l'effecteur :
        x_point_effecteur = J x q_point

    Chaque colonne J[:,i] = d(pos)/d(theta_i) est approchee par :
        J[:,i] ≈ (FK(q + eps*e_i) - FK(q)) / eps

    On utilise les differences finies plutot que le Jacobien geometrique
    analytique car les offsets RPY de l'URDF compliquent le calcul des
    axes de rotation dans le frame global.

    Parametres
    ----------
    q : np.ndarray, shape (5,)
        Configuration articulaire courante.
    eps : float
        Pas de perturbation (tres petit pour bonne precision).

    Retourne
    --------
    J : np.ndarray, shape (3, 5)
        Jacobien de position.
    pos0 : np.ndarray, shape (3,)
        Position de l'effecteur a la configuration q.
    """
    pos0, _ = forward_kinematics(q)
    J = np.zeros((3, N_JOINTS))

    for i in range(N_JOINTS):
        # Perturber le joint i d'un tout petit epsilon
        q_perturbe = q.copy()
        q_perturbe[i] += eps

        pos_perturbe, _ = forward_kinematics(q_perturbe)

        # Approximation de la derivee partielle par difference finie
        J[:, i] = (pos_perturbe - pos0) / eps

    return J, pos0


# ─────────────────────────────────────────────────────────────────────────────
# CINÉMATIQUE INVERSE — FONCTION PRINCIPALE
# ─────────────────────────────────────────────────────────────────────────────

def compute_ik(x: float, y: float, z: float,
               q_init: np.ndarray = None,
               n_restarts: int = 8,
               max_iter: int = 500,
               tol_mm: float = 1.0) -> JointAngles:
    """
    Calcule les angles articulaires pour atteindre la position (x, y, z).

    Methode : Damped Least Squares (DLS) avec multi-restarts.

    A chaque iteration :
        1. Calculer l'erreur : e = cible - FK(q)
        2. Calculer le Jacobien : J = d(FK)/d(q)
        3. Calculer la correction : delta_q = J^T (J J^T + lambda*I)^-1 * e
        4. Mettre a jour : q <- q + alpha * delta_q  (clippe aux limites)
        5. Repeter jusqu'a convergence ou max_iter atteint

    Le facteur d'amortissement lambda (lam) evite les instabilites numeriques
    quand le Jacobien est proche d'une singularite.

    Les multi-restarts permettent d'echapper aux minima locaux en testant
    plusieurs configurations initiales aleatoires en plus du q_init fourni.

    Parametres
    ----------
    x, y, z : float
        Position cible de l'effecteur en METRES.
    q_init : np.ndarray optionnel, shape (5,)
        Configuration initiale en radians.
        Si None, utilise la position home (tous angles a zero).
        Passer la configuration actuelle du bras ameliore la convergence
        et donne une solution proche de l'etat courant (continuite).
    n_restarts : int
        Nombre de departs aleatoires supplementaires (defaut 8).
        Augmenter si l'IK echoue souvent.
    max_iter : int
        Nombre maximum d'iterations par restart (defaut 500).
    tol_mm : float
        Tolerance d'erreur FK acceptable en millimetres (defaut 1.0 mm).

    Retourne
    --------
    JointAngles
        Objet contenant les 5 angles + statut + erreur FK.
        Verifiez TOUJOURS .success avant d'utiliser les angles.

    Exemple
    -------
    result = compute_ik(x=0.10, y=0.0, z=0.18)
    if result.success:
        radians = result.as_array()    # -> np.array([t1, t2, t3, t4, t5])
        degrees = result.as_degrees()  # -> np.array([d1, d2, d3, d4, d5])
        result.print_report()
    else:
        print(result.message)
    """
    target = np.array([x, y, z], dtype=float)
    tol    = tol_mm / 1000.0    # convertir mm -> metres pour calculs internes

    # -- Parametres du solveur ------------------------------------------------
    alpha = 0.4     # pas d'apprentissage (0 < alpha <= 1 ; trop grand = instable)
    lam   = 1e-4    # amortissement DLS (evite les singularites du Jacobien)

    # -- Construction de la liste des configurations de depart ----------------
    # Graine fixe : resultats reproductibles d'une execution a l'autre
    rng    = np.random.default_rng(seed=42)
    starts = []

    if q_init is not None:
        # Priorite au point de depart fourni (generalement la config actuelle)
        starts.append(np.array(q_init, dtype=float))

    # Toujours tester la position home (tous angles a zero)
    starts.append(np.zeros(N_JOINTS))

    # Completer avec des configurations aleatoires dans les limites articulaires
    for _ in range(n_restarts):
        q_rand = np.array([rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS])
        starts.append(q_rand)

    # -- Boucle principale : on teste chaque configuration de depart ----------
    best_q   = np.zeros(N_JOINTS)   # meilleure solution trouvee jusqu'ici
    best_err = np.inf               # erreur correspondante

    for q_start in starts:
        q = q_start.copy()

        # -- Descente de gradient iterative ------------------------------------
        for _ in range(max_iter):

            # Calcul du Jacobien et de la position courante
            J, pos_current = _jacobian(q)

            # Vecteur d'erreur : de la position courante vers la cible
            err_vec = target - pos_current
            err     = np.linalg.norm(err_vec)

            # Critere d'arret : on est assez proche de la cible
            if err < tol:
                break

            # -- Damped Least Squares : delta_q = J^T (J J^T + lambda*I)^-1 * e
            # Plus stable que la pseudo-inverse directe J+ = J^T (J J^T)^-1
            # Le terme lambda*I evite la division par zero pres des singularites
            A  = J @ J.T + lam * np.eye(3)
            dq = J.T @ np.linalg.solve(A, err_vec)

            # Mise a jour avec pas alpha
            q = q + alpha * dq

            # Respect des limites articulaires
            for i, (lo, hi) in enumerate(JOINT_LIMITS):
                q[i] = np.clip(q[i], lo, hi)

        # -- Evaluation de cette solution -------------------------------------
        pos_final, _ = forward_kinematics(q)
        err_final     = np.linalg.norm(pos_final - target)

        if err_final < best_err:
            best_err = err_final
            best_q   = q.copy()

        # Arret anticipe si on a deja une tres bonne solution
        if best_err < tol:
            break

    # -- Construction du resultat final ---------------------------------------
    result          = JointAngles()
    result.theta1   = float(best_q[0])
    result.theta2   = float(best_q[1])
    result.theta3   = float(best_q[2])
    result.theta4   = float(best_q[3])
    result.theta5   = float(best_q[4])
    result.error_mm = best_err * 1000.0

    if result.error_mm <= tol_mm:
        result.success = True
        result.message = f"IK convergee (erreur = {result.error_mm:.3f} mm)"
    else:
        result.success = False
        result.message = (f"IK non convergee : erreur = {result.error_mm:.2f} mm "
                          f"> tolerance = {tol_mm:.1f} mm")
        warnings.warn(f"[IK] {result.message}", UserWarning, stacklevel=2)

    return result


# ─────────────────────────────────────────────────────────────────────────────
# TEST EN LIGNE DE COMMANDE
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  arm_ik.py -- Tests de la cinematique inverse")
    print("=" * 60)

    """ test_cases = [
        # (  x,     y,     z,   description                    )
        ( 0.10,  0.00,  0.18,  "Position standard"            ),
        ( 0.12, -0.06,  0.05,  "Proche du sol, decalee"       ),
        ( 0.08,  0.08,  0.14,  "Position laterale"            ),
        ( 0.15,  0.00,  0.20,  "Allongee vers l'avant"        ),
        ( 0.00,  0.10,  0.15,  "Laterale pure (y>0)"          ),
        ( 0.50,  0.00,  0.20,  "Hors portee -- doit echouer"  ),
    ]

    for x, y, z, label in test_cases:
        print(f"\n{'-' * 55}")
        print(f"  Test : {label}")
        print(f"  Cible : ({x}, {y}, {z}) m")
        result = compute_ik(x, y, z)
        result.print_report()"""

    result = compute_ik(0.0071,0.2068,0.108)
    result.print_report()
