
# 1) Objectifs du groupe « contrôle »

* Traduire la position détectée par la vision en coordonnées exploitables par le bras.
* Calculer et exécuter des trajectoires sûres (pick & place) via la cinématique et le SDK.
* Assurer la robustesse, sécurité et observabilité (logs, UI, tests).

# 2) Liste de tâches 

## A. Calibration caméra ↔ robot

* Mesurer / estimer la transformée extrinsèque entre repère caméra et repère base robot.
* Implémenter utilitaire de calibration (capture images/couples points 2D–3D, solvePnP, validation).
* Livrable : script `calibrate_camera.py` + fichier de paramètres `camera_to_base.json` + rapport de calibration (erreur reprojection).
* Critère d’acceptation : erreur de reprojection moyenne documentée < seuil raisonnable (ex. < 5 px). adapter au hardware.

## B. Modèles cinématique

* Implémenter **cinématique directe** (forward) et **inverse** (IK) pour xArm (structure DH ou équivalent).
* Fournir solvers : analytique (si possible) + fallback numérique (Newton / Levenberg-Marquardt).
* Livrable : module `kinematics.py` (fonctions `forward(angles)`, `inverse(x,y,z,orientation,seed)`), tests unitaires.
* Critère d’acceptation : pour plusieurs points test, `forward(inverse(p)) ≈ p` (erreur position < quelques mm).
* Commentaire : C'est ce que j'ai commencé à explorer, vous pourez vous en servir comme base puis l'améliorer vu que moi je suis parti sur de l'analytique, ce qui peut être limitant. 

## C. Contrôle bas-niveau & interface SDK

* Wrapper propre autour du SDK xArm (`xarm_controller.py`) exposant fonctions haut niveau :

  * `connect()`, `go_to_joint(angles, speed)`, `go_to_pose(x,y,z,roll,pitch,yaw,speed)`, `open_gripper()`, `close_gripper()`, `stop_emergency()`.
* Livrable : module + README d’utilisation + scripts de test manuel (`test_move.py`).
* Critère : commandes simples exécutables et retournant état/erreur.
* Commentaire : Vous aviez commencer avec le script test du bras donc vous pourrez vous basez dessus pour intégrer la cinématique et les autres fonctions du bras. 

## D. Planification de trajectoire & génération de trajectoire

* Planner simple (trajet point-à-point) et interpolation (linéaire en cartésien, spline en joints).
* Gestion des vitesses/accélérations, limites joint, avoidance basique (zones interdites).
* Livrable : `trajectory.py` avec visualisation (position dans le temps) + paramètres (vitesse, accel).
* Critère : exécution fluide sans dépassement de limites.

## E. Boucle de contrôle et synchronisation vision-action

* Réception de la position détectée par le module vision (format standard).
* Filtre/validation des mesures (moyenne, median filter), transformation via calibration.
* Orchestrateur principal `controller_main.py` : état machine (IDLE → APPROACH → GRASP → LIFT → MOVE → RELEASE → HOME).
* Livrable : orchestrateur + diagramme d’états + logs d’exécution.
* Critère : transitions robustes et possibilité d’annuler / reprendre.

## F. Commande pince (end-effector)

* Commandes de préhension, détection d’échec (courant moteur, position).
* Paramètres de force / durée / small retries.
* Livrable : `gripper_controller.py` + tests de robustesse sur plusieurs objets.
* Commentaire : parallèle à la tâche précédente sur la composante test 

## G. Simulation & tests hors matériel

* Créer une simulation (Gazebo / PyBullet ou simple simulateur kinematique) pour développer sans casse.
* Livrable : scène simulée + scripts de test automatisés.
* Critère : tests CI qui passent en simulation.
* Commentaire : Chacune des étapes précédente dois être valider par simulation avant de passer au test sur le bras. On en a qu'un seul merci d'en prendre soin 🥲. 

## H. Tests, validation et jeux de tests

* Scénarios de test: position cible au centre, bord, proche des limites, objets déplacés, fausse détection.
* Écrire tests unitaires, tests d’intégration (sur simulateur) et checklists pour tests manuels sur robot réel.
* Livrable : dossier `tests/` + rapport de validation.

## I. Observabilité & UI

* UI minimal (web ou OpenCV window) montrant: flux caméra, bbox détectée (fournie par vision), position cible, trajectoire planifiée, status robot.
* Logs structurés, fichiers CSV/JSON d’exécution.
* Livrable : `dashboard.py` + guide d’utilisation.
* Commentaire : C'est non optionnel on va pas remettre un code au gens pour tester il faut vraiment un interface pour un showcase interactif.

## J. Sécurité et robustesse

* Implémenter : arrêt d’urgence logiciel, timeouts de commande, vérification collision simple (zones interdites), soft limits.
* Livrable : doc « Safety procedures » + code (emergency_stop handler).
* Critère : test d’arrêt immédiat fonctionnel.

## K. Intégration continue & packaging

* Scripts d’installation `requirements.txt`, `venv` instructions, container Docker optionnel.
* Livrable : `README`, `setup.sh`, et packaging minimal pour démo.

# 3) Tests & critères d’acceptation

* Position finale de la pince après `go_to_pose` : erreur ≤ 10 mm (à ajuster selon hardware peut être que c'est trop stricte).
* Taux de réussite pick&place sur 10 essais ≥ 9/10.
* Temps d’arrêt complet après emergency_stop ≤ 500 ms .

# 4) Structure de dépôt recommandée

```
/control/
  README.md
  requirements.txt
  /src/
    xarm_controller.py
    kinematics.py
    trajectory.py
    gripper_controller.py
    controller_main.py
    calibration/
      calibrate_camera.py
      camera_to_base.json
    utils.py
  /tests/
  /sim/
  dashboard.py
  run_demo.py
  docs/
    safety.md
    test_report.md
```
