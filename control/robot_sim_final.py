import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class RobotController:
    def __init__(self):
        # --- Paramètres Géométriques (cm) ---
        # Squelette: Base, Humerus, Avant-bras
        self.links = {
            'l1': 10.0, 'l2': 15.0, 'l3': 15.0
        }
        
        # État actuel : 3 angles en radians
        self.current_angles = np.zeros(3)
        
        # Historique pour tracer le chemin parcouru (visualisation)
        self.trace_x = []
        self.trace_y = []
        self.trace_z = []

        # Configuration de la fenêtre 3D
        plt.ion()
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()

    def setup_plot(self):
        """Configure les limites et labels du graphique"""
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        limit = 40
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([0, 50])
        self.ax.set_title("Simulation Bras Robotique - Club IA")

    def get_rotation_matrix(self, axis, theta):
        c, s = np.cos(theta), np.sin(theta)
        if axis == 'x': return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        elif axis == 'y': return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        elif axis == 'z': return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def forward_kinematics(self, angles):
        """Calcule la position de chaque articulation pour l'affichage"""
        t1, t2, t3= angles
        l = self.links
        
        points = []
        p = np.array([0, 0, 0]); points.append(p) # Base au sol
        
        # Transformations successives
        R1 = self.get_rotation_matrix('z', t1)
        p = p + np.array([0, 0, l['l1']]); points.append(p) # Haut de base (J2)
        
        R2 = np.dot(R1, self.get_rotation_matrix('y', t2))
        p = p + np.dot(R2, np.array([0, 0, l['l2']])); points.append(p) # Coude (J3)
        
        R3 = np.dot(R2, self.get_rotation_matrix('y', t3))
        p = p + np.dot(R3, np.array([0, 0, l['l3']])); points.append(p) # Poignet 1 (J4)

        return np.array(points)

    def inverse_kinematics(self, x, y, z):
        """
        Calcule les angles pour atteindre (x,y,z).
        """
        l = self.links
        
        # 1. Base (J1)
        theta1 = np.arctan2(y, x)
        
        # 2. Paramètres Al-Kashi
        r = np.sqrt(x**2 + y**2) # Distance au sol
        z_local = z - l['l1'] # Hauteur relative à l'épaule
        D = np.sqrt(r**2 + z_local**2) # Distance Epaule -> Poignet virtuel
        
        if D > (l['l2'] + l['l3']): return None # Hors de portée

        # 3. Coude (J3)
        cos_elbw = (l['l2']**2 + l['l3']**2 - D**2) / (2 * l['l2'] * l['l3'])
        theta3 = (np.pi - np.arccos(np.clip(cos_elbw, -1, 1)))

        # 4. Epaule (J2)
        alpha = np.arctan2(z_local, r)
        cos_shldr = (l['l2']**2 + D**2 - l['l3']**2) / (2 * l['l2'] * D)
        beta = np.arccos(np.clip(cos_shldr, -1, 1))
        theta2 = (np.pi/2 - (alpha + beta)) # Correction pour repère vertical


        return np.array([theta1, theta2, theta3])

    def update_display(self, angles, target_pos=None):
        """Met à jour le graphique"""
        self.ax.cla() # Nettoyer
        self.setup_plot()
        
        # Calculer la forme du robot
        points = self.forward_kinematics(angles)
        
        # Tracer le robot
        self.ax.plot(points[:,0], points[:,1], points[:,2], '-o', linewidth=4, color='#2c3e50', label='Bras')
        self.ax.scatter(points[-1,0], points[-1,1], points[-1,2], s=100, c='orange', label='Pince')
        
        # Tracer la trace du mouvement
        self.trace_x.append(points[-1,0])
        self.trace_y.append(points[-1,1])
        self.trace_z.append(points[-1,2])
        # On garde les 50 derniers points pour ne pas surcharger
        if len(self.trace_x) > 50: 
            self.trace_x.pop(0); self.trace_y.pop(0); self.trace_z.pop(0)
        self.ax.plot(self.trace_x, self.trace_y, self.trace_z, '--', color='green', linewidth=1, alpha=0.5)

        # Tracer la cible si fournie
        if target_pos is not None:
            self.ax.scatter([target_pos[0]], [target_pos[1]], [target_pos[2]], c='red', marker='x', s=100, label='Cible')

        plt.legend()
        plt.draw()
        plt.pause(0.01) # Petite pause pour l'animation

    def goTo(self, x, y, z, duration=2.0):
        """
        FONCTION PRINCIPALE DE CONTRÔLE
        Déplace le robot de sa position actuelle vers (x,y,z) en 'duration' secondes.
        """
        print(f"Commande reçue : goTo({x}, {y}, {z})")
        
        # 1. Calculer les angles cibles (Inverse Kinematics)
        target_angles = self.inverse_kinematics(x, y, z)
        
        if target_angles is None:
            print("ERREUR : Cible hors de portée !")
            return False
        
        # 2. Générer la trajectoire (Interpolation)
        # Nombre d'étapes basé sur la durée (ex: 30 images par seconde)
        steps = int(duration * 30)
        start_angles = self.current_angles
        
        # np.linspace crée une transition fluide pour chaque angle
        trajectory = np.linspace(start_angles, target_angles, steps)
        
        # 3. Exécuter le mouvement
        for angles_step in trajectory:
            self.current_angles = angles_step # Mettre à jour l'état interne
            self.update_display(angles_step, target_pos=(x,y,z))
        
        print(f"Arrivé à destination : {x, y, z}")
        return True

# --- SCÉNARIO DE DÉMONSTRATION ---
if __name__ == "__main__":
    bot = RobotController()
    
    # Séquence de mouvements type "Pick and Place"
    # 1. Position de repos haute
    bot.goTo(15, 0, 20, duration=1.5)
    time.sleep(0.5)
    
    # 2. Descendre chercher l'objet (droite)
    bot.goTo(20, -10, 2, duration=1.5) # Z=2 (proche du sol)
    time.sleep(0.5) # Simulation prise objet (fermeture pince)
    
    # 3. Remonter
    bot.goTo(20, -10, 15, duration=1.5)
    
    # 4. Aller vers la zone de dépôt (gauche)
    bot.goTo(15, 15, 15, duration=2.0)
    
    # 5. Déposer
    bot.goTo(15, 15, 2, duration=1.5)
    time.sleep(1.0) # Ouverture pince
    
    # 6. Retour maison
    bot.goTo(10, 0, 30, duration=2.0)
    
    print("Démonstration terminée.")
    plt.ioff()
    plt.show()