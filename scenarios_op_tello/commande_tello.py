import threading
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import json
from djitellopy import Tello
from ObstacleDetector import ObstacleDetector

class TelloNavigator:
    def __init__(self, waypoint_file, video_path):
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()
        self.video_path = video_path
        self.frame_width, self.frame_height = 640, 480

        self.orb = cv2.ORB_create()
        self.prev_frame = None

        # Chargement des waypoints
        with open(waypoint_file, 'r') as f:
            data = json.load(f)
            self.waypoints = data["wp"]

        # Initialisation de l'enregistrement vidéo
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(self.video_path, fourcc, 20.0, (self.frame_width, self.frame_height))

        # Initialisation Matplotlib
        self.fig, self.ax = plt.subplots()
        self.img = self.ax.imshow(np.zeros((self.frame_height, self.frame_width, 3), dtype=np.uint8))
        plt.axis("off")

        # Variable pour capturer les images du drone
        self.frame = None
        self.lock = threading.Lock()  # Lock pour éviter les accès concurrents

    def update_frame(self):
        """Capture les frames du drone dans un thread secondaire."""
        while True:
            frame = self.tello.get_frame_read().frame
            if frame is None:
                print("⚠️ Frame non reçue, attente...")
                continue
            # Enregistrement vidéo
            frame_resized = cv2.resize(frame, (self.frame_width, self.frame_height))
            frame_bgr = cv2.cvtColor(frame_resized, cv2.COLOR_RGB2BGR)
            self.out.write(frame_bgr)

            # Mise à jour de la frame avec verrou pour éviter les accès concurrents
            with self.lock:
                self.frame = frame
            time.sleep(0.03)  # Petite pause pour éviter de surcharger la capture

    def detect_obstacle(self, frame):
        """Détection d'obstacle avec ORB et expansion de caractéristiques."""
        if self.prev_frame is None:
            self.prev_frame = frame
            return False  # Pas assez d'info pour détecter un obstacle

        # Extraction des points clés et descripteurs
        kp1, des1 = self.orb.detectAndCompute(self.prev_frame, None)
        kp2, des2 = self.orb.detectAndCompute(frame, None)

        if des1 is None or des2 is None:
            self.prev_frame = frame
            return False  # Impossible de détecter un obstacle

        # Correspondances avec Brute-Force Matcher
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        # Détection d'expansion des points caractéristiques
        detector = ObstacleDetector(frame, self.prev_frame, matches, kp1, kp2)
        detector.confirm_scale()

        # Met à jour le frame précédent pour la prochaine détection
        self.prev_frame = frame

        # Si expansion significative détectée, il y a un obstacle
        return len(detector.matches) > 10  # Ajustez le seuil selon vos besoins


    def navigate(self):
        """Exécute le parcours selon les waypoints."""
        print(f"Niveau de batterie : {self.tello.get_battery()}%")
        self.tello.takeoff()
        time.sleep(2)

        self.tello.send_command_without_return("rc 0 0 0 0")
        time.sleep(1)

        try:
            for wp in self.waypoints:
                dist_cm, angle_deg = wp["dist_cm"], wp["angle_deg"]

                # Rotation
                time.sleep(1)  # Attendre 1 seconde avant de tenter une autre commande
                response = self.tello.send_command_with_return(f"cw {angle_deg}")
                if "error" in response:
                    print(f"⚠️ Erreur de rotation ({response}), tentative avec ccw")
                    response = self.tello.send_command_with_return(f"ccw {angle_deg}")
                if "error" in response:
                    print(f"🚨 Rotation impossible ({response}), atterrissage d'urgence!")
                    self.tello.land()
                    break

                # Détection d'obstacle
                if self.frame is not None and self.detect_obstacle(self.frame):
                    print("🚧 Obstacle détecté! Tentative d'évitement...")
                    self.tello.move_up(20)
                    time.sleep(1)

                    if self.frame is not None and self.detect_obstacle(self.frame):
                        print("↔️ Déplacement latéral pour éviter obstacle...")
                        self.tello.move_right(30)
                        time.sleep(1)

                # Déplacement
                self.tello.move_forward(dist_cm)

        except KeyboardInterrupt:
            print("⛔ Arrêt d'urgence!")
            self.tello.send_command_with_return("emergency")

        # Atterrissage et libération des ressources
        self.tello.land()
        self.cleanup()

    def cleanup(self):
        """Ferme proprement les ressources vidéo et OpenCV."""
        self.out.release()
        cv2.destroyAllWindows()

def main():
    waypoint_file = "pfe_2025/scenarios_op_tello/waypoint.json"
    video_path = "pfe_2025/scenarios_op_tello/recorded_video.avi"

    drone = TelloNavigator(waypoint_file, video_path)

    # Créer un thread pour capturer les frames du drone
    video_thread = threading.Thread(target=drone.update_frame)
    video_thread.daemon = True  # Ferme automatiquement le thread à la fin du programme
    video_thread.start()

    # Créer un thread pour naviguer dans le scénario
    navigation_thread = threading.Thread(target=drone.navigate)
    navigation_thread.daemon = True
    navigation_thread.start()

    # Créer un objet animation pour afficher les frames capturées
    def update_plot(frame):
        # Utiliser un verrou pour garantir l'accès sécurisé à la frame
        with drone.lock:
            if drone.frame is not None:
                frame_rgb = cv2.cvtColor(drone.frame, cv2.COLOR_BGR2RGB)
                drone.img.set_data(frame_rgb)
        return [drone.img]

    ani = animation.FuncAnimation(drone.fig, update_plot, interval=50)

    # Pour garder la fenêtre ouverte jusqu'à ce que le programme soit terminé
    plt.show()

if __name__ == "__main__":
    main()
