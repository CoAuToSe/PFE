import cv2
import matplotlib.pyplot as plt
import time
from djitellopy import Tello

# Initialisation du drone
tello = Tello()
tello.connect()
time.sleep(2)  # Laisser le temps à la connexion de s'établir
print(f"Niveau de batterie : {tello.get_battery()}%")
tello.streamon()
time.sleep(2)  # Attendre que le flux démarre

# Initialisation de Matplotlib
plt.ion()  # Mode interactif
fig, ax = plt.subplots()
img = ax.imshow(cv2.cvtColor(tello.get_frame_read().frame, cv2.COLOR_BGR2RGB))
plt.axis("off")  # Cacher les axes

try:
    while True:
        frame = tello.get_frame_read().frame
        if frame is None:
            print("⚠️ Frame non reçue, attente...")
            continue

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Conversion BGR -> RGB
        img.set_data(frame)  # Mise à jour de l'image

        plt.pause(0.05)  # Pause pour rafraîchir l'affichage

except KeyboardInterrupt:
    print("Arrêt demandé par l'utilisateur.")

finally:
    tello.streamoff()
    tello.end()
    plt.ioff()
    plt.close(fig)  # Fermer proprement la fenêtre
