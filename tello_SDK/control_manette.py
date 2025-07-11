import pygame
from djitellopy import Tello
import time

# Initialiser pygame
pygame.init()

# Initialiser la manette
pygame.joystick.init()    
print("Programme sponsorisé par l'ENSTA")


# Initialiser le drone
tello = Tello()
tello.connect()
print(f"Drone connecté. Niveau de batterie : {tello.get_battery()}%")
tello.streamon()

# Commandes du drone via les boutons de la manette
def control_drone():

    # Vérifier combien de manettes sont connectées
    num_joysticks = pygame.joystick.get_count()
    if num_joysticks == 0:
        print("Aucune manette détectée !")
        pygame.quit()
        exit()

    pygame.event.pump()  # Récupère les événements de la manette
    print("Pump")
    # Lire les axes (stick analogique)
    for i in range(joystick.get_numaxes()):
        axis_value = joystick.get_axis(i)
        print(f"Axis {i}: {axis_value:.2f}")
   
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            tello.land()  # Atterrissage avant de quitter
            exit()
        
        # Contrôle de base : Déplacement
        if event.type == pygame.JOYAXISMOTION:
            # Contrôle du joystick gauche pour mouvement avant/arrière, gauche/droite
            left_stick_x = pygame.joystick.Joystick(0).get_axis(0)  # Gauche/Droite
            left_stick_y = pygame.joystick.Joystick(0).get_axis(1)  # Avant/Arrière
            right_stick_x = pygame.joystick.Joystick(0).get_axis(2)  # Rotation (Yaw)
            right_stick_y = pygame.joystick.Joystick(0).get_axis(3)  # Monter/Descendre


            # Envoi des commandes
            if left_stick_y < -0.1:  # Avancer
                tello.forward(20)
            elif left_stick_y > 0.1:  # Reculer
                tello.back(20)

            if left_stick_x > 0.1:  # Droite
                tello.right(20)
            elif left_stick_x < -0.1:  # Gauche
                tello.left(20)

            if right_stick_x > 0.1:  # Rotation horaire
                tello.cw(10)
            elif right_stick_x < -0.1:  # Rotation antihoraire
                tello.ccw(10)
            
            if right_stick_y < -0.1:
                tello.up(10)
            elif right_stick_y > 0.1:
                tello.down(10)

        # Contrôle des boutons de la manette
        if event.type == pygame.JOYBUTTONDOWN:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()

            # Sélectionner la première manette connectée
            print(f"Manette détectée : {joystick.get_name()}")
            
            # Bouton pour décoller
            if joystick.get_button(0):  # Button A (par exemple)
                print("décollage")
                tello.takeoff()

            # Bouton pour atterrir
            elif joystick.get_button(1):  # Button B (par exemple)
                tello.land()

            # Bouton pour activer le flip
            elif joystick.get_button(2):  # Button X (par exemple)
                tello.flip('f')  # Flip avant

            # Bouton pour activer l'urgence
            elif joystick.get_button(3):  # Button Y (par exemple)
                tello.emergency()

# Boucle de contrôle
try:
    print("Start")
    while True:
        control_drone()
        time.sleep(0.1)  # Pour limiter l'utilisation du processeur

except KeyboardInterrupt:
    print("Fin du programme.")
    tello.land()
    pygame.quit()

