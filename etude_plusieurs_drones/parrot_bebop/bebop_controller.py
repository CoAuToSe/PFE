import pygame
from pyparrot.Bebop import Bebop

# Initialisation du drone
bebop = Bebop(drone_type="Bebop")
print("Connexion au Bebop...")
if not bebop.connect(10):
    print("Échec de connexion au drone.")
    exit()

print("Connexion réussie !")
bebop.smart_sleep(2)
bebop.ask_for_state_update()

# Initialisation de pygame pour la manette
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Aucune manette détectée.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Manette détectée : {joystick.get_name()}")

running = True
decollage = False

while running:
    pygame.event.pump()
    

    if decollage == True :
        # Joystick axes
        roll = joystick.get_axis(0) * 100   # Gauche/Droite
        pitch = -joystick.get_axis(1) * 100 # Avant/Arrière
        yaw = joystick.get_axis(2) * 100    # Rotation
        gaz = -joystick.get_axis(3) * 50   # Monter/Descendre
        
        # Appliquer les commandes au drone
        print("Attention je bouge !")
        bebop.fly_direct(roll=roll, pitch=pitch, yaw=yaw, vertical_movement=gaz, duration=0.1)
    
    # Bouton A (Bouton 0) -> Décollage
    if joystick.get_button(0) and not decollage:
        print("Décollage !")
        bebop.safe_takeoff(5)
        decollage = True
    
    # Bouton B (Bouton 1) -> Atterrissage
    if joystick.get_button(1) and decollage:
        print("Atterissage !")
        bebop.safe_land(5)
        decollage = False
    
    # Bouton Start (Bouton 7) -> Quitter
    if joystick.get_button(7):
        print("Arret total !")
        running = False
        break

    pygame.time.wait(50)  # Petite pause pour éviter une surcharge CPU

# Arrêt du drone
bebop.safe_land(5)
bebop.disconnect()
print("Déconnexion du drone.")
