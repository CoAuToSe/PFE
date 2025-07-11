import pygame
import time

# Initialiser pygame
pygame.init()

# Initialiser la manette
pygame.joystick.init()

# Vérifier combien de manettes sont connectées
num_joysticks = pygame.joystick.get_count()
if num_joysticks == 0:
    print("Aucune manette détectée !")
    pygame.quit()
    exit()

# Sélectionner la première manette connectée
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Manette détectée : {joystick.get_name()}")

# Fonction pour afficher l'état des boutons et axes
def detect_buttons_and_axes():
    while True:
        pygame.event.pump()  # Récupère les événements de la manette
        
        # Lire les axes (stick analogique)
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            print(f"Axis {i}: {axis_value:.2f}")
        
        # Lire les événements de boutons
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:  # Un bouton a été pressé
                print(f"Button {event.button} pressed.")
            elif event.type == pygame.JOYBUTTONUP:  # Un bouton a été relâché
                print(f"Button {event.button} released.")
        
        time.sleep(0.1)  # Met en pause pour éviter de surcharger la sortie

# Lancer la détection
try:
    detect_buttons_and_axes()
except KeyboardInterrupt:
    print("\nDétection terminée.")
    pygame.quit()
