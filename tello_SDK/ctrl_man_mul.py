import socket
import threading
import pygame
import time

# Configuration des drones
TELLO1_IP = "192.168.50.33"
TELLO2_IP = "192.168.50.103"
TELLO_PORT = 8889  # Port UDP des drones

# Création des sockets pour chaque drone
sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(sock1, sock2)

# Initialisation de Pygame pour la manette
pygame.init()
pygame.joystick.init()

# Vérifier si une manette est connectée
if pygame.joystick.get_count() == 0:
    print("🚨 Aucune manette détectée ! Connecte une manette et redémarre le script.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

# Fonction pour envoyer une commande à un drone
def send_command(tello_ip, command):
    sock = sock1 if tello_ip == TELLO1_IP else sock2
    sock.sendto(command.encode(), (tello_ip, TELLO_PORT))
    try:
        sock.settimeout(3)  # Timeout de 3 secondes
        response, _ = sock.recvfrom(1024)
        print(f"[{tello_ip}] → {response.decode()}")
    except socket.timeout:
        print(f"[{tello_ip}] ⚠️ Aucune réponse du drone")

# Fonction pour envoyer une commande aux deux drones en parallèle
def send_to_all(command):
    thread1 = threading.Thread(target=send_command, args=(TELLO1_IP, command))
    thread2 = threading.Thread(target=send_command, args=(TELLO2_IP, command))

    thread1.start()
    thread2.start()
    
    thread1.join()
    thread2.join()

# Activation du mode commande
send_to_all("command")
time.sleep(1)

print("✅ Prêt à contrôler les drones avec la manette !")

running = True
while running:
    pygame.event.pump()

    # Joystick axes
    left_stick_y = -joystick.get_axis(1)  # Avant/Arrière
    left_stick_x = joystick.get_axis(0)   # Gauche/Droite
    right_stick_y = -joystick.get_axis(3) # Haut/Bas
    right_stick_x = joystick.get_axis(2)  # Rotation

    # Boutons
    takeoff_button = joystick.get_button(0)  # Bouton A (Xbox) / Croix (PS4)
    land_button = joystick.get_button(1)     # Bouton B (Xbox) / Rond (PS4)
    emergency_button = joystick.get_button(7)  # Bouton START

    # Décollage
    if takeoff_button:
        send_to_all("takeoff")
        time.sleep(1)

    # Atterrissage
    if land_button:
        send_to_all("land")
        time.sleep(1)

    # Bouton d’urgence
    if emergency_button:
        print("🚨 ATTERRISSAGE D'URGENCE 🚨")
        send_to_all("land")
        running = False

    # Contrôle du premier drone (Tello1)
    if abs(left_stick_y) > 0.1:
        send_command(TELLO1_IP, f"forward {int(left_stick_y * 50)}" if left_stick_y > 0 else f"back {int(-left_stick_y * 50)}")
    if abs(left_stick_x) > 0.1:
        send_command(TELLO1_IP, f"right {int(left_stick_x * 50)}" if left_stick_x > 0 else f"left {int(-left_stick_x * 50)}")

    # Contrôle du deuxième drone (Tello2)
    if abs(right_stick_y) > 0.1:
        send_command(TELLO2_IP, f"up {int(right_stick_y * 50)}" if right_stick_y > 0 else f"down {int(-right_stick_y * 50)}")
    if abs(right_stick_x) > 0.1:
        send_command(TELLO2_IP, f"cw {int(right_stick_x * 50)}" if right_stick_x > 0 else f"ccw {int(-right_stick_x * 50)}")

    time.sleep(0.1)  # Petite pause pour éviter de spammer les commandes

# Fermeture des sockets
sock1.close()
sock2.close()
pygame.quit()
