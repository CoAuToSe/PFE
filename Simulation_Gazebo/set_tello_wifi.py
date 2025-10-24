import socket
import time

# Adresse et port du drone (en mode Wi-Fi direct)
TELLO_IP = "192.168.50.103"
TELLO_PORT = 8889

# SSID et mot de passe de ton routeur
SSID = "routeur_asus"
PASSWORD = "ensta3012*"

# Créer un socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 9000))  # Port local pour écouter les réponses du drone

def send_command(command):
    """ Envoie une commande UDP au drone """
    sock.sendto(command.encode(), (TELLO_IP, TELLO_PORT))
    time.sleep(2)  # Pause pour éviter les envois trop rapides

# Activer le mode SDK
send_command("command")

# Dire au drone de se connecter au routeur
send_command(f"ap {SSID} {PASSWORD}")

print("Commande envoyée. Le drone devrait redémarrer et se connecter au routeur.")

# Fermer le socket
sock.close()
