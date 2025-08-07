import socket

## On a dit que le tello est en command

# Adresse et port du drone
tello_address = ('192.168.50.144', 8889)

# Créer un socket d’envoi de commandes
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9000))  # Port local arbitraire

# Envoyer "command" pour activer le mode SDK
sock.sendto(b'command', tello_address)

# Attendre une réponse
try:
    response, _ = sock.recvfrom(1024)
    print("Réponse du Tello :", response.decode('utf-8'))
except socket.timeout:
    print("Pas de réponse (timeout).")


## On a écoute le tello

# Adresse IP et port du Tello pour le state
state_port = 8890

# Crée le socket UDP
state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_socket.bind(('', state_port))

print("En attente des données de state...")

while True:
    try:
        data, _ = state_socket.recvfrom(1024)
        print("State brut :", data.decode('utf-8'))
    except KeyboardInterrupt:
        break

state_socket.close()
