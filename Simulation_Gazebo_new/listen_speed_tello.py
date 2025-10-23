import socket
import threading

# Adresse et ports
TELLO_IP = '192.168.50.103'
CMD_PORT = 8889
STATE_PORT = 8890

# Socket pour les commandes
cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_socket.bind(('', 9000))

# Socket pour l'état
state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
state_socket.bind(('', STATE_PORT))

# Envoyer "command" pour activer le SDK
cmd_socket.sendto(b'command', (TELLO_IP, CMD_PORT))
print("Commande envoyée : 'command'")

# Fonction de parsing du state
def parse_state(state_str):
    state_dict = {}
    for pair in state_str.strip().split(';'):
        if pair:
            key, val = pair.split(':')
            state_dict[key] = val
    return state_dict

# Écouter les données de vitesse
def listen_velocity():
    while True:
        try:
            data, _ = state_socket.recvfrom(1024)
            state = parse_state(data.decode('utf-8'))
            vgx = int(state.get('vgx', 0))
            vgy = int(state.get('vgy', 0))
            vgz = int(state.get('vgz', 0))
            print(f"Vitesse → X: {vgx} cm/s | Y: {vgy} cm/s | Z: {vgz} cm/s")
        except Exception as e:
            print("Erreur dans le state:", e)
            break

# Démarrer l’écoute dans un thread
thread = threading.Thread(target=listen_velocity, daemon=True)
thread.start()

# Attendre la fin (Ctrl+C pour quitter)
input("Appuie sur Entrée pour quitter...\n")

# Nettoyage
state_socket.close()
cmd_socket.close()
