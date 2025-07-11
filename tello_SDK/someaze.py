import socket
import threading
import time

# IP et ports des drones Tello EDU
TELLO_DRONES = [
    {"ip": "192.168.50.33", "cmd_port": 8889, "telemetry_port": 8890, "name": "Tello1"},
    {"ip": "192.168.50.103", "cmd_port": 8889, "telemetry_port": 8891, "name": "Tello2"},
]

# Création des sockets commandes et télémétrie
command_sockets = {}
telemetry_sockets = {}

for drone in TELLO_DRONES:
    # Socket pour commandes
    sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_cmd.bind(('', 0))  # Port OS choisi automatiquement
    command_sockets[drone["name"]] = sock_cmd

    # Socket pour télémétrie (bind sur le port spécifique pour écouter)
    sock_telemetry = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_telemetry.bind(('', drone["telemetry_port"]))
    telemetry_sockets[drone["name"]] = sock_telemetry

def send_command(drone_name, ip, port, command):
    sock = command_sockets[drone_name]
    sock.sendto(command.encode(), (ip, port))
    try:
        sock.settimeout(5)
        response, _ = sock.recvfrom(1024)
        print(f"[{drone_name}] Réponse: {response.decode()}")
    except socket.timeout:
        print(f"[{drone_name}] Pas de réponse pour la commande '{command}'")

def listen_telemetry(drone_name):
    sock = telemetry_sockets[drone_name]
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            message = data.decode('utf-8')
            imu = parse_imu(message)
            print(f"[{drone_name} IMU] {imu}")
        except Exception as e:
            print(f"[{drone_name}] Erreur réception télémétrie: {e}")
            break

def parse_imu(data_str):
    # Extrait pitch, roll, yaw, etc. du message télémétrie
    data = {}
    for item in data_str.strip().split(';'):
        if ':' in item:
            key, value = item.split(':')
            data[key] = value
    return {
        "pitch": data.get("pitch"),
        "roll": data.get("roll"),
        "yaw": data.get("yaw"),
        "vgx": data.get("vgx"),
        "vgy": data.get("vgy"),
        "vgz": data.get("vgz"),
        "templ": data.get("templ"),
        "temph": data.get("temph"),
        "tof": data.get("tof")
    }

# Initialisation : activer mode commande + activer streamon (utile parfois)
def init_drone(drone):
    send_command(drone["name"], drone["ip"], drone["cmd_port"], "command")
    time.sleep(0.5)
    send_command(drone["name"], drone["ip"], drone["cmd_port"], "streamon")

if __name__ == "__main__":
    # Initialiser chaque drone
    for drone in TELLO_DRONES:
        init_drone(drone)

    # Lancer l'écoute télémétrie en parallèle pour chaque drone
    for drone in TELLO_DRONES:
        threading.Thread(target=listen_telemetry, args=(drone["name"],), daemon=True).start()

    print("📡 Écoute de la télémétrie IMU des drones Tello EDU démarrée.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nArrêt du programme.")
        for sock in command_sockets.values():
            sock.close()
        for sock in telemetry_sockets.values():
            sock.close()
# # import socket
# # import threading

# # def listen_imu(drone_name, port):
# #     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# #     sock.bind(('', port))
# #     print(f"[{drone_name}] Écoute sur le port {port} démarrée.")
# #     while True:
# #         try:
# #             data, _ = sock.recvfrom(1024)
# #             data_str = data.decode('utf-8')
# #             print(f"[{drone_name} IMU] {data_str}")
# #         except Exception as e:
# #             print(f"[{drone_name} IMU] Erreur : {e}")
# #             break

# # if __name__ == "__main__":
# #     # Ports IMU assignés pour chaque drone Tello EDU
# #     # Par défaut, le premier drone écoute sur 8890, le second sur 8891, etc.
# #     drone1_port = 8890
# #     drone2_port = 8891

# #     # Lancer un thread par drone pour écouter la télémétrie IMU
# #     thread1 = threading.Thread(target=listen_imu, args=("Tello1", drone1_port))
# #     thread2 = threading.Thread(target=listen_imu, args=("Tello2", drone2_port))

# #     thread1.start()
# #     thread2.start()

# #     thread1.join()
# #     thread2.join()
# import socket

# def listen_tello(port, name):
#     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock.bind(('', port))
#     print(f"[{name}] Écoute sur le port {port} démarrée.")

#     while True:
#         data, addr = sock.recvfrom(1024)
#         message = data.decode('utf-8')
#         if "mid" in message:  # condition simple pour repérer les données IMU
#             print(f"[{name} IMU] {message}")

# # Lancer ces deux threads ou fonctions en parallèle pour les deux tellos
# import threading
# threading.Thread(target=listen_tello, args=(8890, "Tello1")).start()
# threading.Thread(target=listen_tello, args=(8891, "Tello2")).start()
