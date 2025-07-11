from djitellopy import Tello
import time
import threading

# Liste des IPs des drones connectés
TELLO_IPS = ['192.168.50.103', '192.168.50.33', '192.168.50.63']

def control_drone(ip):
    drone = Tello(host=ip)
    drone.connect()
    print(f"{ip} battery: {drone.get_battery()}%")
    drone.takeoff()
    time.sleep(3)
    drone.move_up(50)
    time.sleep(2)
    drone.land()

threads = []

# Lancer un thread pour chaque drone
for ip in TELLO_IPS:
    t = threading.Thread(target=control_drone, args=(ip,))
    threads.append(t)
    t.start()

# Attendre la fin de tous les threads
for t in threads:
    t.join()
