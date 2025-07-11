import time
import matplotlib.pyplot as plt
import math
from djitellopy import Tello

class TelloNavigator:
    def __init__(self, sin_period=8):
        self.tello = Tello()
        self.tello.connect()
        self.battery_start = self.tello.get_battery()

        self.length = 200 # Longueur en cm
        self.width = 100   # Largeur en cm
        self.base_height = 50  # Hauteur de vol initiale
        self.amplitude = 10    # Amplitude sinusoïde
        self.sin_period = sin_period  # Période en secondes
        self.battery_threshold = 20

        self.battery_data = []
        self.turns = 0
        self.segment_count = 0

    def wait_for_ok(self, command):
        response = self.tello.send_command_with_return(command)
        if response != "ok":
            raise Exception(f"Commande {command} échouée avec réponse : {response}")
        print(f"Commande '{command}' exécutée avec succès")

    def takeoff(self):
        print(f"Décollage à {self.base_height} cm")
        retries = 3
        for i in range(retries):
            try:
                self.wait_for_ok("takeoff")
                print("Décollage réussi")
                break
            except Exception as e:
                print(f"Tentative {i+1}/{retries} échouée : {e}")
                if i == retries - 1:
                    raise e
                time.sleep(2)
        time.sleep(1)
        self.wait_for_ok(f"up {self.base_height}")
        time.sleep(1)

    def continuous_sinusoidal_movement(self, duration, speed=40):
        print("Début du déplacement sinusoïdal")
        start_time = time.time()
        while time.time() - start_time < duration:
            t = time.time() - start_time
            height_offset = int(self.amplitude * math.sin(2 * math.pi * t / self.sin_period))
            self.tello.send_rc_control(0, speed, height_offset, 0)
            time.sleep(0.1)  # 10 Hz de fréquence pour lisser le vol

        self.tello.send_rc_control(0, 0, 0, 0)
        print("Fin du déplacement sinusoïdal")

    def make_turn(self):
        print(f"Début du tour {self.turns + 1}")

        for distance in [self.length, self.width, self.length, self.width]:
            duration = distance / 40  # 40 cm/s vitesse constante
            print(f"Déplacement sur {distance} cm pendant {duration:.2f} secondes")
            self.continuous_sinusoidal_movement(duration)
            self.segment_count += int(duration * 10)
            self.wait_for_ok("cw 90")

    def monitor_battery(self):
        battery_percentage = self.tello.get_battery()
        print(f"Batterie restante : {battery_percentage}%")
        return battery_percentage

    def record_battery(self):
        battery_current = self.monitor_battery()
        battery_consumed = self.battery_start - battery_current
        self.battery_data.append((self.turns, battery_current, battery_consumed))
        print(f"Batterie consommée après ce tour : {battery_consumed}%")
        self.turns += 1

    def plot_battery_data(self):
        if self.battery_data:
            turns, battery_percentages, _ = zip(*self.battery_data)
            plt.figure(figsize=(10, 6))
            plt.plot(turns, battery_percentages, label="Batterie restante (%)", color='blue', marker='o')
            plt.xlabel("Nombre de tours")
            plt.ylabel("Batterie restante (%)")
            plt.title("Consommation de batterie du drone Tello")
            plt.grid(True)
            plt.legend()
            plt.show()

    def run(self):
        
        print(f"Batterie de départ : {self.battery_start}%")
        self.takeoff()

        while True:
            self.make_turn()
            self.record_battery()

            if self.monitor_battery() <= self.battery_threshold:
                print("⚠️ Batterie faible - Atterrissage imminent")
                self.wait_for_ok("land")
                break

            time.sleep(2)

        self.plot_battery_data()


def main():
    navigator = TelloNavigator(sin_period=4)
    navigator.run()


if __name__ == "__main__":
    main()
