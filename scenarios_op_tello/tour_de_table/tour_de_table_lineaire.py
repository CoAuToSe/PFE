import time
import matplotlib.pyplot as plt
from djitellopy import Tello

class TelloNavigator:
    def __init__(self):
        self.tello = Tello()
        self.tello.connect()
        self.battery_start = self.tello.get_battery()

        # Paramètres de la table
        self.length = 238  # Longueur en cm
        self.width = 119   # Largeur en cm
        self.height = 60    # Hauteur de vol en cm
        self.battery_threshold = 20  # Seuil de batterie en %

        # Liste pour enregistrer les données de batterie et le nombre de tours
        self.battery_data = []
        self.turns = 0

    def send_command(self, command):
        """Envoie une commande et attend la confirmation 'ok'"""
        response = self.tello.send_command_with_return(command)
        while response != "ok":
            time.sleep(1)

    def takeoff(self):
        """Faire décoller le drone et s'élever à la hauteur souhaitée."""
        print(f"Décollage à {self.height} cm de hauteur")
        self.send_command("takeoff")
        time.sleep(2)
        self.send_command(f"up {self.height}")
        time.sleep(1)

    def make_turn(self):
        """Tour autour de la table avec déplacements linéaires (sans rotation)."""
        print(f"Début du tour {self.turns + 1}")

        # Avant
        self.send_command(f"forward {int(self.length)}")
        time.sleep(1)
        # Droite
        self.send_command(f"right {int(self.width)}")
        time.sleep(1)
        # Arrière
        self.send_command(f"back {int(self.length)}")
        time.sleep(1)
        # Gauche
        self.send_command(f"left {int(self.width)}")
        time.sleep(1)

    def monitor_battery(self):
        """Vérifier la consommation de la batterie et stopper si nécessaire."""
        battery_percentage = self.tello.get_battery()
        print(f"Batterie restante: {battery_percentage}%")
        return battery_percentage

    def record_battery(self):
        """Enregistre la consommation de la batterie après chaque tour."""
        battery_current = self.monitor_battery()
        battery_consumed = self.battery_start - battery_current
        self.battery_data.append((self.turns, battery_current, battery_consumed))
        self.turns += 1
        print(f"Batterie consommée après ce tour : {battery_consumed}%")

    def plot_battery_data(self):
        """Affiche un graphique des données de batterie."""
        turns, battery_percentages, _ = zip(*self.battery_data)
        plt.figure(figsize=(10, 6))
        plt.plot(turns, battery_percentages, label='Batterie restante (%)', color='blue', marker='o')
        plt.xlabel("Nombre de tours")
        plt.ylabel("Batterie restante (%)")
        plt.title("Consommation de batterie du drone Tello")
        plt.grid(True)
        plt.legend()
        plt.show()

    def run(self):
        """Lancer le scénario"""
        time.sleep(2)
        self.takeoff()
        print(f"Batterie de départ : {self.battery_start}%")
        
        while True:
            self.make_turn()
            self.record_battery()

            if self.monitor_battery() <= self.battery_threshold:
                print("Batterie trop faible - Atterrissage")
                self.send_command("land")
                break
            time.sleep(2)

        self.plot_battery_data()


def main():
    navigator = TelloNavigator()
    navigator.run()


if __name__ == "__main__":
    main()
