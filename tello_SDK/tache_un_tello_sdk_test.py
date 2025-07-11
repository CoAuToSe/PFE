from djitellopy import Tello
import time

# Initialisation du drone
tello = Tello()

try:
    # Connexion au drone
    tello.connect()

    # Affichage du niveau de batterie
    battery = tello.get_battery()
    print(f"Niveau de batterie : {battery}%")

    # Vérification si la batterie est suffisante pour le vol
    if battery < 20:
        print("⚠️ Batterie trop faible pour le test ! Rechargez le drone.")
    else:
        # Décollage
        tello.takeoff()
        print("🛫 Drone en vol...")

        # Maintien en vol stationnaire pendant 5 secondes
        time.sleep(5)

        # Atterrissage
        tello.land()
        print("🛬 Atterrissage réussi.")

except Exception as e:
    print(f"❌ Erreur : {e}")

finally:
    # Déconnexion propre du drone
    tello.end()
