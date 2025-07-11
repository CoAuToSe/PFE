from djitellopy import Tello
import time

tello = Tello()
tello.connect()
print(f'Niveau de batterie : {tello.get_battery()}%')

tello.takeoff()
time.sleep(5)
tello.land()
