from pyparrot.Bebop import Bebop

# Crée une instance du drone Bebop 2
bebop = Bebop(drone_type="Bebop2")

print("connecting")
success = bebop.connect(10)
print(success)

if success:
    print("turning on the video")
    bebop.start_video_stream()

    print("sleeping")
    bebop.smart_sleep(2)

    try:
        bebop.ask_for_state_update()
    except Exception as e:
        print(f"Error accessing state update: {e}")

    # Décollage sécurisé avec un délai
    bebop.safe_takeoff(10)

    # Paramètres sécurisés pour voler à l'intérieur
    bebop.set_max_tilt(5)  # Angle de tilt maximum
    bebop.set_max_vertical_speed(1)  # Vitesse verticale limitée

    # Protection de la coque (à activer)
    bebop.set_hull_protection(1)

    print("Flying direct: Slow move for indoors")
    bebop.fly_direct(roll=0, pitch=20, yaw=0, vertical_movement=0, duration=2)

    bebop.smart_sleep(5)

    # Atterrissage sécurisé avec un délai
    bebop.safe_land(10)

    print("DONE - disconnecting")
    bebop.stop_video_stream()
    bebop.smart_sleep(5)
    bebop.disconnect()
else:
    print("Failed to connect to Bebop 2")
