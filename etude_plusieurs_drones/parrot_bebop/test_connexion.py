from pyparrot.Bebop import Bebop

bebop = Bebop(drone_type="Bebop")
print("connecting")
success = bebop.connect(10)
print("Connection Success:", success)

if success:
    print("Drone Connected")
    bebop.disconnect()
