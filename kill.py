from comm import Drone

def main():
    drone = Drone("192.168.4.1", 23, 1)
    drone.disarm()

main()