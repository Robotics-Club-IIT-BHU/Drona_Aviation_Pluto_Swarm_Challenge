from localization import poseEstimation, ImageServer
from control import altitude_cnt, main_controller, my_controller
from comm import Drone
from trajectory import Trajectory

# from utils import sleepTimer
# from trajectory import BasicTrajectoryServer as trajectoryServer # Add task specific trajectory servers
# from ppm_driver import Lidar
import time
import numpy as np
import signal
import sys

drone = Drone("192.168.4.1", 23, 1, plotter=False)
img_server = ImageServer(source=1, sleep_rate=1, frame_rate=60)


def main():
    global drone, img_server
    # lidar = Lidar("192.168.4.125", 8888, 1, plotter=True, reference_line=100)
    # trajectory = Trajectory(endpoints=[[0, 0], [2, 2]], accuracy=0.1, stepsize=0.1)
    # drone.prepare()

    # trajectory_server = trajectoryServer(r, n) ## Define shape using polygon

    img_server.connect()
    time.sleep(2)
    initial_rpy = drone.getState()[0]
    # target_rpy[1]+=0.001
    drone.arm()
    print("Armed")
    drone.take_off()
    print("Takeoff initiated")
    coordinates = poseEstimation(plotter=True, reference_line=[100, 100, 100])

    # drone.getControl().start()
    # sleep_timer = sleepTimer(50) # rate
    controllers = [
        altitude_cnt(tracker_mode=False),
        # rpy_cnt()
        # main_controller(initial_rpy=initial_rpy)
        my_controller(initial_state=initial_rpy),
    ]
    lp_pose = [0.0, 0.0, 0.0]
    command = [0, 0, 0, 0]
    while drone.ok():
        # print(drone.getState())

        if img_server.grabbed:
            pose = coordinates.fetch(img_server.prev)
            # print("Test ",pose)
            if True:
                drone_info = drone.getState()[0]

                # print(height)
                print(drone_info, "Acc= ", drone.getState()[1])
                # desired_coordinates = trajectory.getDestination(current_point=[1, 2])
                # desired_states = trajectory_server.fetch_next_goal(coordinates, drone_info)
                # command = []
                # for controller in controllers:
                # height = lp_pose[2]
                target_pos = [0.2, -0.26, 1.0]
                command = controllers[1].update(
                    np.array(lp_pose), target_pos, np.array(drone_info), initial_rpy
                )
                # print(f"Command {command}")
                # print(command)
                if pose is not None:
                    lp_pose = pose
                    lp_pose[2] = max(0.0, pose[2])
                    print(
                        f"X={int(lp_pose[0]*100)}cm Y={int(lp_pose[1]*100)}cm Z={int(lp_pose[2]*100)}cm"
                    )

                    command1 = controllers[0].update(
                        np.array(lp_pose), np.array(drone_info), target_pos[2]
                    )
                    command[2] = command1[2]
                    thrust = command[2]
                #     print("Detected command ",command)

                #     # command=command1
                else:
                    thrust = max(thrust - 2, 1550)
                    #     # command[0] = 1485
                    #     # command[1] = 1535
                    #     # command[0] = 1500
                    #     # command[1] = 1500
                    command[2] = thrust
                    command[3] = 1500
                #     # time.sleep(0.0022)
                coordinates.show_plotter()
                # print(height)
                # command = controllers[0].update(np.array([1500, 1500, 1500]), np.array(drone_info))
                # command=controller
                commandp = drone.command_preprocess(command)
                drone.sendCommand(commandp)
                # drone.show_plotter()
                # sleep_timer.sleep()
            else:

                # print("Cannot detect aruco")
                # drone.reset()
                pass

    time.sleep(0.05)
    drone.land()
    time.sleep(0.05)

    del drone
    del img_server
    # del trajectory_server
    del controllers
    print("Completed run")


def signal_handler(signal, frame):
    # your code here

    global drone, img_server
    #   drone.land()
    #   time.sleep(2)
    drone.disarm()
    del drone
    del img_server
    print("\n--- Threads Ended ---")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


main()
