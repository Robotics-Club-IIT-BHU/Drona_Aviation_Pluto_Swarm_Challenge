from localization import poseEstimation, ImageServer
from control import altitude_cnt, cartesian_cnt, rpy_cnt
from comm import Drone
from trajectory import Trajectory

# from utils import sleepTimer
# from trajectory import BasicTrajectoryServer as trajectoryServer # Add task specific trajectory servers
# from ppm_driver import Lidar
import time
import numpy as np
import signal
import sys

drone = Drone("192.168.4.1", 23, 1)
img_server = ImageServer(source = 2, sleep_rate = 1, frame_rate = 60)

def main():
    global drone, img_server
    # lidar = Lidar("192.168.4.125", 8888, 1, plotter=True, reference_line=100)
    # trajectory = Trajectory(endpoints=[[0, 0], [2, 2]], accuracy=0.1, stepsize=0.1)
    # drone.prepare()
    
    # trajectory_server = trajectoryServer(r, n) ## Define shape using polygon

    img_server.connect()
    time.sleep(10)
    drone.arm()
    print("Armed")
    drone.take_off()
    print("Takeoff initiated")
    # height = lidar.Distance
    coordinates = poseEstimation(plotter=True, reference_line=100)

    # drone.getControl().start()
    # sleep_timer = sleepTimer(50) # rate
    controllers = [
        altitude_cnt(tracker_mode=False),
        # rpy_cnt()
        # cartesian_cnt()
    ]
    lp_pose=[0.0,0.0,0.0]
    while drone.ok():
        if img_server.grabbed:
            pose=coordinates.fetch(img_server.prev)
            coordinates.show_plotter()
            if pose is not None:
                lp_pose=pose
                lp_pose[2]=max(0.0, pose[2])
            print(f"X={int(lp_pose[0]*100)}cm Y={int(lp_pose[1]*100)}cm Z={int(lp_pose[2]*100)}cm")
        # print(height)
        drone_info = drone.getState()
        # desired_coordinates = trajectory.getDestination(current_point=[1, 2])
        # desired_states = trajectory_server.fetch_next_goal(coordinates, drone_info)
        # command = []
        # for controller in controllers:
        height = lp_pose[2]
        command = controllers[0].update([0, 0, height], drone_info, 0.8)
        # print(height)
        # command += controllers[0].update(np.array([1500, 1500, 1500]), np.array(drone_info))
        # command=controllers
        command = drone.command_preprocess(command)
        drone.sendCommand(command)
        # sleep_timer.sleep()

        # time.sleep(0.02)

    # final_command = []
    # for cont in controllers:
    # 	final_command += cont.kill()
    # final_command = drone.command_preprocess(final_command)
    # drone.sendCommand(final_command)
    # sleep_timer.sleep()
    time.sleep(0.05)
    drone.land()
    # sleep_timer.sleep()
    time.sleep(0.05)

    del drone
    del img_server
    # del trajectory_server
    del controllers
    print("Completed run")


def signal_handler(signal, frame):
  # your code here
  
  global drone, img_server
  drone.disarm()
  del drone
  del img_server
  print("\n--- Threads Ended ---")
  sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


main()