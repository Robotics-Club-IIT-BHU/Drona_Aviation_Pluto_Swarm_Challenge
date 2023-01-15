from localization import poseEstimation, ImageServer
from control import altitude_cnt, cartesian_cnt
from comm import Drone
# from utils import sleepTimer
# from trajectory import BasicTrajectoryServer as trajectoryServer # Add task specific trajectory servers
from ppm_driver import Lidar
import time

def main():
	drone = Drone(
			"192.168.4.1",
			23,
			1
			)
	lidar = Lidar("192.168.4.124",8888,1,plotter=True,reference_line=50)

	# drone.prepare()
	drone.arm()
	print('Armed')
	drone.take_off()
	print('Takeoff initiated')
	# trajectory_server = trajectoryServer(r, n) ## Define shape using polygon

	# img_server = ImageServer(source = 1, sleep_rate = 1, frame_rate = 30)
	# img_server.connect()
	height = lidar.Distance
	# coordinates = poseEstimation(img_server.prev)

	# drone.getControl().start()
	# sleep_timer = sleepTimer(50) # rate
	controllers = [
			altitude_cnt(mode=1),
			# cartesian_cnt()
			]

	while drone.ok():
		height = lidar.Distance
		# coordinates.fetch(height)
		drone_info = drone.getState()
		# desired_states = trajectory_server.fetch_next_goal(coordinates, drone_info)
		command = []
		for controller in controllers:
			command += controller.update([0,0,height], drone_info, 50)
		# command=controllers
		command = drone.command_preprocess(command)
		drone.sendCommand(command)
		# if KeyboardInterrupt:
		# 	drone.disarm()
		# sleep_timer.sleep()
		time.sleep(0.05)
	

	final_command = []
	for cont in controllers:
		final_command += cont.kill()
	final_command = drone.command_preprocess(final_command)
	drone.sendCommand(command)
	# sleep_timer.sleep()
	time.sleep(0.05)
	drone.land()
	# sleep_timer.sleep()
	time.sleep(0.05)

	
	del drone
	# del trajectory_server
	del controllers
	print("completed run")

main()
