from localization import fetch_coordinates, imageServer
from control import altitude_cnt, cartesian_cnt
from comm import Drone
from utils import sleepTimer
from trajectory import BasicTrajectoryServer as trajectoryServer # Add task specific trajectory servers

def main():
	drone = Drone(
			# ip address,
			# port number,
			# timeout
			)
	# drone.prepare()
	drone.arm()
	drone.take_off()

	trajectory_server = trajectoryServer(r, n) ## Define shape using polygon

	img_server = imageServer()
	img_server.start()
	coordinates = fetch_coordinates(img_server)

	# drone.getControl().start()
	sleep_timer = sleepTimer(50) # rate
	controllers = [
			altitude_cnt(),
			cartesian_cnt()
			]

	while drone.ok():
		coordinates.fetch()
		drone_info = drone.getState()
		desired_states = trajectory_server.fetch_next_goal(coordinates, drone_info)
		command = []
		for controller in controllers:
			command += controller.update(coordinates, drone_info, desired_states)
		command = drone.command_preprocess(command)
		drone.sendCommand(command)
		sleep_timer.sleep()
	
	finally:
		final_command = []
		for cont in controllers:
			final_command += cont.kill()
		final_command = drone.command_preprocess(final_command)
		drone.sendCommand(command)
		sleep_timer.sleep()
		drone.land()
		sleep_timer.sleep()
	del drone
	del trajectory_server
	del controllers
	print("completed run")
