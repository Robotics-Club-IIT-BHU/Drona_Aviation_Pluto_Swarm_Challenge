import time

class Data():
	def __init__(self):
		self.rcRoll =1500
		self.rcPitch = 1500
		self.rcYaw =1500
		self.rcThrottle =1500
		self.rcAUX1 =1500
		self.rcAUX2 =1500
		self.rcAUX3 =1500
		self.rcAUX4 =1000
		self.commandType = 0
		self.trim_roll = 0
		self.trim_pitch = 0
		self.isAutoPilotOn = 0

class send_data():
	"""docstring for request_data"""
	def __init__(self,drone):
		self.key_value =0
		self.cmd=Data()
		self.drone=drone

	def arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1000
		self.cmd.rcAUX4 =1500
		self.cmd.isAutoPilotOn = 0
		self.drone.sendCommand(self.cmd)
		time.sleep(1)

	def box_arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX4 =1500
		self.cmd.isAutoPilotOn = 0
		self.drone.sendCommand(self.cmd)
		time.sleep(0.5)

	def disarm(self):
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 = 1200
		self.drone.sendCommand(self.cmd)
		time.sleep(0.5)

	def indentify_key(self, msg):
		self.key_value = msg

		if self.key_value == 70:
			if(self.cmd.rcAUX4 == 1500):
				self.disarm()
			else:
				self.arm()
		elif self.key_value == 10:
			self.forward()
		elif self.key_value == 30:
			self.left()
		elif self.key_value == 40:
			self.right()
		elif self.key_value == 80:
			self.reset()
		elif self.key_value == 90:
			if(self.cmd.isAutoPilotOn == 1):
				self.cmd.isAutoPilotOn = 0
			else:
				self.cmd.isAutoPilotOn = 1
		elif self.key_value == 50:
			self.increase_height()
		elif self.key_value == 60:
			self.decrease_height()
		elif self.key_value == 110:
			self.backward()
		elif self.key_value == 130:
			self.take_off()
		elif self.key_value == 140:
			self.land()
		elif self.key_value == 150:
			self.left_yaw()
		elif self.key_value == 160:
			self.right_yaw()
		self.drone.sendCommand(self.cmd)

	def forward(self):
		self.cmd.rcPitch =1600
		self.drone.sendCommand(self.cmd)
	def backward(self):
		self.cmd.rcPitch =1400
		self.drone.sendCommand(self.cmd)
	def left(self):
		self.cmd.rcRoll =1400
		self.drone.sendCommand(self.cmd)
	def right(self):
		self.cmd.rcRoll =1600
		self.drone.sendCommand(self.cmd)
	def left_yaw(self):
		self.cmd.rcYaw = 1200
		self.drone.sendCommand(self.cmd)
	def right_yaw(self):
		self.cmd.rcYaw = 1800
		self.drone.sendCommand(self.cmd)
	def reset(self):
		self.cmd.rcRoll =1500
		self.cmd.rcThrottle =1600
		self.cmd.rcPitch =1500
		self.cmd.rcYaw = 1500
		self.cmd.commandType = 0
		self.drone.sendCommand(self.cmd)
	def increase_height(self):
		self.cmd.rcThrottle = 2100
		self.drone.sendCommand(self.cmd)
	def decrease_height(self):
		self.cmd.rcThrottle =1300
		self.drone.sendCommand(self.cmd)
	def take_off(self):
		self.disarm()
		self.box_arm()
		# for i in range(6500000):
		# 	self.increase_height()
		# self.reset()
		self.cmd.commandType = 1
		self.drone.sendCommand(self.cmd)
	def land(self):
		self.cmd.commandType = 2
		self.drone.sendCommand(self.cmd)
