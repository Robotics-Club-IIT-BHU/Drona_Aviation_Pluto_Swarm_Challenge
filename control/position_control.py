import math
import numpy as np
class cartesian_cnt:

    def __init__(self,g: float=9.8):
        self.P_COEFF_FOR = np.array([.1, .1, .2])
        self.I_COEFF_FOR = np.array([.0001, .0001, .0001])
        self.D_COEFF_FOR = np.array([.3, .3, .4])
        self.MAX_ROLL_PITCH = np.pi/6
        self.GRAVITY = g
        self.control_timestep = 1./125.
        self.reset()

    def reset(self):

        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)
        self.control_counter =0

    def getMatrix(self,cur_rpy):

        yaw = cur_rpy[2]
        pitch = cur_rpy[1]
        roll = cur_rpy[0]
        yaw_matrix = np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        pitch_matrix = np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
        roll_matrix = np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
        return np.matmul(np.matmul(yaw_matrix,pitch_matrix), roll_matrix)

    def _simplePIDPositionControl(self,
                                  cur_pos,
                                  cur_rpy,
                                  target_pos
                                  ):

        pos_e = target_pos - cur_pos
        d_pos_e = (pos_e - self.last_pos_e) / self.control_timestep
        self.last_pos_e = pos_e
        self.integral_pos_e = self.integral_pos_e + pos_e*self.control_timestep
        target_force = np.array([0, 0, self.GRAVITY]) \
                       + np.multiply(self.P_COEFF_FOR, pos_e) \
                       + np.multiply(self.I_COEFF_FOR, self.integral_pos_e) \
                       + np.multiply(self.D_COEFF_FOR, d_pos_e)
        target_rpy = np.zeros(3)
        sign_z =  np.sign(target_force[2])
        if sign_z == 0:
            sign_z = 1
        target_rpy[0] = np.arcsin(-sign_z*target_force[1] / np.linalg.norm(target_force))
        target_rpy[1] = np.arctan2(sign_z*target_force[0], sign_z*target_force[2])
        target_rpy[2] = 0.
        target_rpy[0] = np.clip(target_rpy[0], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        target_rpy[1] = np.clip(target_rpy[1], -self.MAX_ROLL_PITCH, self.MAX_ROLL_PITCH)
        cur_rotation = np.array(self.getMatrix(cur_rpy)).reshape(3, 3)
        thrust = np.dot(cur_rotation, target_force)
        return thrust[2], target_rpy, pos_e

    def update(self,cur_pos,cur_rpy,target_pos):

        self.control_counter += 1

        thrust, computed_target_rpy, pos_e = self._simplePIDPositionControl(cur_pos,
                                                                            cur_rpy,
                                                                            target_pos
                                                                            )

        return computed_target_rpy,thrust
