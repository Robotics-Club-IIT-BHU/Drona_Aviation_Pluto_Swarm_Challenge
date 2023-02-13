import os
import time
import argparse
import numpy as np
import pybullet as p

from gym_pybullet_drones.envs.BaseAviary import BaseAviary
from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.utils.utils import sync, str2bool

MAX_ACC = 0.19
MAX_VEL = 0.14


def desired_pos(env):
    global MAX_ACC
    global MAX_VEL
    global DEFAULT_CONTROL_FREQ_HZ

    dt = 1

    ######ALIGNMENT##########
    des_vel = np.array([env.vel[0, 0], env.vel[0, 1], env.vel[0, 2]])
    des_vel = (des_vel/(np.linalg.norm(des_vel) + 1e-6))*MAX_VEL
    a_align = des_vel - np.array([env.vel[1, 0], env.vel[1, 1], env.vel[1, 2]])
    a_align_dirn = a_align/(np.linalg.norm(a_align) + 1e-6)
    #########################


    ######COHESION######
    rel_pos = np.array([env.pos[0, 0], env.pos[0, 1], env.pos[0, 2]]) - np.array(
        [env.pos[1, 0], env.pos[1, 1], env.pos[1, 2]])

    a_cohesion = rel_pos - np.array([env.vel[1, 0], env.vel[1, 1], env.vel[1, 2]])
    a_cohesion_dirn = a_cohesion/(np.linalg.norm(a_cohesion) + 1e-6)
    #####################

    trgt_a = a_align_dirn + a_cohesion_dirn
    trgt_a = (trgt_a/(np.linalg.norm(trgt_a) + 1e-6))*MAX_ACC


    trgt_vel = np.array([env.vel[1, 0], env.vel[1, 1], env.vel[1, 2]]) + trgt_a*dt
    trgt_vel = (trgt_vel/(np.linalg.norm(trgt_vel) + 1e-6))*MAX_VEL


   ######SEPERATION#######
    if np.linalg.norm(rel_pos) < 0.2:
        trgt_pos = np.array([env.pos[1, 0], env.pos[1, 1], env.pos[1, 2]])

    else:
        trgt_pos = np.array([env.pos[1, 0], env.pos[1, 1], env.pos[1, 2]]) + trgt_vel*dt
    ########################

    print("Trgt_pos: ", trgt_pos)
    return trgt_pos


DEFAULT_AGGREGATE = True
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 24
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False


def run(
        aggregate=DEFAULT_AGGREGATE,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
):
    #### Initialize the simulation #############################
    H = .1
    H_STEP = .05
    R = .3
    INIT_XYZS = np.array([[0, 0, 1], [-0.3, 0, 1]])
    INIT_RPYS = np.array([[0, 0, i * (np.pi / 2) / 2] for i in range(2)])
    AGGR_PHY_STEPS = int(simulation_freq_hz / control_freq_hz) if aggregate else 1

    #### Initialize a circular trajectory ######################
    PERIOD = 20
    NUM_WP = control_freq_hz * PERIOD
    TARGET_POS = np.zeros((NUM_WP, 3))
    a = 0

    X_WIDTH = 1
    Y_HEIGHT = 1

    for i in range(NUM_WP // 4):
        TARGET_POS[a, :] = INIT_XYZS[0, 0] + X_WIDTH * 0.005 * i, INIT_XYZS[0, 1], 0
        a += 1
    for i in range(NUM_WP // 4):
        TARGET_POS[a, :] = INIT_XYZS[0, 0] + X_WIDTH * 0.005 * (NUM_WP // 4 - 1), INIT_XYZS[0, 1] + Y_HEIGHT * 0.005 * i, 0
        a += 1
    for i in range(NUM_WP // 4):
        TARGET_POS[a, :] = INIT_XYZS[0, 0] + X_WIDTH * 0.005 * (NUM_WP // 4 - 1) - X_WIDTH * 0.005 * i, INIT_XYZS[0, 1] + Y_HEIGHT * 0.005 * (
                NUM_WP // 4 - 1), 0
        a += 1
    for i in range(NUM_WP // 4):
        TARGET_POS[a, :] = INIT_XYZS[0, 0], INIT_XYZS[0, 1] + Y_HEIGHT * 0.005 * (NUM_WP // 4 - 1) - Y_HEIGHT * 0.005 * i, 0
        a += 1
    wp_counters = np.array([0])

    #### Create the environment with or without video capture ##
    env = BaseAviary(initial_xyzs=INIT_XYZS,
                     initial_rpys=INIT_RPYS,
                     neighbourhood_radius=10,
                     freq=simulation_freq_hz,
                     aggregate_phy_steps=AGGR_PHY_STEPS,
                     )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the controllers ############################
    ctrl = [BaseControl() for i in range(2)]


    #### Run the simulation ####################################
    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ / control_freq_hz))
    action = {str(i): np.array([0, 0, 0, 0]) for i in range(2)}
    START = time.time()

    prev_pos_0 = INIT_XYZS[0]
    prev_pos_1 = INIT_XYZS[1]
    line =0
    for i in range(0, 3 * int(duration_sec * env.SIM_FREQ), AGGR_PHY_STEPS):

        #### Step the simulation ###################################
        obs, done, info = env.step(action)

        next_pos_1 = np.array([env.pos[1, 0], env.pos[1, 1], env.pos[1, 2]])
        p.addUserDebugLine(prev_pos_1, next_pos_1, [1, 0, 0], lifeTime=3, lineWidth=2.0, physicsClientId=PYB_CLIENT)
        prev_pos_1 = next_pos_1

        next_pos_0 = np.array([env.pos[0, 0], env.pos[0, 1], env.pos[0, 2]])
        if line==1:
            p.addUserDebugLine(prev_pos_0, next_pos_0, [0, 1, 0], lifeTime=0, lineWidth=2.0, physicsClientId=PYB_CLIENT)
        prev_pos_0 = next_pos_0

        p.addUserDebugLine(prev_pos_0, prev_pos_1, [0, 0, 1], lifeTime=0.1, physicsClientId=PYB_CLIENT)

        #### Compute control at the desired frequency ##############
        if i % CTRL_EVERY_N_STEPS == 0:
            #### Compute control for the current way point #############

            action[str(0)], _, _ = ctrl[0].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS * env.TIMESTEP,
                                                                   state=obs[str(0)]["state"],
                                                                   target_pos=np.hstack(
                                                                       [TARGET_POS[wp_counters[0], 0:2],
                                                                        INIT_XYZS[0, 2]]),
                                                                   target_rpy=INIT_RPYS[0, :]
                                                                   )


            action[str(1)], _, _ = ctrl[1].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS * env.TIMESTEP,
                                                                   state=obs[str(1)]["state"],
                                                                   target_pos=desired_pos(env),
                                                                   target_rpy=INIT_RPYS[1, :]
                                                                   )


            #### Go to the next way point and loop #####################
            wp_counters[0] = wp_counters[0] + 1 if wp_counters[0] < (NUM_WP - 1) else 0
            if wp_counters[0] == 0:
                line+=1



        #### Sync the simulation ###################################
        sync(i, START, env.TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################

    #### Plot the simulation results ###########################


if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser()
    parser.add_argument('--aggregate', default=DEFAULT_AGGREGATE, type=str2bool,
                        help='Whether to aggregate physics steps (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int,
                        help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int,
                        help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec', default=DEFAULT_DURATION_SEC, type=int,
                        help='Duration of the simulation in seconds (default: 5)', metavar='')


    ARGS = parser.parse_args()

    run(**vars(ARGS))
