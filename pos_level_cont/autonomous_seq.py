import time
import pandas as pd
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.positioning.position_hl_commander import PositionHlCommander
import numpy as np
import math

from mocaptools import sqrt, Pose, QtmWrapper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')

# argparse this or replace with relative path  
path_np = np.array(np.genfromtxt('/home/rajpal/CF programs/crazypaths/paths/circle_waypoints.csv',delimiter=',', skip_header=1, dtype=float).tolist())
sequence = path_np[:,1:]

QTM_IP = '192.168.0.106' 
CF_BODY = 'cf1'
# Change the sequence according to your setup
#             x    y    z  YAW
# sequence = [
#     (0.0, 0.0, 0.4, 0),
#     (0.0, 0.0, 0.5, 0),
#     (0.1, -0.1, 0.5, 0),
#     (0.1, 0.1, 0.5, 0),
#     (-0.1, 0.1, 0.5, 0),
#     (-0.1, -0.1, 0.5, 0),
#     (0.0, 0.0, 0.5, 0),
#     (0.0, 0.0, 0.4, 0),
#     (0.0, 0.0, 0.0, 0.0)
# ]


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=100)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    # activate Kalman estimator 
    cf.param.set_value('stabilizer.estimator', '2')

    # reset estimator
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def run_sequence(scf, sequence):
    cf = scf.cf
    for position in sequence:
        print('Setting position {}'.format(position))
        for i in range(10):
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                0.4,
                                                0)
            time.sleep(0.01)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def send_extpose_rot_matrix(cf, x, y, z, rot):
    """Send full pose from mocap to Crazyflie."""
    # get quaternion from rot matrix
    qw = sqrt(1 + rot[0][0] + rot[1][1] + rot[2][2]) / 2
    qx = sqrt(1 + rot[0][0] - rot[1][1] - rot[2][2]) / 2
    qy = sqrt(1 - rot[0][0] + rot[1][1] - rot[2][2]) / 2
    qz = sqrt(1 - rot[0][0] - rot[1][1] + rot[2][2]) / 2
    # Normalize the quaternion
    ql = math.sqrt(qx ** 2 + qy ** 2 + qz ** 2 + qw ** 2)
    # Send to Crazyflie
    cf.extpos.send_extpose(x, y, z, qx / ql, qy / ql, qz / ql, qw / ql)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    mocap = QtmWrapper()
    
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        #mocap.on_cf_pose = lambda pose: send_extpose_rot_matrix(scf.cf, pose[0], pose[1], pose[2], pose[3])

        reset_estimator(scf)
        # start_position_printing(scf)
        run_sequence(scf, sequence)
        mocap.close()
