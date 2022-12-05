import time
import pandas as pd
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
# from cflib.positioning.position_hl_commander import PositionHlCommander
# from cflib.positioning.motion_commander import MotionCommander
from paths import path_pars
import numpy as np
import math

import logging
from threading import Event
from scipy.spatial.transform import Rotation

from mocaptools import sqrt, Pose, QtmWrapper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')
QTM_IP = '192.168.0.128' 
CF_BODY = 'cf'

# argparse this or replace with relative path  
# path_np = np.array(np.genfromtxt('/home/rajpal/CF programs/crazypaths/paths/circle_waypoints.csv',delimiter=',', skip_header=1, dtype=float).tolist())
# sequence = path_np[:,1:]
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

# path parameters
t_run = 10
hieght = 0.5
t_lift = 5
t_land = 1
n_iters = 2


deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)



def log_motor_callback(timestamp, data, logconf):
    global mocap, INPUTS, OUTPUTS
    INPUTS['motor_timestamp'].append(timestamp)
    global start_time
    t = time.time() - start_time
    INPUTS['system_timestamp_inputs'].append(t)
    OUTPUTS['system_timestamp_outputs'].append(t)
    [INPUTS['{}'.format(i)].append(data['motor.{}'.format(i)]) for i in ('m1', 'm2', 'm3', 'm4')]
    p = mocap.getpose()
    OUTPUTS['x'].append(p.x)
    OUTPUTS['y'].append(p.y)
    OUTPUTS['z'].append(p.z)
    rotation = Rotation.from_matrix(p.rotmatrix)
    r = rotation.as_quat()
    OUTPUTS['qx'].append(r[0])
    OUTPUTS['qy'].append(r[1])
    OUTPUTS['qz'].append(r[2])
    OUTPUTS['qw'].append(r[3])
    print(f'mocap: x: {p.x}, y: {p.y}, z: {p.z}')
  

def log_stabilizer_callback(timestamp, data, logconf):
    global OUTPUTS
    OUTPUTS['stabilizer_timestamp'].append(timestamp)
    OUTPUTS['stabilizer.thrust'].append(data['stabilizer.thrust'])



def log_battery_voltage(timestamp, data, logconf):
    global INPUTS, start_time
    INPUTS['bat_timestamp'].append(timestamp)
    INPUTS['bat_volt'].append(data['pm.vbat'])


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=10)
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
    # cf.param.set_value('stabilizer.estimator', '0')

    # cf.param.set_value('locSrv.extQuatStdDev', 0.5)

    # reset estimator
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('kalman pos: ({}, {}, {})'.format(x, y, z))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=10)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')

    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


# def run_sequence(scf, sequence):
#     cf = scf.cf
#     for position in sequence:
#         #print('Setting position {}'.format(position))
#         for i in range(10):
#             cf.commander.send_position_setpoint(position[0],
#                                                 position[1],
#                                                 0.4,
#                                                 0)
#             time.sleep(0.02)

#     cf.commander.send_stop_setpoint()
#     # Make sure that the last packet leaves before the link is closed
#     # since the message queue is not flushed before closing
#     time.sleep(0.1)


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
    print(f'mocap: x: {x}, y: {y}, z: {z}')
    cf.extpos.send_extpose(x, y, z, qx / ql, qy / ql, qz / ql, qw / ql)

def run_sequence(scf):
    cf = scf.cf
    while True:
        t_now = time.time()
        t = t_now-t_in
        print(t)
        
        rd,_,_ = path_pars(t-t_lift,t_run,c = 0.6,tilt=0,rd_init = r_init,shape = 'circle')
        print('Setting position {},time {}'.format(rd,t))
   
        if t < t_lift:
            for i in range(10):
                cf.commander.send_position_setpoint(r_init[0],
                                                    r_init[1],
                                                    r_init[2],
                                                    0)
                time.sleep(0.001)
        elif t < n_iters*t_run + t_lift:
            for i in range(10):
                cf.commander.send_position_setpoint(rd[0],
                                                rd[1],
                                                hieght,
                                                0)
                time.sleep(0.001)
        elif t < n_iters*t_run + t_lift + t_land:
            for i in range(10):
                cf.commander.send_position_setpoint(r_init[0],
                                                    r_init[1],
                                                    0.05,
                                                    0)
                time.sleep(0.001)
        elif t >= n_iters*t_run + t_lift + t_land:
            break


    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    time_string = time.ctime().replace(':', '-')
    start_time = time.time()
    INPUTS = {'motor_timestamp': [], 'system_timestamp_inputs': [], 'm1': [], 'm2': [], 'm3': [], 'm4': [], 'bat_volt':[], 'bat_timestamp': []}
    OUTPUTS = {'system_timestamp_outputs': [], 'x': [], 'y': [], 'z': [], 'qx': [], 'qy': [], 'qz': [], 'qw': [],
     'stabilizer.thrust':[], 'stabilizer_timestamp': []}
    lg_motor = LogConfig(name='motor', period_in_ms=10) 
    [lg_motor.add_variable('motor.m{}'.format(i), 'uint16_t') for i in (1, 2, 3, 4)]
    bat_volt = LogConfig(name='pm.vbat', period_in_ms=10)
    bat_volt.add_variable('pm.vbat', 'float')
    lg_stab = LogConfig('stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.thrust', 'float')
    mocap = QtmWrapper(QTM_IP, CF_BODY)
    # time.sleep(5)

    # init_pos = mocap.getpose()
    # r_init = np.array([init_pos.x,init_pos.y,hieght])
    r_init = np.array([0., 0., hieght])
    t_in = time.time()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(1)
        scf.cf.log.add_config(lg_motor)
        scf.cf.log.add_config(bat_volt)
        scf.cf.log.add_config(lg_stab)
            

        lg_motor.data_received_cb.add_callback(log_motor_callback)
        bat_volt.data_received_cb.add_callback(log_battery_voltage)
        lg_stab.data_received_cb.add_callback(log_stabilizer_callback)
            

        bat_volt.start()
        lg_motor.start() 
        lg_stab.start()
        #mocap.on_cf_pose = lambda pose: send_extpose_rot_matrix(scf.cf, pose[0], pose[1], pose[2], pose[3])
        
        # start_position_printing(scf)

        # with MotionCommander(scf) as mc:
        #     mc.take_off(0.4)
        #     mc.stop()
        reset_estimator(scf)
        # run_sequence(scf, sequence)
        run_sequence(scf)
        lg_stab.stop()
        bat_volt.stop()
        lg_motor.stop() 
    mocap.close()
    x = input('Do you want to write the file to CSV? (y/n): ')
    if x == 'y' or x == 'Y':
        ds = {**INPUTS, **OUTPUTS}
        DF = pd.DataFrame.from_dict(ds, orient='index')
        DF = DF.transpose()
        DF.to_csv('/home/rajpal/CF programs/pos_level_cont/datasets/dynamics_dataset {}.csv'.format(time_string))
        print("written to CSV")


        