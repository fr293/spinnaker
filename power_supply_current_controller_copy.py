# A command-line interface for the TTi power supply based current controller

import serial
import random
import time
import numpy as np



def open_controller():
    try:
        ps = serial.Serial('COM5', 19200, timeout=0.05)
        print "Connection to Current Controller Successful"
        return ps
    except serial.SerialException:
        print('Error: Could not connect to Current Controller')


def close_controller(connection_object):
    connection_object.close()


def write_values(connection_object, config, amplitude):
    supply = range(1, 5)
    random.shuffle(supply)

    current_configs = np.array([[0, 1, 1, 0],
                                [0.5, 0.5, 1, 1],
                                [1, 1, 0.5, 0.5],
                                [1, 0, 0, 1],
                                [-1, -1, 1, 1],
                                [1, -1, -1, 1]])

    current_amplitudes = np.array([0.5, 1, 1.5, 2, 2.5])

    current_values = current_amplitudes[amplitude-1] * current_configs[config-1]
    current_values[current_values == 0] = 0.001

    for i in supply:
        connection_object.write('PW ' + str(i) + ' ' + str(current_values[i-1]) + '\r\n')
        time.sleep(0.02)


def switch_on(connection_object):
    connection_object.write('P_ON\r\n')


def switch_off(connection_object):
    connection_object.write('P_OFF\r\n')


def light_on(connection_object):
    connection_object.write('Light_ON' + '\r\n')


def light_off(connection_object):
    connection_object.write('Light_OFF' + '\r\n')

def trigger_currents(connection_object, duration):
    switch_on(connection_object)
    time.sleep(duration)
    switch_off(connection_object)