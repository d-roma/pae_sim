# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE global configuration parameters
"""

import math

MOTOR_ID_L = 0x01
MOTOR_ID_R = 0x02
SENSOR_ID = 100

# Funciones y definiciones para la simulacion del movimiento
WORLD__N_BYTES = 4  # 4 bytes
WORLD__N_BITS = 32  # 32 bits
WORLD__MAX_2POW = int(math.log2(WORLD__N_BYTES * 8))

#Simulator initial positions
INITIAL_POS_X = 50
INITIAL_POS_Y = 350
INITIAL_POS_THETA = math.pi / 2

# Socket to graphical app
SOCKET_IP = 'localhost'
SOCKET_PORT = 6000


fichero_habitacion = "habitacion_003.h"
OUTPUT_FILE_NAME = "movement.log"

Comando_plot = "python plot_movement.py"

DEMO = 0
DEBUG_LEVEL = 3

Default_port_com = 'COM5'
port_com = Default_port_com
baud_rate = 115200
baud_rates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000]
timeout = 0.002  # para lectura puerto, en s
lectura = 0
seguir = 1