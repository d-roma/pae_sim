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

# Simulator initial positions
INITIAL_POS_X = 50
INITIAL_POS_Y = 350
INITIAL_POS_THETA = math.pi / 2

# Simulator update parameters
SIM_STEP_MS_TIME = 200  # en ms
MAX_SIM_STEPS = 24000

# Socket to graphical app
SOCKET_IP = 'localhost'
SOCKET_PORT = 6000

fichero_habitacion = "habitacion_003.h"
OUTPUT_FILE_NAME = "movement.log"

Comando_plot = "python plot_movement.py"

DEFAULT_COM_PORT = 'COM7'
DEFAULT_BAUD_RATE = 115200
# SERIAL_TIMEOUT = 0.002  # para lectura puerto, en s
SERIAL_TIMEOUT = 0  # para lectura puerto, en s

# V_inicial_demo_L = 0x3FF
# V_inicial_demo_R = 0x7FF
V_inicial_demo_L = 0
V_inicial_demo_R = 0
