# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE Communication
"""

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