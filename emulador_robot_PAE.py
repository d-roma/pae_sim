# -*- coding: utf-8 -*-
"""
Spyder Editor
"""

# !/usr/bin/python3

import tkinter as tk
import subprocess

import logging

from global_config import  fichero_habitacion, MOTOR_ID_L, MOTOR_ID_R, SENSOR_ID
from AX import AX
from world import World
from sim import Simulator
from gui import TkApplication

logging.basicConfig(format='%(asctime)s %(module)s - %(levelname)s - %(message)s', level=logging.DEBUG)

try:
    comando_plot = "python plot_movement.py"
    grafica = subprocess.Popen(comando_plot, stdin=subprocess.PIPE, text=True)
except OSError as e:
    print(e)
try:
    grafica.communicate("Ventana plot, creada desde el Emulador PAE!", 0.001)
except subprocess.TimeoutExpired:
    pass

# AX12 motors, left and right
AX12 = {MOTOR_ID_L: AX(MOTOR_ID_L),
        MOTOR_ID_R: AX(MOTOR_ID_R), }
# AXS1 sensor modules
AXS1 = AX(SENSOR_ID)


habitacion = World(fichero_habitacion)
simulador = Simulator(habitacion, AX12, AXS1)

# Iniciamos el simulador
#simulador.start()

logging.info("Robots en (%d, %d) y angulo %d" % (simulador.x, simulador.y, simulador.theta))

root = tk.Tk()
root.title("EMULADOR ROBOT PAE")
app = TkApplication(simulador, AX12, AXS1, root=root)

root.lift()
root.after(1, lambda: root.lift())
root.after(2, lambda: root.focus_force())
app.mainloop()
grafica.terminate()
logging.warning("Aplicacion terminada.")
