# -*- coding: utf-8 -*-
"""
Spyder Editor
"""

# !/usr/bin/python3

import serial
import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
import threading
import queue
import time
import serial.tools.list_ports
import subprocess

import logging

from global_config import  fichero_habitacion, MOTOR_ID_R, MOTOR_ID_L
from AX import *
from world import World
from sim import Simulator
from gui import TkApplication

logging.basicConfig(format='%(asctime)s %(message)s', level=logging.DEBUG)

try:
    comando_plot = "python plot_movement.py"
    grafica = subprocess.Popen(comando_plot, stdin=subprocess.PIPE, text=True)
except OSError as e:
    print(e)
try:
    grafica.communicate("Ventana plot, creada desde el Emulador PAE!", 0.001)
except subprocess.TimeoutExpired:
    pass

habitacion = World(fichero_habitacion)
simulador = Simulator(habitacion)
AX12 = simulador.AX12
AXS1 = simulador.AXS1

logging.info("Robots en (%d, %d) y angulo %d" % (simulador.x, simulador.y, simulador.theta))


#timeout = 0.002  # para lectura puerto, en s
timeout = 0.0  # para lectura puero, en s
lectura = 0
seguir = 1
ser = serial.Serial()  # se configura luego


INSTR_PING = 0x01
INSTR_READ = 0x02
INSTR_WRITE = 0x03
INSTR_REG_WR = 0x04
INSTR_ACTION = 0x05
INSTR_RESET = 0x06
INSTR_SYNC_WRT = 0x83

ID_L = 0x01
ID_R = 0x02

AX12_moving_L = ""
AX12_moving_R = ""
ESTAT_ROBOT = ""
ID = 0
trama = b''





def reset_modul_AX12(id_modul):
    simulador.AX12[id_modul].reset()



































root = tk.Tk()
root.title("EMULADOR ROBOT PAE")
app = TkApplication(simulador, master=root)
# Iniciamos el simulador
simulador.run()

# si esta activat el debug dels moduls presenta el contingut de memoria d'aquests
#if app.DEBUG_Moduls:
#    print_AX_MemoryMap()

# # inicialitza el port serie
# try:
#     if ser.is_open:
#         ser.close()
#     ser.port = app.port_com
#     ser.baudrate = app.baud_rate
#     ser.timeout = timeout
#     ser.open()
#     lectura = 1
# except serial.SerialException:
#     mensaje = "No se puede abrir el puerto\n\t" + app.port_com
#     messagebox.showerror("Error COM", mensaje)

root.lift()
root.after(1, lambda: root.lift())
root.after(2, lambda: root.focus_force())
app.mainloop()
ser.close()
grafica.terminate()
# fichero_log.close()
print("Aplicacion terminada.")
