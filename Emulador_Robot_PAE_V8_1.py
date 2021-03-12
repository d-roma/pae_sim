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
CCW = 0x00
CW = 0x04
AX12_moving_L = ""
AX12_moving_R = ""
ESTAT_ROBOT = ""
ID = 0
trama = b''

instruction_set = {
    0x00: "IDLE",
    0x01: "PING",
    0x02: "READ",
    0x03: "WRITE",
    0x04: "REG_WRITE",
    0x05: "ACTION",
    0x06: "RESET",
    0x83: "SYNC_WRITE",
    0xFF: "END"
}


def f_led():
    if app.DEBUG_Consola == 1:
        if AX12[ID][AX_registers.LED] == 1:
            print("LED motor", ID, "ON")
        else:
            print("LED motor", ID, "OFF")
    if ID == 1:
        app.Led_motor_left.set(AX12[ID][AX_registers.LED])
    else:
        app.Led_motor_right.set(AX12[ID][AX_registers.LED])
    return


def f_moving_speed():
    global AX12_moving_L
    global AX12_moving_R
    velocitat = AX12[ID][AX_registers.GOAL_SPEED_L] + (AX12[ID][AX_registers.GOAL_SPEED_H] & 0x03) << 8
    if app.DEBUG_Consola == 1:
        print("Velocitat  motor", ID, "=", velocitat)
    text_motor = "v=" + str(velocitat)
    sentit = AX12[ID][AX_registers.GOAL_SPEED_H] & 0x04
    if sentit == 0:
        if app.DEBUG_Consola == 1:
            print("Sentit gir motor", ID, "= CCW")
        if ID == 1:
            AX12_moving_L = text_motor + " CCW"
        else:
            AX12_moving_R = text_motor + " CCW"
    else:
        if app.DEBUG_Consola == 1:
            print("Sentit gir motor", ID, "= CW")
        if ID == 1:
            AX12_moving_L = text_motor + " CW"
        else:
            AX12_moving_R = text_motor + " CW"
    return


def f_angle_limit():
    cw_angle = AX12[ID][AX_registers.CW_ANGLE_LIMIT_L] + (AX12[ID][AX_registers.CW_ANGLE_LIMIT_H] & 0x03) << 8
    ccw_angle = AX12[ID][AX_registers.CCW_ANGLE_LIMIT_L] + (AX12[ID][AX_registers.CCW_ANGLE_LIMIT_H] & 0x03) << 8
    if app.DEBUG_Consola == 1:
        if cw_angle == 0:
            print("Motor", ID, "gir continu en sentit horari")
        else:
            print("Motor", ID, "angle limit en sentit horari:", cw_angle)
        if ccw_angle == 0:
            print("Motor", ID, "gir continu en sentit anti-horari")
        else:
            print("Motor", ID, "angle limit en sentit anti-horari:", ccw_angle)
    return


def f_ADC_value():
    ADC_val = AX12[ID][AX_registers.ADC_VALUE]
    if app.DEBUG_Consola == 1:
        print("Mesura del ADC", ID, ":", ADC_val)
    return


def reset_modul_AX12(id_modul):
    simulador.AX12[id_modul].reset()


# comprova si hi ha error de checksum a la trama
def comprova_checksum(frame):
    len_trama = len(frame)
    chk_sum = 0
    for index in range(2, (len_trama - 1)):
        chk_sum = chk_sum + frame[index]
    chk_sum = chk_sum & 0xFF
    if (chk_sum | frame[len_trama - 1]) == 0xFF:
        # ser.write(b'\x00')
        if app.DEBUG_trama:
            print('Checksum correcte')
        return 0x00
    else:
        print('Error de Checksum')
        # ser.write(b'\x10')
        return 0x10


# comprova si hi ha error d'instruccio la trama
def comprova_instr(instruccio):
    if (instruccio < 0x07) or (instruccio == 0xFF) or (instruccio == 0x83):
        if app.DEBUG_Consola == 1:
            print("Instrucció:", instruction_set[instruccio])
        return 0x00
    else:
        print("Error d'instruccio")
        return 0x70


# crea una trama d'status per indicar que no hi ha error
Status_NoError = [0xff, 0xff, 0x01, 0x02, 0x00, 0xfc]

def send_status_packet(modul_id, error_code):
    status_frame = Status_NoError[:]
    status_frame[2] = modul_id
    status_frame[4] = error_code
    len_trama = len(status_frame)
    l_chksum = 0
    for index in range(2, (len_trama - 1)):
        l_chksum = l_chksum + status_frame[index]
    l_chksum = (~l_chksum & 0xFF)
    status_frame[len_trama - 1] = l_chksum
    string = ''.join(['0x%02X ' % b for b in status_frame])
    if app.DEBUG_trama:
        print("status packet in hex:", string)
        print("status packet in dec:", status_frame)
    ser.write(status_frame)
    return


def generate_read_packet(modul_id, address, num_param):
    global simulando
    # simulando = 0 #simulacion en pausa, para poner un breakpoint mas adelante antes de reanudar la simulacion
    status_frame = Status_NoError[:]
    status_frame[2] = modul_id
    status_frame[3] = num_param + 2
    for index in range(0, num_param):
        status_frame.insert(index + 5, AX12[modul_id][AX_registers(address + index)])
    len_trama = len(status_frame)
    l_chksum = 0
    for index in range(2, (len_trama - 1)):
        l_chksum = l_chksum + status_frame[index]
    l_chksum = (~l_chksum & 0xFF)
    status_frame[len_trama - 1] = l_chksum
    string = ''.join(['0x%02X ' % b for b in status_frame])
    if app.DEBUG_trama:
        print("status packet in hex:", string)
        print("status packet in dec:", status_frame)
    ser.write(status_frame)
    # simulando = 1 #para poner un breakpoint antes de reanudar la simulacion
    return


def generate_status_packet(id_modul, instruc, code_error, trama):
    if instruc != 2:
        send_status_packet(id_modul, code_error)
    elif instruc == INSTR_READ:
        address = trama[5]
        num_param = trama[6]
        generate_read_packet(id_modul, address, num_param)
    return


# dona l'estat del robot
def robot_status():
    global ESTAT_ROBOT
    if app.DEBUG_Consola == 1:
        print("----------- ESTAT DEL ROBOT -----------------------")
    v_left = AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_L] + (AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_H] & 0x03) << 8
    sentit_left = AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_H] & 0x04
    v_right = AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_L] + (AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_H] & 0x03) << 8
    sentit_right = AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_H] & 0x04
    if (v_left == 0) & (v_right == 0):
        if app.DEBUG_Consola == 1:
            print("Robot Parat")
        ESTAT_ROBOT = "Robot Parat"
    elif sentit_left == sentit_right:  # si motors giren mateix sentit => robot gira
        if sentit_left == CW:
            if app.DEBUG_Consola == 1:
                print("Robot Gira Esquerra")
            ESTAT_ROBOT = "Robot Gira Esquerra"
        else:
            if app.DEBUG_Consola == 1:
                print("Robot Gira Dreta")
            ESTAT_ROBOT = "Robot Gira Dreta"
    elif abs(v_left - v_right) < 1:  # si motors giren sentit contrari a mateixa velocitat=> robot va recte
        if sentit_left == CW:
            if app.DEBUG_Consola == 1:
                print("Robot Marxa Enrere")
            ESTAT_ROBOT = "Robot Marxa Enrere"
        else:
            if app.DEBUG_Consola == 1:
                print("Robot Marxa Endavant")
            ESTAT_ROBOT = "Robot Marxa Endavant"
    elif v_left > v_right:  # velocitats diferents, motor esquerre mes rapid
        if sentit_left == CW:
            if app.DEBUG_Consola == 1:
                print("Robot Gira Esquerra")
            ESTAT_ROBOT = "Robot Gira Esquerra"
        else:
            if app.DEBUG_Consola == 1:
                print("Robot Gira Dreta")
            ESTAT_ROBOT = "Robot Gira Dreta"
    elif sentit_left == CW:  # velocitats diferents, motor dret mes rapid
        if app.DEBUG_Consola == 1:
            print("Robot Gira Dreta")
        ESTAT_ROBOT = "Robot Gira Dreta"
    else:
        if app.DEBUG_Consola == 1:
            print("Robot Gira Esquerra")
        ESTAT_ROBOT = "Robot Gira Esquerra"
    return


def print_AX_MemoryMap():
    print("---------------------------------------------------")
    print("========== MOTOR ======== [1] ======= [2]")
    for key in AX_registers:
        mot1 = ''.join(['0x%02X ' % (AX12[MOTOR_ID_L][key])])  # format Hexadecimal
        mot2 = ''.join(['0x%02X ' % (AX12[MOTOR_ID_R][key])])  # format Hexadecimal
        print('{:-<23}'.format(AX12_reset_memory[key][1]), ">", mot1, "     ", mot2)
    print("####################################################")
    return


def Actualitza_AX_Memory(id_modul, adressa, nparametres):
    global trama
    for index in range(nparametres):
        AX12[id_modul][AX_registers((adressa + index))] = trama[index + 6]
    return


def print_trama():
    print(" ")
    print("####################################################")
    print("Received Instruction Frame:", trama)
    for index in range(len(trama)):
        print("Received Instruction Frame[", index, "]:", '0x%02X ' % trama[index])
    print("####################################################")
    return


def print_tipus_Instruccio(instruccio, comandament):
    error_de_instr = comprova_instr(instruccio)
    if app.DEBUG_Consola == 1:
        if error_de_instr == 0:
            if instruccio == INSTR_ACTION:
                comandament = 0x2C  # "Registered Instruction"
            print("----------- INSTRUCCIÓ i COMANDAMENT --------------")
            print("Command:", AX12_reset_memory[AX_registers(comandament)][1])
    return error_de_instr



root = tk.Tk()
root.title("EMULADOR ROBOT MiSE")
app = TkApplication(simulador, master=root)

# si esta activat el debug dels moduls presenta el contingut de memoria d'aquests
if app.DEBUG_Moduls:
    print_AX_MemoryMap()

# inicialitza el port serie
try:
    if ser.is_open:
        ser.close()
    ser.port = app.port_com
    ser.baudrate = app.baud_rate
    ser.timeout = timeout
    ser.open()
    lectura = 1
except serial.SerialException:
    mensaje = "No se puede abrir el puerto\n\t" + app.port_com
    messagebox.showerror("Error COM", mensaje)

root.lift()
root.after(1, lambda: root.lift())
root.after(2, lambda: root.focus_force())
app.mainloop()
ser.close()
grafica.terminate()
# fichero_log.close()
print("Aplicacion terminada.")
