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

root = tk.Tk()

OUTPUT_FILE_NAME = "movement.log"

fichero_log = open(OUTPUT_FILE_NAME, "w")  # Con "w", reiniciamos el fichero si ya existia
DEMO = 0
DEBUG_LEVEL = 3

if DEBUG_LEVEL > 2:
    log_file = open(OUTPUT_FILE_NAME, "w")

# Las unidades de distancia se consideran en mm
SIM_STEP_MS_TIME = 50  # en ms
MAX_SIM_STEPS = 24000
delay_Simul = 0.090  # en s
delay_Puerto = 0.001  # en s
DELTA_T = SIM_STEP_MS_TIME / 1000.0  # convertimos el paso de la simul a s
CNTS_2_MM = 1000.0 * DELTA_T / 1023  # conversion del valor de velocidad del AX12 [0..3FF] a mm/s
L_AXIS = 1.0
t_last_upd = 0.0

ORIENT_L = 1  # Orientacion motor izquierdo
ORIENT_R = -1  # Orientacion motor derecho
# V_inicial_demo_L = 0x3FF
# V_inicial_demo_R = 0x7FF
V_inicial_demo_L = 0
V_inicial_demo_R = 0

DEBUG_trama_check = IntVar()
DEBUG_Moduls_check = IntVar()
DEBUG_Consola_check = IntVar()
SIMUL_check = IntVar()
SIMUL_Save_check = IntVar()
DEBUG_trama_check.set(0)
DEBUG_Moduls_check.set(0)
DEBUG_Consola_check.set(0)
SIMUL_Save_check.set(0)
DEBUG_trama = DEBUG_trama_check.get()
DEBUG_Moduls = DEBUG_Moduls_check.get()
DEBUG_Consola = DEBUG_Consola_check.get()
SIMUL_On_Off = SIMUL_check.get()
SIMUL_Save = SIMUL_Save_check.get()
texto_trama = StringVar()
texto_motor_left = StringVar()
texto_motor_right = StringVar()
valor_barra_izq = IntVar()
valor_barra_der = IntVar()
valor_barra_cent = IntVar()
Led_motor_left = IntVar()
Led_motor_right = IntVar()
estados = {0: "OFF", 1: "ON "}

Default_port_com = 'COM5'
port_com = Default_port_com
baud_rate = 115200
baud_rates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000]
#timeout = 0.002  # para lectura puerto, en s
timeout = 0.0  # para lectura puero, en s
lectura = 0
seguir = 1
ser = serial.Serial()  # se configura luego

INSTR_IDLE = 0x00
INSTR_END = 0xFF
INSTR_STOP_THREAD = 0xEE
INSTR_STOP_SIMUL = 0xCC
INSTR_SIMUL_ENDED = 0xDD
INSTR_Actualizar_graf = 0xBB

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

# instruccio = INSTR_IDLE
simulacio = INSTR_IDLE
simulando = 0
actualizar_graf = 0

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


def puertos_serie():
    lista_puertos = []
    puertos = serial.tools.list_ports.comports()
    for puerto in puertos:
        lista_puertos.append(str(puerto[0]))
    lista_puertos.append('None')
    lista_puertos.sort()
    print(lista_puertos)
    return lista_puertos


def refrescar_puertos(cb_lista):
    global port_com
    print("Actualizando lista puertos")
    lista_puertos = puertos_serie()
    cb_lista.config(values=lista_puertos)
    indice = 0
    for puerto in lista_puertos:
        if puerto == Default_port_com:
            cb_lista.current(indice)
        indice += 1
    port_com = cb_lista.get()
    print("Puerto seleccionado: ", port_com)


def f_led():
    if DEBUG_Consola == 1:
        if AX12[ID][AX_registers.LED] == 1:
            print("LED motor", ID, "ON")
        else:
            print("LED motor", ID, "OFF")
    if ID == 1:
        Led_motor_left.set(AX12[ID][AX_registers.LED])
    else:
        Led_motor_right.set(AX12[ID][AX_registers.LED])
    return


def f_moving_speed():
    global AX12_moving_L
    global AX12_moving_R
    velocitat = AX12[ID][AX_registers.GOAL_SPEED_L] + (AX12[ID][AX_registers.GOAL_SPEED_H] & 0x03) << 8
    if DEBUG_Consola == 1:
        print("Velocitat  motor", ID, "=", velocitat)
    text_motor = "v=" + str(velocitat)
    sentit = AX12[ID][AX_registers.GOAL_SPEED_H] & 0x04
    if sentit == 0:
        if DEBUG_Consola == 1:
            print("Sentit gir motor", ID, "= CCW")
        if ID == 1:
            AX12_moving_L = text_motor + " CCW"
        else:
            AX12_moving_R = text_motor + " CCW"
    else:
        if DEBUG_Consola == 1:
            print("Sentit gir motor", ID, "= CW")
        if ID == 1:
            AX12_moving_L = text_motor + " CW"
        else:
            AX12_moving_R = text_motor + " CW"
    return


def f_angle_limit():
    cw_angle = AX12[ID][AX_registers.CW_ANGLE_LIMIT_L] + (AX12[ID][AX_registers.CW_ANGLE_LIMIT_H] & 0x03) << 8
    ccw_angle = AX12[ID][AX_registers.CCW_ANGLE_LIMIT_L] + (AX12[ID][AX_registers.CCW_ANGLE_LIMIT_H] & 0x03) << 8
    if DEBUG_Consola == 1:
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
    if DEBUG_Consola == 1:
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
        if DEBUG_trama:
            print('Checksum correcte')
        return 0x00
    else:
        print('Error de Checksum')
        # ser.write(b'\x10')
        return 0x10


# comprova si hi ha error d'instruccio la trama
def comprova_instr(instruccio):
    if (instruccio < 0x07) or (instruccio == 0xFF) or (instruccio == 0x83):
        if DEBUG_Consola == 1:
            print("Instrucció:", instruction_set[instruccio])
        return 0x00
    else:
        print("Error d'instruccio")
        return 0x70


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
    if DEBUG_trama:
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
    if DEBUG_trama:
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
    if DEBUG_Consola == 1:
        print("----------- ESTAT DEL ROBOT -----------------------")
    v_left = AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_L] + (AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_H] & 0x03) << 8
    sentit_left = AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_H] & 0x04
    v_right = AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_L] + (AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_H] & 0x03) << 8
    sentit_right = AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_H] & 0x04
    if (v_left == 0) & (v_right == 0):
        if DEBUG_Consola == 1:
            print("Robot Parat")
        ESTAT_ROBOT = "Robot Parat"
    elif sentit_left == sentit_right:  # si motors giren mateix sentit => robot gira
        if sentit_left == CW:
            if DEBUG_Consola == 1:
                print("Robot Gira Esquerra")
            ESTAT_ROBOT = "Robot Gira Esquerra"
        else:
            if DEBUG_Consola == 1:
                print("Robot Gira Dreta")
            ESTAT_ROBOT = "Robot Gira Dreta"
    elif abs(v_left - v_right) < 1:  # si motors giren sentit contrari a mateixa velocitat=> robot va recte
        if sentit_left == CW:
            if DEBUG_Consola == 1:
                print("Robot Marxa Enrere")
            ESTAT_ROBOT = "Robot Marxa Enrere"
        else:
            if DEBUG_Consola == 1:
                print("Robot Marxa Endavant")
            ESTAT_ROBOT = "Robot Marxa Endavant"
    elif v_left > v_right:  # velocitats diferents, motor esquerre mes rapid
        if sentit_left == CW:
            if DEBUG_Consola == 1:
                print("Robot Gira Esquerra")
            ESTAT_ROBOT = "Robot Gira Esquerra"
        else:
            if DEBUG_Consola == 1:
                print("Robot Gira Dreta")
            ESTAT_ROBOT = "Robot Gira Dreta"
    elif sentit_left == CW:  # velocitats diferents, motor dret mes rapid
        if DEBUG_Consola == 1:
            print("Robot Gira Dreta")
        ESTAT_ROBOT = "Robot Gira Dreta"
    else:
        if DEBUG_Consola == 1:
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
    if DEBUG_Consola == 1:
        if error_de_instr == 0:
            if instruccio == INSTR_ACTION:
                comandament = 0x2C  # "Registered Instruction"
            print("----------- INSTRUCCIÓ i COMANDAMENT --------------")
            print("Command:", AX12_reset_memory[AX_registers(comandament)][1])
    return error_de_instr


# crea una trama d'status per indicar que no hi ha error
Status_NoError = [0xff, 0xff, 0x01, 0x02, 0x00, 0xfc]

# aquesta instruccio es no fer res
instruccio = INSTR_IDLE
# si esta activat el debug dels moduls presenta el contingut de memoria d'aquests
if DEBUG_Moduls:
    print_AX_MemoryMap()


class Hilo(threading.Thread):  # El thread que ira leyendo del puerto serie
    def __init__(self, nombre, cola, cola_hilo):
        threading.Thread.__init__(self)
        self.cola = cola
        self.cola_hilo = cola_hilo

    def AX12_func(self, argument):
        switcher = {
            # 0x00: "Model Number(L)",
            # 0x01: "Model NUmber(H)",
            # 0x02: "Firmware Version",
            # 0x03:  "ID",
            # 0x04: "Baud Rate",
            # 0x05: "Return Delay Time",
            0x06: f_angle_limit,
            0x07: f_angle_limit,
            0x08: f_angle_limit,
            0x09: f_angle_limit,
            # 0x0A: "Reserved",
            # 0x0B: "High Temp. Limit",
            # 0x0C: "Low Voltage Limit",
            # 0x0D: "High Voltage Limit",
            # 0x0E: "Max Torque(L)",
            # 0x0F: "Max Torque(H)",
            # 0x10: "Status Return Level",
            # 0x11: "Alarm LED",
            # 0x12: "Alarm Shoutdown",
            # 0x13: "Reserved",
            # 0x14: "Down Calibration(L)",
            # 0x15: "Down Calibration(H)",
            # 0x16: "Up Calibration(L)",
            # 0x17: "Up Calibration(H)",
            # 0x18: "Torque Enable",
            0x19: f_led,
            # 0x1A: f_Left_IR,
            # 0x1B: f_Center_IR,
            # 0x1C: f_Right_IR,
            # 0x1D: "CCW Compliance Slope",
            # 0x1E: "Goal Position(L)",
            # 0x1F: "Goal Position(H)",
            0x20: f_moving_speed,
            0x21: f_moving_speed,
            # 0x22: "Torque Limit(L)",
            # 0x23: "Torque Limit(H)",
            # 0x24: "Present Position(L)",
            # 0x25: "Present Position(H)",
            # 0x26: "Present Speed(L)",
            # 0x27: "Present Speed(H)",
            # 0x28: "Present Load(L)",
            # 0x29: "Present Load(H)",
            # 0x2A: "Present Voltage",
            # 0x2B: "Present Temperature",
            # 0x2C: "Registered Instruction",
            0x2D: f_ADC_value,
            # 0x2E: "Moving",
            # 0x2F: "Lock",
            # 0x30: "Punch(L)",
            # 0x31: "Punch(H)"
        }
        func = switcher.get(argument, lambda: "Funció no implementada")
        func()

    def leer_puerto(self, la_cola, la_cola_hilo):
        global instruccio
        global trama
        global ID
        global AX12_moving_L
        global AX12_moving_R
        global lectura
        Lista_acciones = []  # para almacenar acciones recibidas con REG_WRITE
        AX12_moving_L = "PARAT"
        AX12_moving_R = "PARAT"
        print("Lectura puerto iniciada...", instruccio)
        while instruccio != INSTR_END:
            if not la_cola.empty():
                mensaje = la_cola.get()
                if mensaje == INSTR_END:
                    instruccio = mensaje
                    if DEBUG_Consola == 1:
                        print("instruccio rebuda: ", instruccio)
                if DEBUG_Consola == 1:
                    print("mensaje recibido en leer_puerto: ", mensaje)
            elif lectura == 1:
                if ser.is_open:
                    trama = ser.read(16)
                if trama != b'':
                    items_array = len(trama)
                    if DEBUG_Consola == 1:
                        print("Número de items en el array:", items_array)
                    if items_array >= 6:  # en cas contrari no es un instruction packet
                        if DEBUG_Consola == 1:
                            print("")
                            print("****************************************************")
                        instruccio = trama[4]  # posicio de la trama on esta la instruccio
                        # si esta activat el debug mostra la trama que arriba
                        if DEBUG_trama:
                            print_trama()
                        # mira quina instruccio es i si no es una de les que existeix dona un error
                        instr_error = print_tipus_Instruccio(instruccio, trama[5])
                        # copmprova el checksum rebut amb el calculat
                        chk_sum_error = comprova_checksum(trama[0:trama[3] + 4])
                        # error indicara si hi ha un error, sigui d'instruccio o de checksum
                        error = (chk_sum_error | instr_error)
                        ID = trama[2]  # posicio a la trama del identificador edl modul
                        if ID != 0xFE:  # si el ID no es el de broadcast respon amb un status packet
                            # send_status_packet(ID, instruccio, error, trama)
                            generate_status_packet(ID, instruccio, error, trama)
                        else:
                            print("Broadcasting ID Instruction Packet")
                        if error == 0:  # si no hi ha hagut cap error analitza la instruccio i l'executa
                            if instruccio == INSTR_WRITE:  # per ara nomes executa la instruccio WRITE
                                n_parametres = trama[3] - 3
                                # posicio de la tama on esta l'adreça de memoria del modul a escriure
                                address = trama[5]
                                Actualitza_AX_Memory(ID, address, n_parametres)
                                self.AX12_func(address)  # informa quin comandament s'ha executat
                                if DEBUG_Moduls:
                                    print_AX_MemoryMap()
                                robot_status()
                                # label_trama.set(str(trama)[3:-1])
                                texto_trama.set(ESTAT_ROBOT)
                                texto_motor_left.set(AX12_moving_L)
                                texto_motor_right.set(AX12_moving_R)

                            elif instruccio == INSTR_REG_WR:
                                # tenemos que ir almacenando las acciones pendientes:
                                ID = trama[2]
                                direccion = trama[5]
                                num_param = trama[3] - 3
                                parametros = []
                                for param in range(num_param):
                                    parametros.append(trama[6 + param])
                                Lista_acciones.append([ID, direccion, num_param, parametros])
                                if DEBUG_Consola == 1:
                                    print(Lista_acciones)
                            elif instruccio == INSTR_ACTION:
                                if not Lista_acciones:
                                    print("No hay acciones pendientes...")
                                else:
                                    # print("Acciones pendientes:")
                                    for index in range(len(Lista_acciones)):
                                        if DEBUG_Consola == 1:
                                            print("Accion pendiente:", Lista_acciones[index])
                                        ID = Lista_acciones[index][0]
                                        direccion = Lista_acciones[index][1]
                                        num_param = Lista_acciones[index][2]
                                        for param in range(num_param):
                                            parametro = Lista_acciones[index][3][param]
                                            # Hay que actualizar la memoria del (los) modulo(s):
                                            AX12[ID][AX_registers(direccion + param)] = parametro
                                        self.AX12_func(direccion)  # informa quin comandament s'ha executat
                                    robot_status()
                                    texto_trama.set(ESTAT_ROBOT)
                                    texto_motor_left.set(AX12_moving_L)
                                    texto_motor_right.set(AX12_moving_R)
                                    # una vez ejecutadas las acciones pendientes, limpiamos la lista:
                                    Lista_acciones = []
                        else:  # Ha habido un error, hay que hacer limpieza:
                            Lista_acciones = []
                            ser.flushInput()
                            # Algo mas??
                time.sleep(delay_Puerto)  # para dejar tiempo de procesador al hilo principal

        la_cola_hilo.put(INSTR_STOP_THREAD)
        if DEBUG_Consola == 1:
            print("lectura parada")
        lectura = 0  # creo que esto no es necesario?

    def run(self):
        self.leer_puerto(self.cola, self.cola_hilo)
        if DEBUG_Consola == 1:
            print("funcion run terminada")


class Simul(threading.Thread):
    def __init__(self, nombre, cola, cola_simul):
        threading.Thread.__init__(self)
        self.cola = cola
        self.cola_simul = cola_simul

    def movement(self, la_cola, la_cola_simul):
        global simulacio, simulando, actualizar_graf
        print("Hilo simulacion movimiento iniciado, simulación en pausa...")
        # Pondremos una velocidad inicial, para las pruebas y/o la demo:
        AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_L] = V_inicial_demo_L & 0xFF
        AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_H] = (V_inicial_demo_L >> 8) & 0x07
        AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_L] = V_inicial_demo_R & 0xFF
        AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_H] = (V_inicial_demo_R >> 8) & 0x07
        while simulacio != INSTR_STOP_SIMUL:
            if simulando == 1:
                if simulador.update_movement_simulator_values():
                    # Actualizamos las barras graficas:
                    valor_barra_izq.set(AXS1[AX_registers.IR_LEFT])
                    valor_barra_der.set(AXS1[AX_registers.IR_RIGHT])
                    valor_barra_cent.set(AXS1[AX_registers.IR_CENTER])
                # else:
                # en principio, no hacer nada?...
            time.sleep(delay_Simul)  # para dejar tiempo de procesador al hilo principal
        la_cola_simul.put(INSTR_STOP_SIMUL)
        if DEBUG_Consola == 1:
            print("simulacion parada")
        simulando = 0  # creo que esto no es necesario?
        simulacio = INSTR_SIMUL_ENDED

    def run(self):
        self.movement(self.cola, self.cola_simul)
        if DEBUG_Consola == 1:
            print("funcion run terminada")


# Funciones para gestionar la aplicacion grafica
class Application(tk.Frame):
    counter = 0
    contador = 0
    puntos = []

    def __init__(self, simulador, master=None):
        super().__init__(master)
        self.simulador = simulador
        self.master = master
        self.grid()
        self.create_widgets()
        self.crear_hilo()
        self.crear_simul()
        self.check_queue()

    def create_widgets(self):
        global port_com
        global baud_rate
        self.spacer_up = Label(self, width=5, height=1)
        self.spacer_up.grid(row=0, column=0)
        self.spacer_center = Label(self, width=5, height=1)
        self.spacer_center.grid(row=20, column=0)
        self.spacer_bottom = Label(self, width=5, height=1)
        self.spacer_bottom.grid(row=40, column=10)

        self.Label_Debug = Label(self, text="Debug:")
        self.Label_Debug.grid(row=1, column=1, sticky=E)

        self.Debug_frame = Checkbutton(self, variable=DEBUG_trama_check)
        self.Debug_frame["text"] = "Debug Trames"
        self.Debug_frame["fg"] = "blue"
        self.Debug_frame["command"] = self.set_debug_frames
        self.Debug_frame.grid(row=1, column=2, sticky=W)

        self.Debug_module = Checkbutton(self, variable=DEBUG_Moduls_check)
        self.Debug_module["text"] = "Debug Moduls"
        self.Debug_module["fg"] = "blue"
        self.Debug_module["command"] = self.set_debug_moduls
        self.Debug_module.grid(row=1, column=3, sticky=W)

        self.Debug_consola = Checkbutton(self, variable=DEBUG_Consola_check)
        self.Debug_consola["text"] = "Consola On/off"
        self.Debug_consola["fg"] = "blue"
        self.Debug_consola["command"] = self.set_debug_consola
        self.Debug_consola.grid(row=1, column=4, sticky=E)

        self.Label_Simul = Label(self, text="Simulació:")
        self.Label_Simul.grid(row=2, column=1)

        self.Simul_On_Off = Checkbutton(self, variable=SIMUL_check)
        self.Simul_On_Off["text"] = "Simul On/off"
        self.Simul_On_Off["fg"] = "blue"
        self.Simul_On_Off["command"] = self.set_Simul_On_Off
        self.Simul_On_Off.grid(row=2, column=2, sticky=W)

        self.Simul_grabar = Checkbutton(self, fg="blue", variable=SIMUL_Save_check)
        self.Simul_grabar["text"] = "Desar"
        self.Simul_grabar["command"] = self.grabar_Simul_OnOff
        self.Simul_grabar.grid(row=2, column=3, sticky=W)

        self.Simul_reset = Button(self, text="Reset", fg="blue", command=self.reset_simul)
        self.Simul_reset.grid(row=2, column=4, sticky=W)

        self.label_robot = Label(self, text="ESTAT ROBOT:")
        self.label_robot.grid(row=3, column=1)
        self.label_trama = Label(self, textvariable=texto_trama)
        texto_trama.set("ROBOT PARAT")
        self.label_trama.grid(row=3, column=2, columnspan=2, sticky=W)

        self.label_AX12_L = Label(self, text="MOTOR Esq.:")
        self.label_AX12_L.grid(row=4, column=1)
        self.label_motor_left = Label(self, textvariable=texto_motor_left)
        texto_motor_left.set("PARAT")
        self.label_motor_left.grid(row=4, column=2)
        self.label_AX12_R = Label(self, text="MOTOR Dret:")
        self.label_AX12_R.grid(row=5, column=1)
        self.label_motor_right = Label(self, textvariable=texto_motor_right)
        texto_motor_right.set("PARAT")
        self.label_motor_right.grid(row=5, column=2)

        self.led_motor_left = Radiobutton(self, text="Led Esq.", value=1,
                                          variable=Led_motor_left, state=DISABLED)
        self.led_motor_left.grid(row=4, column=3)
        self.led_motor_right = Radiobutton(self, text="Led Dret", value=1,
                                           variable=Led_motor_right, state=DISABLED)
        self.led_motor_right.grid(row=5, column=3)

        self.label_izq = Label(self, text="IR Esq.")
        self.label_izq.grid(row=6, column=1)
        self.progress_bar_izq = ttk.Progressbar(self, orient="horizontal",
                                                length=255, maximum=255,
                                                mode="determinate", variable=valor_barra_izq)
        self.progress_bar_izq["value"] = 0
        self.progress_bar_izq.place(relx=0.5, rely=0.5, relwidth=0.80, anchor=tk.CENTER)
        self.progress_bar_izq.grid(row=6, column=2, columnspan=3)

        self.label_der = Label(self, text="IR dret")
        self.label_der.grid(row=7, column=1)
        self.progress_bar_der = ttk.Progressbar(self, orient="horizontal",
                                                length=255, maximum=255,
                                                mode="determinate", variable=valor_barra_der)
        self.progress_bar_der["value"] = 200
        self.progress_bar_der.place(relx=0.5, rely=0.5, relwidth=0.80, anchor=tk.CENTER)
        self.progress_bar_der.grid(row=7, column=2, columnspan=3)

        self.label_der = Label(self, text="IR Centre")
        self.label_der.grid(row=8, column=1)
        self.progress_bar_der = ttk.Progressbar(self, orient="horizontal",
                                                length=255, maximum=255,
                                                mode="determinate", variable=valor_barra_cent)
        self.progress_bar_der["value"] = 200
        self.progress_bar_der.place(relx=0.5, rely=0.5, relwidth=0.80, anchor=tk.CENTER)
        self.progress_bar_der.grid(row=8, column=2, columnspan=3)

        self.quit = tk.Button(self, text="SORTIR", fg="red",
                              command=self.salir)
        self.quit.grid(row=21, column=3)

        self.hi_there = tk.Button(self)
        self.hi_there["text"] = "About"
        self.hi_there["fg"] = "blue"
        self.hi_there["command"] = self.say_hi
        self.hi_there.grid(row=21, column=4)

        self.cb = ttk.Combobox(self, values=lista_puertos, width=10)
        self.cb.current(0)
        self.cb.grid(row=21, column=1)
        self.cb.bind('<<ComboboxSelected>>', self.on_select)
        indice = 0
        for puerto in lista_puertos:
            if puerto == Default_port_com:
                self.cb.current(indice)
            indice += 1
        port_com = self.cb.get()
        print("Puerto seleccionado: ", port_com)

        self.cb_rate = ttk.Combobox(self, state="readonly", width=12,
                                    values=baud_rates)  # desplegable con velocidades de bps estandares
        self.cb_rate.current(4)
        self.cb_rate.grid(row=21, column=2)
        self.cb_rate.bind('<<ComboboxSelected>>', self.on_select_rate)
        baud_rate = self.cb_rate.get()
        print("Velocidad: ", baud_rate)

        self.refrescar = tk.Button(self, command=lambda: refrescar_puertos(self.cb))
        self.refrescar["text"] = "Refrescar"
        self.refrescar["fg"] = "blue"
        self.refrescar.grid(row=22, column=1, sticky=W)

        self.logo_frame = Label(text=" ")
        self.logo_frame.grid(row=30, column=0, columnspan=4, pady=10)
        try:
            self.logo = PhotoImage(file="logoUB.png")
            self.logo_frame["image"] = self.logo
        except tk.TclError:
            self.logo_frame["text"] = "Universitat de Barcelona"
            return

    def on_select(self, event=None):
        global lectura
        global ser
        global port_com
        lectura = 0
        seleccion = self.cb.get()
        port_com = seleccion
        print("Nuevo puerto seleccionado:", port_com)
        try:
            if ser.is_open:
                ser.close()
            ser.port = port_com
            ser.baudrate = baud_rate
            ser.timeout = timeout
            ser.open()
            lectura = 1
        except serial.SerialException:
            mensaje_error = "No se puede abrir el puerto\n\t" + port_com
            messagebox.showerror("Error COM", mensaje_error)
            lectura = 0

    def on_select_rate(self, event=None):
        global baud_rate
        baud_rate = self.cb_rate.get()
        ser.baudrate = baud_rate
        print("Baud rate seleccionado: ", baud_rate, " bps")

    def check_queue(self):
        global instruccio, simulacio, simulando
        # revisar la cola para evitar bloque en la interfaz
        if not self.cola_hilo.empty():
            # obtener mensaje de la cola
            instruccio = self.cola_hilo.get()
            if DEBUG_Consola == 1:
                print("get text from read queue:", instruccio)
        if instruccio == INSTR_STOP_THREAD:
            simulando = 0  # parar la simulacion
            simulacio = INSTR_STOP_SIMUL  # terminar el hilo de la simulacion
            return INSTR_STOP_THREAD
        root.after(100, self.check_queue)

    def say_hi(self):
        print("J. Bosch & C. Serre,")
        print("UB, 2020-2021.")
        messagebox.showinfo("Autors", "J. Bosch & C. Serre\nUB, 2020-2021.")

    def set_debug_frames(self):
        global DEBUG_trama
        DEBUG_trama = DEBUG_trama_check.get()

    def set_debug_moduls(self):
        global DEBUG_Moduls
        DEBUG_Moduls = DEBUG_Moduls_check.get()

    def set_debug_consola(self):
        global DEBUG_Consola
        DEBUG_Consola = DEBUG_Consola_check.get()

    def set_Simul_On_Off(self):
        global simulando, SIMUL_On_Off
        SIMUL_On_Off = SIMUL_check.get()
        simulando = SIMUL_On_Off
        print("Simulacion ", estados[SIMUL_On_Off])

    def grabar_Simul_OnOff(self):
        global SIMUL_Save
        if SIMUL_Save_check.get():
            self.simulador.enable_data_logging()
            SIMUL_Save = True
        else:
            self.simulador.disable_data_logging()
            SIMUL_Save = False

    def crear_hilo(self):
        # crear cola para comunicar/enviar tareas al thread
        self.cola = queue.Queue()
        self.cola_hilo = queue.Queue()
        if DEBUG_Consola == 1:
            print(self.cola, self.cola_hilo)
        # crear el thread
        hilo = Hilo("puerto", self.cola, self.cola_hilo)
        # iniciar thread
        hilo.start()

    def crear_simul(self):
        # crear cola para comunicar/enviar tareas al thread
        self.cola_simul = queue.Queue()
        if DEBUG_Consola == 1:
            print(self.cola, self.cola_simul)
        # crear el thread
        simul = Simul("Simul", self.cola, self.cola_simul)
        self.simulador.update_sensor_data()
        print("Robot @:", self.simulador.x, self.simulador.y)
        print("Distancia: Izq.=", self.simulador.AXS1[AX_registers.IR_LEFT], ", Centro =",
              self.simulador.AXS1[AX_registers.IR_CENTER], ", Der. =",
              self.simulador.AXS1[AX_registers.IR_RIGHT])
        # Actualizamos las barras graficas:
        valor_barra_izq.set(self.simulador.AXS1[AX_registers.IR_LEFT])
        valor_barra_der.set(self.simulador.AXS1[AX_registers.IR_RIGHT])
        valor_barra_cent.set(self.simulador.AXS1[AX_registers.IR_CENTER])

        # iniciar thread
        simul.start()

    def reset_simul(self):
        print("Simulacio reiniciada")
        SIMUL_check.set(0)
        self.set_Simul_On_Off()
        # reiniciar el robot_pos
        # reiniciar el contador de simulacion
        self.contador = 0
        # Actualizamos las barras graficas:
        valor_barra_izq.set(self.simulador.AX12[0][AX_registers.IR_LEFT])
        valor_barra_der.set(self.simulador.AX12[0][AX_registers.IR_RIGHT])
        valor_barra_cent.set(self.simulador.AX12[0][AX_registers.IR_CENTER])
        self.simulador.reset_plot()

    def salir(self):
        self.cola.put(INSTR_END)  # terminar funcion leer_puerto
        if (instruccio == INSTR_STOP_THREAD) & (simulacio == INSTR_SIMUL_ENDED):
            self.simulador.close()
             # Cerramos la ventana del plot
            if DEBUG_Consola == 1:
                print("Hilos finalizados")
            self.master.destroy()  # Termina la aplicacion
        else:
            root.after(200, self.salir)


lista_puertos = puertos_serie()
root.title("EMULADOR ROBOT MiSE")
app = Application(simulador, master=root)
# inicialitza el port serie
try:
    if ser.is_open:
        ser.close()
    ser.port = port_com
    ser.baudrate = baud_rate
    ser.timeout = timeout
    ser.open()
    lectura = 1
except serial.SerialException:
    mensaje = "No se puede abrir el puerto\n\t" + port_com
    messagebox.showerror("Error COM", mensaje)

root.lift()
root.after(1, lambda: root.lift())
root.after(2, lambda: root.focus_force())
app.mainloop()
ser.close()
grafica.terminate()
# fichero_log.close()
print("Aplicacion terminada.")
