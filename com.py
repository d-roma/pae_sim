# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE Communication
"""

import serial
import serial.tools.list_ports
import threading

from global_config import *

BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000]

def lista_puertos_serie():
    lista_puertos = []
    puertos = serial.tools.list_ports.comports()
    for puerto in puertos:
        lista_puertos.append(str(puerto[0]))
    lista_puertos.append('None')
    lista_puertos.sort()
    return lista_puertos

class Hilo(threading.Thread):  # El thread que ira leyendo del puerto serie
    delay_Puerto = 0.001  # en s

    def __init__(self, nombre, cola, cola_hilo, tk_app):
        threading.Thread.__init__(self)
        self.cola = cola
        self.cola_hilo = cola_hilo
        self.tk_app = tk_app

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
                time.sleep(Hilo.delay_Puerto)  # para dejar tiempo de procesador al hilo principal

        la_cola_hilo.put(INSTR_STOP_THREAD)
        if DEBUG_Consola == 1:
            print("lectura parada")
        lectura = 0  # creo que esto no es necesario?

    def run(self):
        self.leer_puerto(self.cola, self.cola_hilo)
        if DEBUG_Consola == 1:
            print("funcion run terminada")