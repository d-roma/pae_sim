# -*- coding: utf-8 -*-
"""
Spyder Editor
"""

# !/usr/bin/python3

import serial
import serial.tools.list_ports
import numpy as np
import math



# comprova si hi ha error de checksum a la trama
# def comprova_checksum(frame):
#     len_trama = len(frame)
#     chk_sum = 0
#     for index in range(2, (len_trama - 1)):
#         chk_sum = chk_sum + frame[index]
#     chk_sum = chk_sum & 0xFF
#     if (chk_sum | frame[len_trama - 1]) == 0xFF:
#         # ser.write(b'\x00')
#         if DEBUG_trama:
#             print('Checksum correcte')
#         return 0x00
#     else:
#         print('Error de Checksum')
#         # ser.write(b'\x10')
#         return 0x10


# def send_status_packet(modul_id, error_code):
#     status_frame = Status_NoError[:]
#     status_frame[2] = modul_id
#     status_frame[4] = error_code
#     len_trama = len(status_frame)
#     l_chksum = 0
#     for index in range(2, (len_trama - 1)):
#         l_chksum = l_chksum + status_frame[index]
#     l_chksum = (~l_chksum & 0xFF)
#     status_frame[len_trama - 1] = l_chksum
#     string = ''.join(['0x%02X ' % b for b in status_frame])
#     if DEBUG_trama:
#         print("status packet in hex:", string)
#         print("status packet in dec:", status_frame)
#     ser.write(status_frame)
#     return
#
#
# def generate_read_packet(modul_id, address, num_param):
#     global simulando
#     # simulando = 0 #simulacion en pausa, para poner un breakpoint mas adelante antes de reanudar la simulacion
#     status_frame = Status_NoError[:]
#     status_frame[2] = modul_id
#     status_frame[3] = num_param + 2
#     for index in range(0, num_param):
#         status_frame.insert(index + 5, AX12[modul_id - 1][address + index])
#     len_trama = len(status_frame)
#     l_chksum = 0
#     for index in range(2, (len_trama - 1)):
#         l_chksum = l_chksum + status_frame[index]
#     l_chksum = (~l_chksum & 0xFF)
#     status_frame[len_trama - 1] = l_chksum
#     string = ''.join(['0x%02X ' % b for b in status_frame])
#     if DEBUG_trama:
#         print("status packet in hex:", string)
#         print("status packet in dec:", status_frame)
#     ser.write(status_frame)
#     # simulando = 1 #para poner un breakpoint antes de reanudar la simulacion
#     return
#
#
# def generate_status_packet(id_modul, instruc, code_error, trama):
#     if instruc != 2:
#         send_status_packet(id_modul, code_error)
#     elif instruc == INSTR_READ:
#         address = trama[5]
#         num_param = trama[6]
#         generate_read_packet(id_modul, address, num_param)
#     return
#
#
#
# def leer_puerto(self, la_cola, la_cola_hilo):
#     global instruccio
#     global trama
#     global ID
#     global AX12_moving_L
#     global AX12_moving_R
#     global lectura
#     Lista_acciones = []  # para almacenar acciones recibidas con REG_WRITE
#     AX12_moving_L = "PARAT"
#     AX12_moving_R = "PARAT"
#     print("Lectura puerto iniciada...", instruccio)
#     while instruccio != INSTR_END:
#         if not la_cola.empty():
#             mensaje = la_cola.get()
#             if mensaje == INSTR_END:
#                 instruccio = mensaje
#                 if DEBUG_Consola == 1:
#                     print("instruccio rebuda: ", instruccio)
#             if DEBUG_Consola == 1:
#                 print("mensaje recibido en leer_puerto: ", mensaje)
#         elif lectura == 1:
#             if ser.is_open:
#                 trama = ser.read(16)
#             if trama != b'':
#                 items_array = len(trama)
#                 if DEBUG_Consola == 1:
#                     print("Número de items en el array:", items_array)
#                 if items_array >= 6:  # en cas contrari no es un instruction packet
#                     if DEBUG_Consola == 1:
#                         print("")
#                         print("****************************************************")
#                     instruccio = trama[4]  # posicio de la trama on esta la instruccio
#                     # si esta activat el debug mostra la trama que arriba
#                     if DEBUG_trama:
#                         print_trama()
#                     # mira quina instruccio es i si no es una de les que existeix dona un error
#                     instr_error = print_tipus_Instruccio(instruccio, trama[5])
#                     # copmprova el checksum rebut amb el calculat
#                     chk_sum_error = comprova_checksum(trama[0:trama[3] + 4])
#                     # error indicara si hi ha un error, sigui d'instruccio o de checksum
#                     error = (chk_sum_error | instr_error)
#                     ID = trama[2]  # posicio a la trama del identificador edl modul
#                     if ID != 0xFE:  # si el ID no es el de broadcast respon amb un status packet
#                         # send_status_packet(ID, instruccio, error, trama)
#                         generate_status_packet(ID, instruccio, error, trama)
#                     else:
#                         print("Broadcasting ID Instruction Packet")
#                     if error == 0:  # si no hi ha hagut cap error analitza la instruccio i l'executa
#                         if instruccio == INSTR_WRITE:  # per ara nomes executa la instruccio WRITE
#                             n_parametres = trama[3] - 3
#                             address = trama[
#                                 5]  # posicio de la tama on esta l'adreça de memoria del modul a escriure
#                             Actualitza_AX_Memory(ID, address, n_parametres)
#                             AX12_func(address)  # informa quin comandament s'ha executat
#                             if DEBUG_Moduls:
#                                 print_AX_MemoryMap()
#                             robot_status()
#                             # label_trama.set(str(trama)[3:-1])
#                             texto_trama.set(ESTAT_ROBOT)
#                             texto_motor_left.set(AX12_moving_L)
#                             texto_motor_right.set(AX12_moving_R)
#
#                         elif instruccio == INSTR_REG_WR:
#                             # tenemos que ir almacenando las acciones pendientes:
#                             ID = trama[2]
#                             direccion = trama[5]
#                             num_param = trama[3] - 3
#                             parametros = []
#                             for param in range(num_param):
#                                 parametros.append(trama[6 + param])
#                             Lista_acciones.append([ID, direccion, num_param, parametros])
#                             if DEBUG_Consola == 1:
#                                 print(Lista_acciones)
#                         elif instruccio == INSTR_ACTION:
#                             if not Lista_acciones:
#                                 print("No hay acciones pendientes...")
#                             else:
#                                 # print("Acciones pendientes:")
#                                 for index in range(len(Lista_acciones)):
#                                     if DEBUG_Consola == 1:
#                                         print("Accion pendiente:", Lista_acciones[index])
#                                     ID = Lista_acciones[index][0]
#                                     direccion = Lista_acciones[index][1]
#                                     num_param = Lista_acciones[index][2]
#                                     for param in range(num_param):
#                                         parametro = Lista_acciones[index][3][param]
#                                         # Hay que actualizar la memoria del (los) modulo(s):
#                                         AX12[ID - 1][direccion + param] = parametro
#                                     AX12_func(direccion)  # informa quin comandament s'ha executat
#                                 robot_status()
#                                 texto_trama.set(ESTAT_ROBOT)
#                                 texto_motor_left.set(AX12_moving_L)
#                                 texto_motor_right.set(AX12_moving_R)
#                                 # una vez ejecutadas las acciones pendientes, limpiamos la lista:
#                                 Lista_acciones = []
#                     else:  # Ha habido un error, hay que hacer limpieza:
#                         Lista_acciones = []
#                         ser.flushInput()
#                         # Algo mas??
#             time.sleep(delay_Puerto)  # para dejar tiempo de procesador al hilo principal
#
#     la_cola_hilo.put(INSTR_STOP_THREAD)
#     if DEBUG_Consola == 1:
#         print("lectura parada")
#     lectura = 0  # creo que esto no es necesario?


import time

if __name__ == "__main__":
    port_com = 'COM7'
    baud_rate = 115200

    try:
        ser = serial.Serial(port_com, baud_rate, xonxoff=False, rtscts=False, dsrdtr=False)  # se configura luego
    except:
        print("Error opening serial port")

    Status_NoError = [0xff, 0xff, 0x01, 0x02, 0x00, 0xfc]

    ser.timeout = 0

    ser.reset_input_buffer()

    while True:
        header = ser.read(16)
        if len(header) != 0:
            ser.write(Status_NoError)
            ser.flush()
            print(header)