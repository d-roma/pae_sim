# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE Communication
"""

import serial
import serial.tools.list_ports
import threading
import time

import logging

module_logger = logging.getLogger(__name__)

from global_config import *
from AX import AX_registers, AX12_reset_memory, AX_instruction

BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000]

# crea una trama d'status per indicar que no hi ha error
Status_NoError = [0xff, 0xff, 0x01, 0x02, 0x00, 0xfc]


def lista_puertos_serie():
    lista_puertos = []
    puertos = serial.tools.list_ports.comports()
    for puerto in puertos:
        lista_puertos.append(str(puerto[0]))
    lista_puertos.append('None')
    lista_puertos.sort()
    return lista_puertos


class SerialCom(threading.Thread):  # El thread que ira leyendo del puerto serie
    delay_Puerto = 0.001  # en s

    INSTR_IDLE = 0x00
    INSTR_END = 0xFF
    INSTR_STOP_THREAD = 0xEE
    INSTR_STOP_SIMUL = 0xCC
    INSTR_SIMUL_ENDED = 0xDD
    INSTR_Actualizar_graf = 0xBB

    def __init__(self, tk_app, AX12, AXS1):
        threading.Thread.__init__(self)
        self.tk_app = tk_app
        self.AX12 = AX12
        self.AXS1 = AXS1
        self.serial = serial.Serial()

        self.logger = logging.getLogger('pae.com.SerialCom')

        self.instruccio = self.INSTR_IDLE
        self.AX12_moving_L = "PARAT"
        self.AX12_moving_R = "PARAT"

        self.ID = None
        self.exit = False

    def open(self, com_port, baud_rate):
        if self.serial.is_open:
            self.serial.close()
        self.serial.port = com_port
        self.serial.baudrate = baud_rate
        self.serial.timeout = SERIAL_TIMEOUT
        try:
            self.serial.open()
            self.exit = False
        except serial.SerialException:
            raise IOError("Error opening port")

    def close(self):
        self.exit = True
        self.serial.close()

    def f_angle_limit(self):
        cw_angle = self.AX12[self.ID][AX_registers.CW_ANGLE_LIMIT_L] + (
                self.AX12[self.ID][AX_registers.CW_ANGLE_LIMIT_H] & 0x03) << 8
        ccw_angle = self.AX12[self.ID][AX_registers.CCW_ANGLE_LIMIT_L] + (
                self.AX12[self.ID][AX_registers.CCW_ANGLE_LIMIT_H] & 0x03) << 8
        if self.tk_app.DEBUG_Consola == 1:
            if cw_angle == 0:
                self.logger.info("Motor" + str(self.ID) + "gir continu en sentit horari")
            else:
                self.logger.info("Motor" + str(self.ID) + "angle limit en sentit horari:", cw_angle)
            if ccw_angle == 0:
                self.logger.info("Motor" + str(self.ID) + "gir continu en sentit anti-horari")
            else:
                self.logger.info("Motor" + str(self.ID) + "angle limit en sentit anti-horari:", ccw_angle)
        return

    def f_led(self):
        if self.tk_app.DEBUG_Consola == 1:
            data = self.get_AX_pointer()
            if data[AX_registers.LED] == 1:
                self.logger.info("LED motor " + str(self.ID) + " ON")
            else:
                self.logger.info("LED motor " + str(self.ID) + " OFF")
        return

    def f_moving_speed(self):
        if (self.ID != MOTOR_ID_R) and (self.ID != MOTOR_ID_L):
            return
        velocitat = self.AX12[self.ID][AX_registers.GOAL_SPEED_L] + (
                self.AX12[self.ID][AX_registers.GOAL_SPEED_H] & 0x03) << 8
        if self.tk_app.DEBUG_Consola == 1:
            self.logger.info("Velocitat  motor" + str(self.ID) + "=" + str(velocitat))
        text_motor = "v=" + str(velocitat)
        sentit = self.AX12[self.ID][AX_registers.GOAL_SPEED_H] & 0x04
        if sentit == 0:
            if self.tk_app.DEBUG_Consola == 1:
                self.logger.info("Sentit gir motor" + str(self.ID) + "= CCW")
            if self.ID == 1:
                self.AX12_moving_L = text_motor + " CCW"
            else:
                self.AX12_moving_R = text_motor + " CCW"
        else:
            if self.tk_app.DEBUG_Consola == 1:
                self.logger.info("Sentit gir motor" + str(self.ID) + "= CW")
            if self.ID == 1:
                self.AX12_moving_L = text_motor + " CW"
            else:
                self.AX12_moving_R = text_motor + " CW"
        return

    def f_ADC_value(self):
        data = self.get_AX_pointer()
        ADC_val = data[AX_registers.ADC_VALUE]
        if self.tk_app.DEBUG_Consola == 1:
            self.logger.info("Mesura del ADC" + str(self.ID) + ":" + str(ADC_val))
        return

    def AX12_func(self, argument):
        switcher = {
            # 0x00: "Model Number(L)",
            # 0x01: "Model NUmber(H)",
            # 0x02: "Firmware Version",
            # 0x03:  "ID",
            # 0x04: "Baud Rate",
            # 0x05: "Return Delay Time",
            0x06: self.f_angle_limit,
            0x07: self.f_angle_limit,
            0x08: self.f_angle_limit,
            0x09: self.f_angle_limit,
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
            0x19: self.f_led,
            # 0x1A: f_Left_IR,
            # 0x1B: f_Center_IR,
            # 0x1C: f_Right_IR,
            # 0x1D: "CCW Compliance Slope",
            # 0x1E: "Goal Position(L)",
            # 0x1F: "Goal Position(H)",
            0x20: self.f_moving_speed,
            0x21: self.f_moving_speed,
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
            0x2D: self.f_ADC_value,
            # 0x2E: "Moving",
            # 0x2F: "Lock",
            # 0x30: "Punch(L)",
            # 0x31: "Punch(H)"
        }
        # if argument in switcher:
        #     switcher[argument]()
        # else:
        #     print("Funció no implementada")
        func = switcher.get(argument, lambda: "Funció no implementada")
        func()

    def print_trama(self, trama):
        self.logger.info(" ")
        self.logger.info("####################################################")
        self.logger.info("Received Instruction Frame:" + str(trama))
        for index in range(len(trama)):
            self.logger.info("Received Instruction Frame[" + str(index) + "]:" + '0x%02X ' % trama[index])
        self.logger.info("####################################################")

    # comprova si hi ha error d'instruccio la trama
    def comprova_instr(self, instruccio):
        if (instruccio < 0x07) or (instruccio == 0xFF) or (instruccio == 0x83):
            if self.tk_app.DEBUG_Consola == 1:
                self.logger.info("Instrucció:" + str(AX_instruction(instruccio).name))
            return 0x00
        else:
            self.logger.info("Error d'instruccio")
            return 0x70

    def print_tipus_Instruccio(self, instruccio, comandament):
        error_de_instr = self.comprova_instr(instruccio)
        if self.tk_app.DEBUG_Consola == 1:
            if error_de_instr == 0:
                if instruccio == AX_instruction.ACTION.value:
                    comandament = 0x2C  # "Registered Instruction"
                self.logger.info("----------- INSTRUCCIÓ i COMANDAMENT --------------")
                self.logger.info("Command:" + str(AX12_reset_memory[AX_registers(comandament)][1]))
        return error_de_instr

    # comprova si hi ha error de checksum a la trama
    def comprova_checksum(self, frame, debug):
        len_trama = len(frame)
        chk_sum = 0
        for index in range(2, (len_trama - 1)):
            chk_sum = chk_sum + frame[index]
        chk_sum = chk_sum & 0xFF
        if (chk_sum | frame[len_trama - 1]) == 0xFF:
            # ser.write(b'\x00')
            if debug:
                self.logger.info('Checksum correcte')
            return 0x00
        else:
            self.logger.warning('Error de Checksum')
            # ser.write(b'\x10')
            return 0x10

    def send_status_packet(self, modul_id, error_code):
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
        if self.tk_app.DEBUG_trama:
            self.logger.info("status packet in hex:" + str(string))
            self.logger.info("status packet in dec:" + str(status_frame))
        self.serial.write(status_frame)

    def generate_read_packet(self, modul_id, address, num_param):
        status_frame = Status_NoError[:]
        status_frame[2] = modul_id
        status_frame[3] = num_param + 2
        data = self.get_AX_pointer(modul_id)
        for index in range(0, num_param):
            status_frame.insert(index + 5, data[AX_registers(address + index)])
        len_trama = len(status_frame)
        l_chksum = 0
        for index in range(2, (len_trama - 1)):
            l_chksum = l_chksum + status_frame[index]
        l_chksum = (~l_chksum & 0xFF)
        status_frame[len_trama - 1] = l_chksum
        string = ''.join(['0x%02X ' % b for b in status_frame])
        if self.tk_app.DEBUG_trama:
            self.logger.info("status packet in hex:" + str(string))
            self.logger.info("status packet in dec:" + str(status_frame))
        self.serial.write(status_frame)

    def generate_status_packet(self, instruc, code_error, trama):
        if instruc != 2:
            self.send_status_packet(self.ID, code_error)
        elif instruc == AX_instruction.READ.value:
            address = trama[5]
            num_param = trama[6]
            self.generate_read_packet(self.ID, address, num_param)

    def get_AX_pointer(self, modul_id=None):
        if modul_id is None:
            modul_id = self.ID
        if modul_id == SENSOR_ID:
            data = self.AXS1
        else:
            data = self.AX12[modul_id]
        return data

    def actualitza_AX_Memory(self, adressa, nparametres, trama):
        data = self.get_AX_pointer()
        for index in range(nparametres):
            data[AX_registers((adressa + index))] = trama[index + 6]
        return

    def print_AX_MemoryMap(self):
        # TODO Move to sim?
        self.logger.info("---------------------------------------------------")
        self.logger.info("========== AX ========= [" + str(MOTOR_ID_L) + "] ====== [" + str(MOTOR_ID_R)
                         + "] ==== [" + str(SENSOR_ID) + "]")
        for key in AX_registers:
            mot1 = ''.join(['0x%02X ' % (self.AX12[MOTOR_ID_L][key])])  # format Hexadecimal
            mot2 = ''.join(['0x%02X ' % (self.AX12[MOTOR_ID_R][key])])  # format Hexadecimal
            mot3 = ''.join(['0x%02X ' % (self.AXS1[key])])  # format Hexadecimal
            self.logger.info('{:-<23}'.format(AX12_reset_memory[key][1]) + ">" + str(mot1) + "     "
                             + str(mot2) + "     " + str(mot3))
        self.logger.info("####################################################")
        return

    # dona l'estat del robot
    def robot_status(self):
        CCW = 0x00
        CW = 0x04

        if self.tk_app.DEBUG_Consola == 1:
            self.logger.info("----------- ESTAT DEL ROBOT -----------------------")
        v_left = self.AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_L] + (
                self.AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_H] & 0x03) << 8
        sentit_left = self.AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_H] & 0x04
        v_right = self.AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_L] + (
                self.AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_H] & 0x03) << 8
        sentit_right = self.AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_H] & 0x04
        if (v_left == 0) & (v_right == 0):
            if self.tk_app.DEBUG_Consola == 1:
                self.logger.info("Robot Parat")
            estat_robot = "Robot Parat"
        elif sentit_left == sentit_right:  # si motors giren mateix sentit => robot gira
            if sentit_left == CW:
                if self.tk_app.DEBUG_Consola == 1:
                    self.logger.info("Robot Gira Esquerra")
                estat_robot = "Robot Gira Esquerra"
            else:
                if self.tk_app.DEBUG_Consola == 1:
                    self.logger.info("Robot Gira Dreta")
                estat_robot = "Robot Gira Dreta"
        elif abs(v_left - v_right) < 1:  # si motors giren sentit contrari a mateixa velocitat=> robot va recte
            if sentit_left == CW:
                if self.tk_app.DEBUG_Consola == 1:
                    self.logger.info("Robot Marxa Enrere")
                estat_robot = "Robot Marxa Enrere"
            else:
                if self.tk_app.DEBUG_Consola == 1:
                    self.logger.info("Robot Marxa Endavant")
                estat_robot = "Robot Marxa Endavant"
        elif v_left > v_right:  # velocitats diferents, motor esquerre mes rapid
            if sentit_left == CW:
                if self.tk_app.DEBUG_Consola == 1:
                    self.logger.info("Robot Gira Esquerra")
                estat_robot = "Robot Gira Esquerra"
            else:
                if self.tk_app.DEBUG_Consola == 1:
                    self.logger.info("Robot Gira Dreta")
                estat_robot = "Robot Gira Dreta"
        elif sentit_left == CW:  # velocitats diferents, motor dret mes rapid
            if self.tk_app.DEBUG_Consola == 1:
                self.logger.info("Robot Gira Dreta")
            estat_robot = "Robot Gira Dreta"
        else:
            if self.tk_app.DEBUG_Consola == 1:
                self.logger.info("Robot Gira Esquerra")
            estat_robot = "Robot Gira Esquerra"
        return estat_robot

    def leer_puerto(self):
        Lista_acciones = []  # para almacenar acciones recibidas con REG_WRITE
        AX12_moving_L = "PARAT"
        AX12_moving_R = "PARAT"
        self.logger.warning("Lectura puerto iniciada..." + str(self.instruccio))
        while True:
            if self.exit:
                break
            if self.serial.is_open and self.serial.in_waiting:
                inicio_trama = self.serial.read(4)
                if inicio_trama != b'':
                    if inicio_trama[0] == 0xFF and inicio_trama[1] == 0xFF:  # inicio de trama valido
                        resto_trama = self.serial.read(inicio_trama[3])
                        trama = b''.join([inicio_trama, resto_trama])
                    else:
                        self.logger.error("ERROR, received packet with wrong start of frame bytes")
                        continue
                    items_array = len(trama)
                    if self.tk_app.DEBUG_Consola == 1:
                        self.logger.debug("Número de items en el array:" + str(items_array))
                    if items_array >= 6:  # en cas contrari no es un instruction packet
                        if self.tk_app.DEBUG_Consola == 1:
                            self.logger.debug("")
                            self.logger.debug("****************************************************")
                        instruccio = trama[4]  # posicio de la trama on esta la instruccio
                        # si esta activat el debug mostra la trama que arriba
                        if self.tk_app.DEBUG_trama:
                            self.print_trama(trama)
                        # mira quina instruccio es i si no es una de les que existeix dona un error
                        instr_error = self.print_tipus_Instruccio(instruccio, trama[5])
                        # copmprova el checksum rebut amb el calculat
                        chk_sum_error = self.comprova_checksum(trama[0:trama[3] + 4], self.tk_app.DEBUG_trama)
                        # error indicara si hi ha un error, sigui d'instruccio o de checksum
                        error = (chk_sum_error | instr_error)
                        self.ID = trama[2]  # posicio a la trama del identificador edl modul
                        if self.ID != 0xFE:  # si el ID no es el de broadcast respon amb un status packet
                            # send_status_packet(ID, instruccio, error, trama)
                            self.generate_status_packet(instruccio, error, trama)
                        else:
                            self.logger.debug("Broadcasting ID Instruction Packet")
                        if error == 0:  # si no hi ha hagut cap error analitza la instruccio i l'executa
                            if instruccio == AX_instruction.WRITE.value:  # per ara nomes executa la instruccio WRITE
                                n_parametres = trama[3] - 3
                                # posicio de la tama on esta l'adreça de memoria del modul a escriure
                                address = trama[5]
                                self.actualitza_AX_Memory(address, n_parametres, trama)
                                self.AX12_func(address)  # informa quin comandament s'ha executat
                                if self.tk_app.DEBUG_Moduls:
                                    self.print_AX_MemoryMap()
                                estat_robot = self.robot_status()
                                self.tk_app.set_estat_robot(estat_robot, self.AX12_moving_L, self.AX12_moving_R)

                            elif instruccio == AX_instruction.REG_WRITE.value:
                                # tenemos que ir almacenando las acciones pendientes:
                                self.ID = trama[2]
                                direccion = trama[5]
                                num_param = trama[3] - 3
                                parametros = []
                                for param in range(num_param):
                                    parametros.append(trama[6 + param])
                                Lista_acciones.append([self.ID, direccion, num_param, parametros])
                                if self.tk_app.DEBUG_Consola == 1:
                                    self.logger.debug(str(Lista_acciones))
                            elif instruccio == AX_instruction.ACTION.value:
                                if not Lista_acciones:
                                    self.logger.debug("No hay acciones pendientes...")
                                else:
                                    # print("Acciones pendientes:")
                                    for index in range(len(Lista_acciones)):
                                        if self.tk_app.DEBUG_Consola == 1:
                                            self.logger.debug("Accion pendiente:" + str(Lista_acciones[index]))
                                        self.ID = Lista_acciones[index][0]
                                        direccion = Lista_acciones[index][1]
                                        num_param = Lista_acciones[index][2]
                                        data = self.get_AX_pointer()
                                        for param in range(num_param):
                                            parametro = Lista_acciones[index][3][param]
                                            # Hay que actualizar la memoria del (los) modulo(s):
                                            data[AX_registers(direccion + param)] = parametro
                                        self.AX12_func(direccion)  # informa quin comandament s'ha executat
                                    estat_robot = self.robot_status()
                                    self.tk_app.set_estat_robot(estat_robot, self.AX12_moving_L, self.AX12_moving_R)

                                    # una vez ejecutadas las acciones pendientes, limpiamos la lista:
                                    Lista_acciones = []
                        else:  # Ha habido un error, hay que hacer limpieza:
                            Lista_acciones = []
                            self.serial.flushInput()
                            # Algo mas??
            time.sleep(self.delay_Puerto)  # para dejar tiempo de procesador al hilo principal

        if self.tk_app.DEBUG_Consola == 1:
            self.logger.warning("lectura parada")
        lectura = 0  # creo que esto no es necesario?

    def run(self):
        self.leer_puerto()
        if self.tk_app.DEBUG_Consola == 1:
            self.logger.debug("funcion run terminada")
