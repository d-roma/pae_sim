# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE GUI
"""

import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter import messagebox

import serial
import queue
import threading
import time

from com import lista_puertos_serie, BAUD_RATES, Hilo
from global_config import DEFAULT_COM_PORT, DEFAULT_BAUD_RATE, SERIAL_TIMEOUT, MOTOR_ID_L, MOTOR_ID_R
from AX import AX_registers

# V_inicial_demo_L = 0x3FF
# V_inicial_demo_R = 0x7FF
V_inicial_demo_L = 0
V_inicial_demo_R = 0

INSTR_IDLE = 0x00
INSTR_END = 0xFF
INSTR_STOP_THREAD = 0xEE
INSTR_STOP_SIMUL = 0xCC
INSTR_SIMUL_ENDED = 0xDD
INSTR_Actualizar_graf = 0xBB


class TkApplication(tk.Frame):
    def __init__(self, simulador, master=None):
        super().__init__(master)

        #Custom objects
        self.simulador = simulador
        self.contador = 0

        self.lista_puertos = lista_puertos_serie()

        #Tk objects
        self.DEBUG_trama_check = IntVar()
        self.DEBUG_Moduls_check = IntVar()
        self.DEBUG_Consola_check = IntVar()
        self.SIMUL_check = IntVar()
        self.SIMUL_Save_check = IntVar()
        self.DEBUG_trama_check.set(0)
        self.DEBUG_Moduls_check.set(0)
        self.DEBUG_Consola_check.set(0)
        self.SIMUL_Save_check.set(0)
        self.DEBUG_trama = self.DEBUG_trama_check.get()
        self.DEBUG_Moduls = self.DEBUG_Moduls_check.get()
        self.DEBUG_Consola = self.DEBUG_Consola_check.get()
        self.SIMUL_On_Off = self.SIMUL_check.get()
        self.SIMUL_Save = self.SIMUL_Save_check.get()
        self.texto_trama = StringVar()
        self.texto_motor_left = StringVar()
        self.texto_motor_right = StringVar()
        self.valor_barra_izq = IntVar()
        self.valor_barra_der = IntVar()
        self.valor_barra_cent = IntVar()
        self.Led_motor_left = IntVar()
        self.Led_motor_right = IntVar()
        self.estados = {0: "OFF", 1: "ON "}


        self.master = master
        self.grid()
        self.create_widgets()
        self.crear_hilo()
        self.crear_simul()
        self.check_queue()

        # TODO
        self.instruccio = INSTR_IDLE
        self.simulacio = INSTR_IDLE
        self.simulando = 0
        #self.actualizar_graf = 0

    def refrescar_puertos(cb_lista):
        global port_com
        print("Actualizando lista puertos")
        lista_puertos = lista_puertos_serie()
        cb_lista.config(values=lista_puertos)
        if DEFAULT_COM_PORT in lista_puertos:
            idx = lista_puertos.index(DEFAULT_COM_PORT)
            cb_lista.current(idx)

    def create_widgets(self):
        self.spacer_up = Label(self, width=5, height=1)
        self.spacer_up.grid(row=0, column=0)
        self.spacer_center = Label(self, width=5, height=1)
        self.spacer_center.grid(row=20, column=0)
        self.spacer_bottom = Label(self, width=5, height=1)
        self.spacer_bottom.grid(row=40, column=10)

        self.Label_Debug = Label(self, text="Debug:")
        self.Label_Debug.grid(row=1, column=1, sticky=E)

        self.Debug_frame = Checkbutton(self, variable=self.DEBUG_trama_check)
        self.Debug_frame["text"] = "Debug Trames"
        self.Debug_frame["fg"] = "blue"
        self.Debug_frame["command"] = self.set_debug_frames
        self.Debug_frame.grid(row=1, column=2, sticky=W)

        self.Debug_module = Checkbutton(self, variable=self.DEBUG_Moduls_check)
        self.Debug_module["text"] = "Debug Moduls"
        self.Debug_module["fg"] = "blue"
        self.Debug_module["command"] = self.set_debug_moduls
        self.Debug_module.grid(row=1, column=3, sticky=W)

        self.Debug_consola = Checkbutton(self, variable=self.DEBUG_Consola_check)
        self.Debug_consola["text"] = "Consola On/off"
        self.Debug_consola["fg"] = "blue"
        self.Debug_consola["command"] = self.set_debug_consola
        self.Debug_consola.grid(row=1, column=4, sticky=E)

        self.Label_Simul = Label(self, text="Simulació:")
        self.Label_Simul.grid(row=2, column=1)

        self.Simul_On_Off = Checkbutton(self, variable=self.SIMUL_check)
        self.Simul_On_Off["text"] = "Simul On/off"
        self.Simul_On_Off["fg"] = "blue"
        self.Simul_On_Off["command"] = self.set_Simul_On_Off
        self.Simul_On_Off.grid(row=2, column=2, sticky=W)

        self.Simul_grabar = Checkbutton(self, fg="blue", variable=self.SIMUL_Save_check)
        self.Simul_grabar["text"] = "Desar"
        self.Simul_grabar["command"] = self.grabar_Simul_OnOff
        self.Simul_grabar.grid(row=2, column=3, sticky=W)

        self.Simul_reset = Button(self, text="Reset", fg="blue", command=self.reset_simul)
        self.Simul_reset.grid(row=2, column=4, sticky=W)

        self.label_robot = Label(self, text="ESTAT ROBOT:")
        self.label_robot.grid(row=3, column=1)
        self.label_trama = Label(self, textvariable=self.texto_trama)
        self.texto_trama.set("ROBOT PARAT")
        self.label_trama.grid(row=3, column=2, columnspan=2, sticky=W)

        self.label_AX12_L = Label(self, text="MOTOR Esq.:")
        self.label_AX12_L.grid(row=4, column=1)
        self.label_motor_left = Label(self, textvariable=self.texto_motor_left)
        self.texto_motor_left.set("PARAT")
        self.label_motor_left.grid(row=4, column=2)
        self.label_AX12_R = Label(self, text="MOTOR Dret:")
        self.label_AX12_R.grid(row=5, column=1)
        self.label_motor_right = Label(self, textvariable=self.texto_motor_right)
        self.texto_motor_right.set("PARAT")
        self.label_motor_right.grid(row=5, column=2)

        self.led_motor_left = Radiobutton(self, text="Led Esq.", value=1,
                                          variable=self.Led_motor_left, state=DISABLED)
        self.led_motor_left.grid(row=4, column=3)
        self.led_motor_right = Radiobutton(self, text="Led Dret", value=1,
                                           variable=self.Led_motor_right, state=DISABLED)
        self.led_motor_right.grid(row=5, column=3)

        self.label_izq = Label(self, text="IR Esq.")
        self.label_izq.grid(row=6, column=1)
        self.progress_bar_izq = ttk.Progressbar(self, orient="horizontal",
                                                length=255, maximum=255,
                                                mode="determinate", variable=self.valor_barra_izq)
        self.progress_bar_izq["value"] = 0
        self.progress_bar_izq.place(relx=0.5, rely=0.5, relwidth=0.80, anchor=tk.CENTER)
        self.progress_bar_izq.grid(row=6, column=2, columnspan=3)

        self.label_der = Label(self, text="IR dret")
        self.label_der.grid(row=7, column=1)
        self.progress_bar_der = ttk.Progressbar(self, orient="horizontal",
                                                length=255, maximum=255,
                                                mode="determinate", variable=self.valor_barra_der)
        self.progress_bar_der["value"] = 200
        self.progress_bar_der.place(relx=0.5, rely=0.5, relwidth=0.80, anchor=tk.CENTER)
        self.progress_bar_der.grid(row=7, column=2, columnspan=3)

        self.label_der = Label(self, text="IR Centre")
        self.label_der.grid(row=8, column=1)
        self.progress_bar_der = ttk.Progressbar(self, orient="horizontal",
                                                length=255, maximum=255,
                                                mode="determinate", variable=self.valor_barra_cent)
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

        self.cb = ttk.Combobox(self, values=self.lista_puertos, width=10)
        self.cb.current(0)
        self.cb.grid(row=21, column=1)
        self.cb.bind('<<ComboboxSelected>>', self.on_select)
        if DEFAULT_COM_PORT in self.lista_puertos:
            idx = self.lista_puertos.index(DEFAULT_COM_PORT)
            self.cb.current(idx)

        self.cb_rate = ttk.Combobox(self, state="readonly", width=12,
                                    values=BAUD_RATES)  # desplegable con velocidades de bps estandares
        self.cb_rate.current(BAUD_RATES.index(DEFAULT_BAUD_RATE))
        self.cb_rate.grid(row=21, column=2)
        self.cb_rate.bind('<<ComboboxSelected>>', self.on_select_rate)
        baud_rate = self.cb_rate.get()
        print("Velocidad: ", baud_rate)

        self.refrescar = tk.Button(self, command=lambda: self.refrescar_puertos(self.cb))
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
            ser.timeout = SERIAL_TIMEOUT
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
        # revisar la cola para evitar bloque en la interfaz
        if not self.cola_hilo.empty():
            # obtener mensaje de la cola
            self.instruccio = self.cola_hilo.get()
            if self.DEBUG_Consola == 1:
                print("get text from read queue:", self.instruccio)
            # TODO: En la versio original no estava anidat, per que?
            if self.instruccio == INSTR_STOP_THREAD:
                self.simulador.stop()  # parar la simulacion
                simulacio = INSTR_STOP_SIMUL  # terminar el hilo de la simulacion
                return INSTR_STOP_THREAD
        self.master.after(100, self.check_queue)

    def say_hi(self):
        print("J. Bosch & C. Serre,")
        print("UB, 2020-2021.")
        messagebox.showinfo("Autors", "J. Bosch & C. Serre\nUB, 2020-2021.")

    def set_debug_frames(self):
        self.DEBUG_trama = self.DEBUG_trama_check.get()

    def set_debug_moduls(self):
        self.DEBUG_Moduls = self.DEBUG_Moduls_check.get()

    def set_debug_consola(self):
        self.DEBUG_Consola = self.DEBUG_Consola_check.get()

    def set_Simul_On_Off(self):
        self.SIMUL_On_Off = self.SIMUL_check.get()
        # TODO que hace simulando
        self.simulando = self.SIMUL_On_Off
        print("Simulacion ", self.estados[self.SIMUL_On_Off])

    def grabar_Simul_OnOff(self):
        if self.SIMUL_Save_check.get():
            self.simulador.enable_data_logging()
            self.SIMUL_Save = True
        else:
            self.simulador.disable_data_logging()
            self.SIMUL_Save = False

    def crear_hilo(self):
        # crear cola para comunicar/enviar tareas al thread
        self.cola = queue.Queue()
        self.cola_hilo = queue.Queue()
        if self.DEBUG_Consola == 1:
            print(self.cola, self.cola_hilo)
        # crear el thread
        hilo = Hilo("puerto", self.cola, self.cola_hilo, self)
        # iniciar thread
        hilo.start()

    def crear_simul(self):
        # crear cola para comunicar/enviar tareas al thread
        self.cola_simul = queue.Queue()
        if self.DEBUG_Consola == 1:
            print(self.cola, self.cola_simul)
        # crear el thread
        simul = Simul("Simul", self.cola, self.cola_simul, self, self.simulador)
        self.simulador.update_sensor_data()
        print("Robot @:", self.simulador.x, self.simulador.y)
        print("Distancia: Izq.=", self.simulador.AXS1[AX_registers.IR_LEFT], ", Centro =",
              self.simulador.AXS1[AX_registers.IR_CENTER], ", Der. =",
              self.simulador.AXS1[AX_registers.IR_RIGHT])
        # Actualizamos las barras graficas:
        self.valor_barra_izq.set(self.simulador.AXS1[AX_registers.IR_LEFT])
        self.valor_barra_der.set(self.simulador.AXS1[AX_registers.IR_RIGHT])
        self.valor_barra_cent.set(self.simulador.AXS1[AX_registers.IR_CENTER])

        # iniciar thread
        simul.start()

    def reset_simul(self):
        print("Simulacio reiniciada")
        self.SIMUL_check.set(0)
        self.set_Simul_On_Off()
        # reiniciar el robot_pos
        # reiniciar el contador de simulacion
        self.contador = 0
        # Actualizamos las barras graficas:
        self.valor_barra_izq.set(self.simulador.AX12[0][AX_registers.IR_LEFT])
        self.valor_barra_der.set(self.simulador.AX12[0][AX_registers.IR_RIGHT])
        self.valor_barra_cent.set(self.simulador.AX12[0][AX_registers.IR_CENTER])
        self.simulador.reset_plot()

    def salir(self):
        self.cola.put(INSTR_END)  # terminar funcion leer_puerto
        if (self.instruccio == INSTR_STOP_THREAD) & (simulacio == INSTR_SIMUL_ENDED):
            self.simulador.close()
             # Cerramos la ventana del plot
            if self.DEBUG_Consola == 1:
                print("Hilos finalizados")
            self.master.destroy()  # Termina la aplicacion
        else:
            self.master.after(200, self.salir)


class Simul(threading.Thread):
    delay_Simul = 0.090  # en s

    def __init__(self, nombre, cola, cola_simul, tkapp, simulador):
        threading.Thread.__init__(self)
        self.cola = cola
        self.cola_simul = cola_simul
        self.tkapp = tkapp
        self.simulador = simulador

    def movement(self,):
        # TODO
        global simulacio
        print("Hilo simulacion movimiento iniciado, simulación en pausa...")
        # Pondremos una velocidad inicial, para las pruebas y/o la demo:
        self.simulador.AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_L] = V_inicial_demo_L & 0xFF
        self.simulador.AX12[MOTOR_ID_L][AX_registers.GOAL_SPEED_H] = (V_inicial_demo_L >> 8) & 0x07
        self.simulador.AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_L] = V_inicial_demo_R & 0xFF
        self.simulador.AX12[MOTOR_ID_R][AX_registers.GOAL_SPEED_H] = (V_inicial_demo_R >> 8) & 0x07
        while simulacio != INSTR_STOP_SIMUL:
            if self.simulador.running == 1:
                if self.simulador.update_movement_simulator_values():
                    # Actualizamos las barras graficas:
                    self.tkapp.valor_barra_izq.set(self.simulador.AXS1[AX_registers.IR_LEFT])
                    self.tkapp.valor_barra_der.set(self.simulador.AXS1[AX_registers.IR_RIGHT])
                    self.tkapp.valor_barra_cent.set(self.simulador.AXS1[AX_registers.IR_CENTER])
                # else:
                # en principio, no hacer nada?...
            time.sleep(Simul.delay_Simul)  # para dejar tiempo de procesador al hilo principal
        self.cola_simul.put(INSTR_STOP_SIMUL)
        if self.tkapp.DEBUG_Consola == 1:
            print("simulacion parada")
        self.simulador.stop()
        simulacio = INSTR_SIMUL_ENDED

    def run(self):
        self.movement()
        if self.tkapp.DEBUG_Consola == 1:
            print("funcion run terminada")