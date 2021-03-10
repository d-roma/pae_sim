from tkinter import *
from tkinter.ttk import *
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib import pylab as plt
import matplotlib.pyplot as pplt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.lines import Line2D
import time
import os.path
from multiprocessing.connection import Listener

seguir = 1
total_puntos = 0
fichero_habitacion = "habitacion_003.h"
fichero_datos = "movement.log"
fichero_lock = "log.lock"

punto_anterior_x = 0
punto_anterior_y = 0
puntos_x = []
puntos_y = []

def plot_habitacion(datos_habitacion):
    global ultimo_punto
    # Leemos los datos de la habitacion:
    print("Datos habitacion:", datos_habitacion)
    parametros = np.genfromtxt(datos_habitacion, dtype="int",
                                delimiter=',', skip_header=2,
                                max_rows=1, deletechars="\n")
    ancho, alto = parametros[0:2]
    print("Ancho = ", ancho, " Alto = ", alto)

    parametros = np.genfromtxt(datos_habitacion, delimiter=',', dtype="int",
                                   skip_header=4, max_rows=1, deletechars="\n")
    num_obstaculos = parametros
    print("Hay", num_obstaculos, "obstaculos.")

    data = np.genfromtxt(datos_habitacion,  dtype="int",
                         delimiter=',', skip_header=6,
                         max_rows=num_obstaculos)

    x0s = data[0:, 0]  # coordenada x de la esquina izq. inferior de cada obstaculo
    y0s = data[0:, 1]  # coordenada y de la esquina izq. inferior de cada obstaculo
    anchos = data[0:, 2]  # anchura de cada obstaculo
    altos = data[0:, 3]  # altura de cada obstaculo
    print("Obstaculos:")
    print("n: \tx \ty \tancho \talto")
    for i in range(0, num_obstaculos-1):
        print(i, ":", "\t", x0s[i], "\t", y0s[i], "\t", anchos[i], "\t", altos[i])

    # Creando la figura:
    fig, ax = plt.subplots(figsize=[9, 6])

    # Creando la habitacion:
    left, bottom, width, height = (0, 0, ancho, alto)
    for i in range(0, num_obstaculos):
        obstaculo = pplt.Rectangle((x0s[i], y0s[i]),
                                    anchos[i], altos[i],
                                    facecolor="black", alpha=0.1)
        ax.add_patch(obstaculo)
         
    habitacion = pplt.Rectangle((left, bottom), width, height, facecolor="green", alpha=0.1)
    ax.add_patch(habitacion)

    return fig, ax


def replot_movement(los_ejes):
    global total_puntos, puntos_x, puntos_y

    # Primero, comprobamos la existencia y adecuacion del fichero de datos
    if not os.path.isfile(fichero_datos):
        print("El fichero de datos", fichero_datos, " no existe??")
        return
    if os.path.getsize(fichero_datos) == 0:
        print("El fichero de datos", fichero_datos, " parece vacio??")
        return

    # Leemos los datos de la simulacion:
    try:
        data = np.genfromtxt(fichero_datos, comments="#", delimiter=',')
    except IOError as e:
        print("El fichero esta siendo utilizado", e)
        return
    try:
        if hasattr(data, "__len__"):
            if hasattr(data[0], "__len__"):
                x = data[:, 0]
                y = data[:, 1]
                w = data[:, 2]
                total_puntos = len(x)
            else:  # Justo al inicio, puede que solo tenga un punto:
                x = data[0]
                y = data[1]
                w = data[2]
                total_puntos = 1
    except IndexError as e:  # hemos dado con un fichero vacio
        total_puntos = 0
        print("Fichero de datos vacio", e)
        return

    # Los vectores de la velocidad:
    # q = los_ejes.quiver(x, y, np.cos(w), np.sin(w), color='red', scale=30)
    # Punto origen y punto final:
    xInit_point = x[0]
    yInit_point = y[0]
    xEnd_point = x[-1]
    yEnd_point = y[-1]

    # Hasta aqui, todo correcto
    # Empezamos por borrar la grafica:
    print(len(los_ejes.lines), "lineas para borrar")
    for i in range(len(los_ejes.lines)):
        los_ejes.lines.remove(los_ejes.lines[0])
    print(i+1, "lineas borradas")

    # Ahora a pintar todo:
    los_ejes.plot(x, y, 'co--')
    los_ejes.plot(xInit_point, yInit_point, 'gs')
    los_ejes.plot(xEnd_point, yEnd_point, 'k*')
    texto_trama.set(total_puntos)
    canvas.draw()
    print(len(los_ejes.lines), "lineas redibujadas")

def actualizar_plot(mensaje, los_ejes):
    global total_puntos, punto_anterior_x, punto_anterior_y
    data = mensaje.split(',')
    puntos_x.append(float(data[0]))
    puntos_y.append(float(data[1]))
    los_ejes.lines.remove(los_ejes.lines[0])
    los_ejes.plot(puntos_x, puntos_y, 'co-', markersize=3)
    total_puntos += 1
    # texto_trama.set(total_puntos)
    texto_trama.set(len(puntos_x))


def reiniciar_plot(los_ejes):
    global total_puntos, puntos_x, puntos_y
    # borrar la grafica:
    for i in range(len(los_ejes.lines)):
        los_ejes.lines.remove(los_ejes.lines[0])
    total_puntos = 0
    texto_trama.set(total_puntos)
    puntos_x = []
    puntos_y = []
    ejes.plot(0, 0, 'k+--')  # plot algo para obligar a actualizar


root = Tk()
texto_trama = StringVar()


def salir():
    global seguir
    seguir = 0


def fcn_boton_replot():
    replot_movement(ejes)

figure, ejes = plot_habitacion(fichero_habitacion)
canvas = FigureCanvasTkAgg(figure, root)
canvas.get_tk_widget().grid(row=0, column=0, columnspan=4)
ejes.plot(0, 0, 'k+--')  # plot algo para obligar a actualizar
plt.ion()  # Modo "interactivo", para que se actualice automaticamente
           # sin tener que llamar a plt.draw() o canvas.draw(), que ralentizan mucho...

toolbarFrame = Frame(master=root)
toolbarFrame.grid(row=1, column=0)
toolbar = NavigationToolbar2Tk(canvas, toolbarFrame)

label_puntos = Label(root, text="Puntos: ")
label_puntos.grid(row=1, column=1, sticky=W)
label_trama = Label(root, textvariable=texto_trama)
texto_trama.set("mensajes robot...")
label_trama.grid(row=1, column=1, sticky=E)

boton_replot = Button(master=root, text="Replot", command=fcn_boton_replot)
boton_replot.grid(row=1, column=2)

inc = 0
lectura = 1
# print(sys.stdin.readline())  # Prueba de comunicacion con el pipe

# Intentamos abrir el socket para recibir datos de la aplicacion madre
address = ('localhost', 6000)     # family is deduced to be 'AF_INET'
listener = Listener(address, authkey=b'secret password')
conn = listener.accept()
print('connection accepted from', listener.last_accepted)
print(conn.recv())

# Bucle principal
while seguir:
    # time.sleep(0.005)
    if conn.poll(0.001):
        msg = conn.recv()
        if msg == 'close':
            print("terminando aplicacion grafica")
            conn.close()
            seguir = 0
        elif msg == 'reset':
            reiniciar_plot(ejes)
        else:
            actualizar_plot(msg, ejes)
    root.update_idletasks()
    root.update()

# Terminando la aplicacion
listener.close()
root.destroy()
print("Aplicacion grafica terminada.")
