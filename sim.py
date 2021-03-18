# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE Simulator Core
"""

import math
import time
import logging
from multiprocessing.connection import Client

module_logger = logging.getLogger(__name__)

from global_config import MOTOR_ID_L, MOTOR_ID_R, SENSOR_ID, OUTPUT_FILE_NAME, INITIAL_POS_X, INITIAL_POS_Y, \
    INITIAL_POS_THETA, WORLD__N_BITS, WORLD__MAX_2POW, SOCKET_IP, SOCKET_PORT, SIM_STEP_MS_TIME, MAX_SIM_STEPS

from AX import AX_registers


class Simulator(object):
    DELTA_T = SIM_STEP_MS_TIME / 1000.0  # convertimos el paso de la simul a s
    CNTS_2_MM = 200.0 * DELTA_T / 1023  # conversion del valor de velocidad del AX12 [0..3FF] a mm/s
    L_AXIS = 1.0
    ORIENT_L = 1  # Orientacion motor izquierdo
    ORIENT_R = -1  # Orientacion motor derecho

    def __init__(self, mundo, AX12, AXS1,
                 initial_pos_x=INITIAL_POS_X, initial_pos_y=INITIAL_POS_Y, initial_theta=INITIAL_POS_THETA,
                 socket_ip=SOCKET_IP, socket_port=SOCKET_PORT, start_conn=True,
                 log_data=True,
                 motor_id_l=MOTOR_ID_L, motor_id_r=MOTOR_ID_R, sensor_id=SENSOR_ID,
                 ):

        # threading.Thread.__init__(self)

        # Motors
        self.AX12 = AX12
        # Sensors
        self.AXS1 = AXS1

        # Starting logger
        self.logger = logging.getLogger('pae.sim.Sim')

        # Socket data
        self.socket_ip = socket_ip
        self.socket_port = socket_port
        self.socket_conn = None
        if start_conn:
            self.open_socket()

        # Initial values for position and orientation
        self.initial_pos_x = initial_pos_x
        self.initial_pos_y = initial_pos_y
        self.initial_theta = initial_theta

        # World used for the simulation
        self.world = mundo.datos
        self.max_y = mundo.alto
        self.max_x = mundo.ancho

        self.log = None
        self.log_data = log_data
        if self.log_data:
            self.enable_data_logging()

        # Sensor and motors IDs
        self.motor_id_l = motor_id_l
        self.motor_id_r = motor_id_r
        self.sensor_id = sensor_id

        # Registering later the gui to update sensor bars
        self.gui = None

        # Not required, only declaring in init
        # Position integer
        self.x = self.y = 0
        # Orientation double
        self.theta = 0.0
        self.sim_step = 0
        # Integer velocities, left and right motor
        self.iv_l = self.iv_r = 0
        # Sensor distances
        self.distance_center = self.distance_right = self.distance_left = 0
        # Double velocities, left and right motor
        self.v_l = self.v_r = 0.0
        # Double position,
        self.x_p = self.y_p = 0.0
        # Radius of turn
        self.r = 0.0
        # Angular velocity
        self.w = 0.0
        # Instantaneous center of curvature point, double
        self.icc_x = self.icc_y = 0.0

        # Last time updated simulation values
        self.t_last_upd = time.time()

        self.objective_delay = self.DELTA_T

        self.running = 1

        self.reset_robot()

    def pause(self):
        self.running = 0

    def resume(self):
        self.running = 1

    def enable_data_logging(self):
        self.log = open(OUTPUT_FILE_NAME, 'a')
        self.log_data = True

    def disable_data_logging(self):
        if self.log is not None:
            self.log.close()
            self.log = None
        self.log_data = False

    def open_socket(self):
        self.socket_conn = Client((self.socket_ip, self.socket_port))

    def close_socket(self):
        self.socket_conn.close()

    def close(self):
        self.send_2_plot("close")
        self.close_socket()

    def send_2_plot(self, arg):
        self.socket_conn.send(arg)

    def reset_plot(self):
        self.send_2_plot("reset")
        self.send_2_plot("%.2f, %.2f\n" % (self.initial_pos_x, self.initial_pos_y))

    def reset_robot(self):
        """ Reset the simulation status

        :return:
        """
        self.x = self.x_p = self.initial_pos_x
        self.y = self.y_p = self.initial_pos_y
        self.theta = self.initial_theta

        self.sim_step = 0

        self.AX12[self.motor_id_l].reset()
        self.AX12[self.motor_id_r].reset()
        self.AXS1.reset()

        self.running = 1

        self.update_sensor_data()

    def obstaculo(self, x, y):
        """ Return True if there is a obstacle in (x,y)

        Los datos se han cargado como un array[4096, 128] => [y, x]
        y es la fila [0..4095]
        x es la columna: bit n de 0..31, dentro de uno de los 0..127 bloques de una fila
        En que bloque se encuentra x?

        :return:
        """

        p_offset = x >> WORLD__MAX_2POW  # INT(x/32)
        p_bit = (WORLD__N_BITS - 1) - (x - (p_offset << WORLD__MAX_2POW))

        if self.world[y, p_offset] & (1 << p_bit):
            return True
        return False

    def sensor_distance(self, theta):
        """ Distance to an obstacle.

        255 is the maximum value

        :param theta: Angle component to use for obstacle detection
        :return:
        """
        modulo = 0.0  # modulo del vector de desplazamiento en la direccion de un sensor
        x = 0.0
        y = 0.0  # componentes del vector de desplazamiento en la direccion de un sensor
        indice = 0  # Distancia al obstaculo

        # incrementos del vector de desplazamiento en la direccion de un sensor:
        dx = math.cos(theta)
        dy = math.sin(theta)

        while (modulo < 255) and not (self.obstaculo(round(self.x + x), round(self.y + y))):
            x += dx
            y += dy
            modulo = math.sqrt(x * x + y * y)
            indice += 1

        if modulo > 255:
            u8_mod = 255
        else:
            u8_mod = round(modulo)

        return u8_mod

    def set_gui(self, tk_app):
        self.gui = tk_app

    def distance(self):
        # Parametros: _robot_pos_t *robot_pos, uint8_t *izq, uint8_t *centro, uint8_t *der
        # x0 = 0 #posicion del bloque de sensores
        # y0 = 0 #posicion del bloque de sensores
        # theta = 0.0 #orientacion del sensor central
        # theta_l = 0.0; theta_r = 0.0 #Orientacion de los sensores izquierdo y derecho

        # Angulos en sentido trigonometrico correcto:
        theta_r = self.theta - math.pi / 2
        theta_l = self.theta + math.pi / 2

        # Sensor central:
        self.distance_center = self.sensor_distance(self.theta)

        # Sensor izquierda:
        self.distance_left = self.sensor_distance(theta_l)

        # Sensor derecha
        self.distance_right = self.sensor_distance(theta_r)

    def elapsed_time(self, milliseconds):
        # Parametros: clock_t t1, uint32_t miliseconds, int32_t *true_elapsed_time
        t2 = time.time()
        true_elapsed_time = (t2 - self.t_last_upd) * 1000  # para tenerlo en ms
        if true_elapsed_time > milliseconds:
            return True, true_elapsed_time
        else:
            return False, true_elapsed_time

    def check_colision(self):
        if self.obstaculo(self.x, self.y):
            self.logger.error(
                "***** COLLISION DETECTED AT (%d, %d) simulator step: %d \n" % (self.x, self.y, self.sim_step))
            return True
        return False

    def check_out_of_bounds(self):
        """Verify we are not getting outside of the room

        """
        if self.x > self.max_x - 1 or self.y > self.max_y - 1 or self.x < 0 or self.y < 0:
            self.logger.warning("***** LEAVING ROOM... STOPPING SIMULATOR\n")
            return True
        return False

    def check_simulation_end(self):
        if self.MAX_SIM_STEPS != 0 and self.sim_step >= self.MAX_SIM_STEPS:
            self.logger.info("***** SIMULATION END REACHED. STOPPING SIMULATOR\n")
            return True
        return False

    def end_simulator(self):
        return

    def _speed_dyn_2_speed_int(self, motor_id):
        """ Reads from the dynamixel memory the speed of a endless turning wheel

        :param motor_id: ID of the Dynamixel module
        :return:
        """
        v = self.AX12[motor_id][AX_registers.GOAL_SPEED_L]
        v |= ((self.AX12[motor_id][AX_registers.GOAL_SPEED_H] & 0x03) << 8)
        if self.AX12[motor_id][AX_registers.GOAL_SPEED_H] & 0x04:
            v *= -1
        return v

    def read_speed(self):
        """ Read the speed of the dynamixel modules and update the position

        :return:
        """
        self.iv_l = self._speed_dyn_2_speed_int(self.motor_id_l) * self.ORIENT_L
        self.iv_r = self._speed_dyn_2_speed_int(self.motor_id_r) * self.ORIENT_R

    def calculate_new_position(self):
        """ Update the position and orientation of the robot using two wheel differential drive kinematics

        http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
        :return:
        """

        self.read_speed()
        self.v_l = self.CNTS_2_MM * self.iv_l
        self.v_r = self.CNTS_2_MM * self.iv_r

        if self.iv_l == self.iv_r:
            self.x_p += self.v_l * self.DELTA_T * math.cos(self.theta)
            self.y_p += self.v_r * self.DELTA_T * math.sin(self.theta)
        else:
            self.r = (self.L_AXIS / 2) * (self.v_l + self.v_r) \
                     / (self.v_r - self.v_l)
            self.w = (self.v_r - self.v_l) / self.L_AXIS
            self.icc_x = self.x_p - self.r * math.sin(self.theta)
            self.icc_y = self.y_p + self.r * math.cos(self.theta)
            self.x_p = (math.cos(self.w * self.DELTA_T) * (self.x_p - self.icc_x)
                        - math.sin(self.w * self.DELTA_T) * (self.y_p - self.icc_y)
                        + self.icc_x)
            self.y_p = (math.sin(self.w * self.DELTA_T) * (self.x_p - self.icc_x)
                        + math.cos(self.w * self.DELTA_T) * (self.y_p - self.icc_y)
                        + self.icc_y)

            self.theta += self.w * self.DELTA_T
            if self.theta < -math.pi:
                self.theta += 2 * math.pi
            elif self.theta > math.pi:
                self.theta -= 2 * math.pi
        self.x = round(self.x_p)
        self.y = round(self.y_p)

        # if SIMUL_Save:
        #     fichero_log.write("%.2f, %.2f, %.3f, %.2f, %.2f\n" %(self.x_p, self.y_p,
        #         self.theta, self.v_l, self.v_r))

    def update_sensor_data(self):
        """ Update the sensor data taking into account the new position

        :return:
        """
        self.distance()
        # Actualizamos la memoria de los modulos: (duplicando los sensores de un modulo al otro)
        self.AXS1[AX_registers.IR_LEFT] = self.distance_left  # Left IR
        self.AXS1[AX_registers.IR_CENTER] = self.distance_center  # Center IR
        self.AXS1[AX_registers.IR_RIGHT] = self.distance_right  # Right IR

        if self.gui is not None:
            self.gui.valor_barra_izq.set(self.AXS1[AX_registers.IR_LEFT])
            self.gui.valor_barra_der.set(self.AXS1[AX_registers.IR_RIGHT])
            self.gui.valor_barra_cent.set(self.AXS1[AX_registers.IR_CENTER])

    def update_movement_simulator_values(self):
        """ Update, if required, the position and sensor information

        :return:
        """
        # while True

        elapsed, true_elapsed_time = self.elapsed_time(self.objective_delay)
        if elapsed and self.running:
            if self.sim_step > 1:
                # self.objective_delay = self.DELTA_T - 1e-3 * (true_elapsed_time - SIM_STEP_MS_TIME)
                self.objective_delay = self.DELTA_T
                if true_elapsed_time * 1e-3 > self.DELTA_T * 1.1:
                    # self.logger.error("Required delay is negative (%g), simulation is too slow!" % self.objective_delay)
                    self.logger.error("Simulation too slow, elapsed time was %g" % (true_elapsed_time * 1e-3))
                    self.objective_delay = self.DELTA_T
            self.t_last_upd = time.time()
            self.sim_step += 1
            if MAX_SIM_STEPS != 0 and self.sim_step >= MAX_SIM_STEPS:
                self.logger.warning("***** SIMULATION END REACHED. STOPPING SIMULATOR\n")
                self.running = 0
                return False, 0
            self.calculate_new_position()
            if not self.check_out_of_bounds():
                self.update_sensor_data()
                if self.check_colision():
                    self.running = 0
                    return False, 0
                # Mandamos las nuevas coordenadas al socket de la ventana grafica:
                self.send_2_plot("%.2f, %.2f\n" % (self.x_p, self.y_p))
                # Si esta activada la grabacion, escribimos los datos al fichero de salida:
                if self.log_data:
                    self.log.write("%.2f, %.2f, %.3f, %.2f, %.2f\n" % (self.x_p, self.y_p,
                                                                       self.theta, self.v_l,
                                                                       self.v_r))
        return True, self.objective_delay

    def run(self):
        while True:
            self.update_movement_simulator_values()
            time.sleep(self.objective_delay)


if __name__ == "__main__":
    import subprocess
    from world import World

    subprocess.Popen("python plot_movement.py", stdin=subprocess.PIPE, text=True)
    mundo = World("habitacion_003.h")
    sim = Simulator(mundo)
    sim.update_movement_simulator_values()
    print(sim.x, sim.y, sim.theta)
