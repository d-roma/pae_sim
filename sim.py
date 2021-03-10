# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE Simulator Core
"""

import math
import time
import logging

from global_config import MOTOR_ID_L, MOTOR_ID_R, SENSOR_ID, OUTPUT_FILE_NAME

module_logger = logging.getLogger('PAE.sim')

from AX import AX, AX_registers


class Simulator(object):
    WORLD__N_BYTES = 4  # 4 bytes
    WORLD__N_BITS = 32  # 32 bits
    WORLD__MAX_2POW = 5  # math.log2(WORLD__N_BYTES * 8)

    # Las unidades de distancia se consideran en mm
    SIM_STEP_MS_TIME = 50  # en ms
    MAX_SIM_STEPS = 24000
    delay_Simul = 0.090  # en s
    delay_Puerto = 0.001  # en s
    DELTA_T = SIM_STEP_MS_TIME / 1000.0  # convertimos el paso de la simul a s
    CNTS_2_MM = 1000.0 * DELTA_T / 1023  # conversion del valor de velocidad del AX12 [0..3FF] a mm/s
    L_AXIS = 1.0
    INITIAL_POS_X = 50
    INITIAL_POS_Y = 350
    INITIAL_POS_THETA = math.pi / 2
    ORIENT_L = 1  # Orientacion motor izquierdo
    ORIENT_R = -1  # Orientacion motor derecho

    def __init__(self, initial_pos_x, initial_pos_y, initial_theta, mundo, socket_conn,
                 log_data = True,
                 motor_id_l=MOTOR_ID_L, motor_id_r=MOTOR_ID_R, sensor_id=SENSOR_ID,
                 ):

        # Initial values for position and orientation
        self.initial_pos_x = initial_pos_x
        self.initial_pos_y = initial_pos_y
        self.initial_theta = initial_theta

        # World used for the simulation
        self.world = mundo

        # TODO: Find world size
        self.max_y =
        self.max_x =

        self.log_data = log_data
        self.log = None

        self.socket_conn = socket_conn
        # Starting logger
        self.logger = logging.getLogger('pae.sim.Sim')

        # Sensor and motors IDs
        self.motor_id_l = motor_id_l
        self.motor_id_r = motor_id_r
        self.sensor_id = sensor_id

        # AX12 motors, left and right
        self.AX12 = {self.motor_id_l: AX(self.motor_id_l),
                   self.motor_id_r: AX(self.motor_id_r),}
        # AXS1 sensor modules
        self.AXS1 = AX(self.sensor_id)

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
        self.t_last_upd = 0

        self.simulator_running = 1

        self.reset_robot()

    def enable_data_logging(self):
        self.log = open(OUTPUT_FILE_NAME, 'a')
        self.log_data = True

    def disable_data_logging(self):
        if self.log is not None:
            self.log.close()
            self.log = None
        self.log_data = False

    def reset_robot(self):
        """ Reset the simulation status

        :return:
        """
        self.x = self.initial_pos_x
        self.y = self.initial_pos_x
        self.theta = self.initial_theta

        self.sim_step = 0

        self.AX12[self.motor_id_l].reset()
        self.AX12[self.motor_id_r].reset()
        self.AXS1.reset()

        self.simulator_running = 1

        self.update_sensor_data()

    def obstaculo(self, x, y):
        """ Return True if there is a obstacle in (x,y)

        Los datos se han cargado como un array[4096, 128] => [y, x]
        y es la fila [0..4095]
        x es la columna: bit n de 0..31, dentro de uno de los 0..127 bloques de una fila
        En que bloque se encuentra x?

        :return:
        """

        p_offset = x >> self.WORLD__MAX_2POW  # INT(x/32)
        p_bit = (self.WORLD__N_BITS - 1) - (x - (p_offset << self.WORLD__MAX_2POW))

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
        if self.obstaculo():
            self.logger.error("***** COLLISION DETECTED AT", self.x, self.y, "simulator step",
                              self.sim_step, "\n")
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
        if self.AX12[motor_id][AX_registers.GOAL_SPEED_H.value] & 0x04:
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
            self.x_p = math.cos(self.w * self.DELTA_T) * (self.x_p - self.icc_x) \
                       - math.sin(self.w * self.DELTA_T) * (self.y_p - self.icc_y) \
                       + self.icc_x
            self.y_p = math.sin(self.w * self.DELTA_T) * (self.x_p - self.icc_x) \
                       + math.cos(self.w * self.DELTA_T) * (self.y_p - self.icc_y) \
                       + self.icc_y

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


    def update_movement_simulator_values(self):
        """ Update, if required, the position and sensor information

        :return:
        """
        #while True
        objective_delay = self.SIM_STEP_MS_TIME
        elapsed, true_elapsed_time = self.elapsed_time(objective_delay)
        if elapsed:
            objective_delay -= (true_elapsed_time - self.SIM_STEP_MS_TIME)
            self.t_last_upd = time.time()
            self.sim_step += 1
            if self.MAX_SIM_STEPS != 0 and self.sim_step >= self.MAX_SIM_STEPS:
                self.logger.warning("***** SIMULATION END REACHED. STOPPING SIMULATOR\n")
                self.simulator_running = 0
                return False  # False pq ya no conviene actualizar nada desde el hilo
            self.calculate_new_position()
            if not self.check_out_of_bounds():
                self.update_sensor_data()
                if self.check_colision():
                    return False  # False pq no conviene actualizar nada desde el hilo
                # Mandamos las nuevas coordenadas al socket de la ventana grafica:
                self.socket_conn.send("%.2f, %.2f\n" % (self.x_p, self.y_p))
                # Si esta activada la grabacion, escribimos los datos al fichero de salida:
                if self.log_data:
                    self.log.write("%.2f, %.2f, %.3f, %.2f, %.2f\n" % (self.x_p, self.y_p,
                                                                              self.theta, self.v_l,
                                                                              self.v_r))
        yield

