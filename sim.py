# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE Simulator Core
"""

import math
import time
import logging

from global_config import MOTOR_ID_L, MOTOR_ID_R, SENSOR_ID

module_logger = logging.getLogger('PAE.sim')

from AX import AX

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
    t_last_upd = 0.0
    INITIAL_POS_X = 50
    INITIAL_POS_Y = 350
    INITIAL_POS_THETA = math.pi / 2
    ORIENT_L = 1  # Orientacion motor izquierdo
    ORIENT_R = -1  # Orientacion motor derecho
    # V_inicial_demo_L = 0x3FF
    # V_inicial_demo_R = 0x7FF
    V_inicial_demo_L = 0
    V_inicial_demo_R = 0

    def __init__(self, initial_pos_x, initial_pos_y, initial_theta, mundo):
        self.initial_pos_x = initial_pos_x
        self.initial_pos_y = initial_pos_y
        self.initial_theta = initial_theta
        self.x = initial_pos_x
        self.y = initial_pos_y

        self.world = mundo
        self.logger = logging.getLogger('pae.sim.Sim')

        self.AX = {}
        self.AX[MOTOR_ID_L] = AX()
        self.AX[MOTOR_ID_R] = AX()
        self.AX[SENSOR_ID] = AX()

        self.reset_robot()
        
    def reset_robot(self):
        self.x = self.initial_pos_x
        self.y = self.initial_pos_x
        self.theta = self.initial_theta

        self.sim_step = 0

        self.AX[MOTOR_ID_L]._reset()
        self.AX[MOTOR_ID_R]._reset()
        self.AX[SENSOR_ID]._reset()

        update_sensor_data(robot_pos)

    def obstaculo(self):
        # Parametros: uint16_t x, uint16_t y, const uint32_t *mundo
        mundo = self.world
        # Los datos se han cargado como un array[4096, 128] => [y, x]
        # y es la fila [0..4095]
        # x es la columna: bit n de 0..31, dentro de uno de los 0..127 bloques de una fila
        # En que bloque se encuentra x?
        p_offset = self.x >> self.WORLD__MAX_2POW  # INT(x/32)
        p_bit = (self.WORLD__N_BITS - 1) - (self.x - (p_offset << self.WORLD__MAX_2POW))

        if mundo[self.y, p_offset] & (1 << p_bit):
            return True
        return False

    def sensor_distance(self, x0, y0, theta):
        modulo = 0.0  # modulo del vector de desplazamiento en la direccion de un sensor
        x = 0.0
        y = 0.0  # componentes del vector de desplazamiento en la direccion de un sensor
        indice = 0  # Distancia al obstaculo
        u8_mod = 0  # modulo redondeado, a 8 bits

        # incrementos del vector de desplazamiento en la direccion de un sensor:
        dx = math.cos(theta)
        dy = math.sin(theta)

        while (modulo < 255) and not (self.obstaculo(round(x0 + x), round(y0 + y))):
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
        self.theta_r = self.theta - math.pi / 2
        self.theta_l = self.theta + math.pi / 2

        # Sensor central:
        centro = self.sensor_distance(self.x, self.y, self.theta)

        # Sensor izquierda:
        izq = self.sensor_distance(self.x, self.y, self.theta_l)

        # Sensor derecha
        der = self.sensor_distance(self.x, self.y, self.theta_r)

        return izq, centro, der

    def elapsed_time(self, t1, milliseconds):
        # Parametros: clock_t t1, uint32_t miliseconds, int32_t *true_elapsed_time
        t2 = time.time()
        true_elapsed_time = (t2 - t1) * 1000  # para tenerlo en ms
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
        if self.x > self.ANCHO - 1 or self.y > self.ALTO - 1 or self.x < 0 or self.y < 0:
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

    # ** Reads from the dynamixel memory the speed of a endless turning wheel
    #  *
    #  * @param v Pointer were the signed speed is stored
    #  * @param motor_id ID of the Dynamixel module
    #  */
    def _speed_dyn_2_speed_int(motor_id):
        # Parametros: int16_t *v, uint8_t motor_id
        v = AX12[motor_id - 1][DYN_REG__GOAL_SPEED_L]
        v |= ((AX12[motor_id - 1][DYN_REG__GOAL_SPEED_H] & 0x03) << 8)
        if AX12[motor_id - 1][DYN_REG__GOAL_SPEED_H] & 0x04:
            v *= -1
        return v

    # /** Read the speed of the dynamixel modules and store them inside the position structure
    def read_speed():
        robot_pos_str.iv_l = _speed_dyn_2_speed_int(ID_L) * ORIENT_L
        robot_pos_str.iv_r = _speed_dyn_2_speed_int(ID_R) * ORIENT_R
        return

    # /** Update the position and orientation of the robot using two wheel differential drive kinematics
    def calculate_new_position():
        # // http://www.cs.columbia.edu/~allen/F15/NOTES/icckinematics.pdf
        read_speed()
        robot_pos_str.v_l = CNTS_2_MM * robot_pos_str.iv_l
        robot_pos_str.v_r = CNTS_2_MM * robot_pos_str.iv_r

        if robot_pos_str.iv_l == robot_pos_str.iv_r:
            robot_pos_str.x_p += robot_pos_str.v_l * DELTA_T * math.cos(robot_pos_str.theta)
            robot_pos_str.y_p += robot_pos_str.v_r * DELTA_T * math.sin(robot_pos_str.theta)
        else:
            robot_pos_str.r = (L_AXIS / 2) * (robot_pos_str.v_l + robot_pos_str.v_r) \
                              / (robot_pos_str.v_r - robot_pos_str.v_l)
            robot_pos_str.w = (robot_pos_str.v_r - robot_pos_str.v_l) / L_AXIS
            robot_pos_str.icc_x = robot_pos_str.x_p \
                                  - robot_pos_str.r * math.sin(robot_pos_str.theta)
            robot_pos_str.icc_y = robot_pos_str.y_p \
                                  + robot_pos_str.r * math.cos(robot_pos_str.theta)
            robot_pos_str.x_p = math.cos(robot_pos_str.w * DELTA_T) * (robot_pos_str.x_p - robot_pos_str.icc_x) \
                                - math.sin(robot_pos_str.w * DELTA_T) * (robot_pos_str.y_p - robot_pos_str.icc_y) \
                                + robot_pos_str.icc_x
            robot_pos_str.y_p = math.sin(robot_pos_str.w * DELTA_T) * (robot_pos_str.x_p - robot_pos_str.icc_x) \
                                + math.cos(robot_pos_str.w * DELTA_T) * (robot_pos_str.y_p - robot_pos_str.icc_y) \
                                + robot_pos_str.icc_y

            robot_pos_str.theta += robot_pos_str.w * DELTA_T
            if robot_pos_str.theta < -math.pi:
                robot_pos_str.theta += 2 * math.pi
            elif robot_pos_str.theta > math.pi:
                robot_pos_str.theta -= 2 * math.pi
        robot_pos_str.x = round(robot_pos_str.x_p)
        robot_pos_str.y = round(robot_pos_str.y_p)

        # if SIMUL_Save:
        #     fichero_log.write("%.2f, %.2f, %.3f, %.2f, %.2f\n" %(robot_pos_str.x_p, robot_pos_str.y_p,
        #         robot_pos_str.theta, robot_pos_str.v_l, robot_pos_str.v_r))
        return

    # /** Update the sensor data taking into account the new position
    def update_sensor_data(robot_pos):
        distancia_left, distancia_center, distancia_right = distance(robot_pos)
        # Actualizamos la memoria de los modulos: (duplicando los sensores de un modulo al otro)
        AX12[0][DYN_REG__IR_LEFT] = distancia_left  # Left IR
        AX12[0][DYN_REG__CENTER_IR_SENSOR] = distancia_center  # Center IR
        AX12[0][DYN_REG__IR_RIGHT] = distancia_right  # Right IR
        AX12[1][DYN_REG__IR_LEFT] = distancia_left  # Left IR
        AX12[1][DYN_REG__CENTER_IR_SENSOR] = distancia_center  # Center IR
        AX12[1][DYN_REG__IR_RIGHT] = distancia_right  # Right IR
        return

    def calcular_distancias_demo():
        # Demo, moviendo las barras con distancias ficticias
        distancia_left = AX12[0][DYN_REG__IR_LEFT]
        distancia_right = AX12[0][DYN_REG__IR_RIGHT]
        distancia_center = AX12[0][DYN_REG__CENTER_IR_SENSOR]
        creciendo = AX12[0][0x1D]
        # Sensor izquierdo:
        if distancia_left < 1:
            distancia_left = 256
        distancia_left -= 1
        # sensor derecho:
        if distancia_right > 255:
            distancia_right = 0
        distancia_right += 1
        # Sensor centro:
        if creciendo:
            if distancia_center < 255:
                distancia_center += 1
            else:
                creciendo = 0
        else:
            if distancia_center > 0:
                distancia_center -= 1
            else:
                creciendo = 1
        AX12[0][0x1D] = creciendo
        # Actualizamos la memoria de los modulos: (duplicando los sensores de un modulo al otro)
        AX12[0][DYN_REG__IR_LEFT] = distancia_left  # Left IR
        AX12[0][DYN_REG__CENTER_IR_SENSOR] = distancia_center  # Center IR
        AX12[0][DYN_REG__IR_RIGHT] = distancia_right  # Right IR
        AX12[1][DYN_REG__IR_LEFT] = distancia_left  # Left IR
        AX12[1][DYN_REG__CENTER_IR_SENSOR] = distancia_center  # Center IR
        AX12[1][DYN_REG__IR_RIGHT] = distancia_right  # Right IR
        # fin de la demo

    # /** Update, if required, the position and sensor information
    def update_movement_simulator_values():
        global t_last_upd, simulando, fichero_log
        objective_delay = SIM_STEP_MS_TIME
        elapsed, true_elapsed_time = elapsed_time(t_last_upd, objective_delay)
        if DEMO:
            calcular_distancias_demo()
            return False  # False pq no hay que actualizar nada desde el hilo
        if elapsed:
            objective_delay -= (true_elapsed_time - SIM_STEP_MS_TIME)
            t_last_upd = time.time()
            robot_pos_str.sim_step += 1
            if MAX_SIM_STEPS != 0 and robot_pos_str.sim_step >= MAX_SIM_STEPS:
                print("***** SIMULATION END REACHED. STOPPING SIMULATOR\n")
                simulando = 0
                return False  # False pq ya no conviene actualizar nada desde el hilo
            calculate_new_position()
            if not check_out_of_bounds(habitacion.ancho, habitacion.alto):
                update_sensor_data(robot_pos_str)
                if check_colision(robot_pos_str):
                    return False  # False pq no conviene actualizar nada desde el hilo
                # Mandamos las nuevas coordenadas al socket de la ventana grafica:
                conn.send("%.2f, %.2f\n" % (robot_pos_str.x_p, robot_pos_str.y_p))
                # Si esta activada la grabacion, escribimos los datos al fichero de salida:
                if SIMUL_Save:
                    # lock = open(fichero_lock, "w")
                    with open(OUTPUT_FILE_NAME, "a") as fichero_log:
                        fichero_log.write("%.2f, %.2f, %.3f, %.2f, %.2f\n" % (robot_pos_str.x_p, robot_pos_str.y_p,
                                                                              robot_pos_str.theta, robot_pos_str.v_l,
                                                                              robot_pos_str.v_r))
                    # lock.close()
                    # os.remove(fichero_lock)
                return True  # True pq hay que actualizar mas cosas desde el hilo
        # #if DEBUG_LEVEL > 2
        #         check_simulation_end();
        # #endif
        #     }
        # }
        return False  # False pq no hay que actualizar nada desde el hilo

