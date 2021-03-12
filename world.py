# !/usr/bin/python3
# -*- coding: utf-8 -*-
"""
PAE Habitacion
"""

import logging
import numpy as np

from global_config import WORLD__MAX_2POW

module_logger = logging.getLogger('PAE.world')


class World(object):

    def __init__(self, fichero_habitacion):
        # Starting logger
        self.logger = logging.getLogger('pae.world.World')

        self.logger.info("Fichero datos habitacion:" + str(fichero_habitacion))
        # Leemos los datos de la habitacion:
        parametros = np.genfromtxt(fichero_habitacion, dtype="int",
                                   delimiter=',', skip_header=2,
                                   max_rows=1, deletechars="\n")
        self.ancho = parametros.data[0]
        self.alto = parametros.data[1]
        self.logger.info("Ancho = %d; \t Alto = %d" % (self.ancho, self.alto))

        num_obstaculos = np.genfromtxt(fichero_habitacion, delimiter=',', dtype="int",
                                       skip_header=4, max_rows=1, deletechars="\n")

        self.logger.debug("Hay" + str(parametros) + "obstaculos.")

        self.obstaculos = np.genfromtxt(fichero_habitacion, dtype="int",
                                        delimiter=',', skip_header=6,
                                        max_rows=num_obstaculos)

        self.obstaculos_x0s = self.obstaculos[0:, 0]
        self.obstaculos_y0s = self.obstaculos[0:, 1]
        self.obstaculos_anchos = self.obstaculos[0:, 2]
        self.obstaculos_altos = self.obstaculos[0:, 3]
        self.logger.debug("Obstaculos:")
        self.logger.debug("n: \tx \ty \tancho \talto")
        for i in range(0, num_obstaculos - 1):
            self.logger.debug("%d:\t %d\t %d\t %d\t %d\t\n" % (i, self.obstaculos_x0s[i], self.obstaculos_y0s[i],
                                                               self.obstaculos_anchos[i], self.obstaculos_altos[i]))
        x_len = (self.ancho >> WORLD__MAX_2POW)
        self.datos = np.genfromtxt(fichero_habitacion, delimiter=',', dtype="int",
                                   skip_header=13 + num_obstaculos, skip_footer=1, deletechars="\n",
                                   usecols=np.arange(0, x_len))


if __name__ == "__main__":
    mundo = World("habitacion_003.h")
    print(mundo.datos)
