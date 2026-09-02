#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
registro_visitas.py — assina ~topico (default /checkpoint_visitado,
std_msgs/Int32, publicado por scripts/visita_checkpoints.py sempre que um
checkpoint é alcançado) e registra cada visita (id do checkpoint + carimbo
de tempo) num CSV, uma linha por visita — ~arquivo_csv controla o destino.

Se o arquivo já existir, as visitas são acrescentadas ao final (não
sobrescreve corridas anteriores); o cabeçalho só é escrito na criação.
"""

import csv
import os

import rospy
from std_msgs.msg import Int32


class RegistroVisitas(object):

    def __init__(self):
        self.caminho = rospy.get_param("~arquivo_csv",
                                        os.path.expanduser("~/visitas_checkpoints.csv"))
        topico = rospy.get_param("~topico", "/checkpoint_visitado")

        arquivo_novo = not os.path.exists(self.caminho)
        self.arquivo = open(self.caminho, "a", newline="")
        self.writer = csv.writer(self.arquivo)
        if arquivo_novo:
            self.writer.writerow(["t", "checkpoint_id"])
            self.arquivo.flush()

        rospy.Subscriber(topico, Int32, self.cb_visita, queue_size=10)
        rospy.on_shutdown(self._fechar)

        rospy.loginfo("registro_visitas: assinando %s, gravando em %s (%s)",
                       topico, self.caminho,
                       "arquivo novo" if arquivo_novo else "adicionando ao existente")

    def cb_visita(self, msg):
        t = rospy.Time.now().to_sec()
        self.writer.writerow(["%.3f" % t, msg.data])
        self.arquivo.flush()
        rospy.loginfo("Checkpoint %d registrado (t=%.3f)", msg.data, t)

    def _fechar(self):
        self.arquivo.close()


if __name__ == "__main__":
    rospy.init_node("registro_visitas")
    RegistroVisitas()
    rospy.spin()
