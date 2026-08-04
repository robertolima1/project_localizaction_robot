#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ruido_uwb.py — soma ruído gaussiano extra, configurável, ao ranging UWB
bruto do plugin (libgtec_uwb_plugin), e republica num tópico separado.

O plugin (fonte externa, uwb_gazebosensorplugins) não expõe nenhum
parâmetro de ruído via SDF — os desvios-padrão de range e o ~5° (3-sigma)
de erro angular estão fixos no código C++ dele (UwbPlugin.cpp), sem
`xacro:arg` nem rosparam possível. Pra variar o ruído UWB entre rodadas
de experimento sem recompilar o plugin, este nó lê o ranging bruto em
/gtec/toa/ranging, soma ruído extra e publica em
/gtec/toa/ranging_ruidoso — é isso que ekf_uwb.py e
localizacao_trilateracao.py assinam por padrão (ver ~ranging_topic de
cada um, repassado pelo bringup.launch).

Com sigma_range_extra=0.0 e sigma_angle_extra=0.0 (padrão — sem mudar
nada na hora de rodar), a saída é idêntica à entrada: o nó vira um
passa-through, sempre ligado, sem custo de complexidade condicional no
launch.
"""

import math
import random

import rospy
from gtec_msgs.msg import Ranging


class RuidoUWB(object):

    def __init__(self):
        self.sigma_range_extra = rospy.get_param("~sigma_range_extra", 0.0)  # metros
        self.sigma_angle_extra = rospy.get_param("~sigma_angle_extra", 0.0)  # radianos

        topico_entrada = rospy.get_param("~topico_entrada", "/gtec/toa/ranging")
        topico_saida = rospy.get_param("~topico_saida", "/gtec/toa/ranging_ruidoso")

        self.pub = rospy.Publisher(topico_saida, Ranging, queue_size=20)
        rospy.Subscriber(topico_entrada, Ranging, self.cb, queue_size=20)

        rospy.loginfo("ruido_uwb: %s -> %s (sigma_range_extra=%.3f m, "
                      "sigma_angle_extra=%.4f rad)",
                      topico_entrada, topico_saida,
                      self.sigma_range_extra, self.sigma_angle_extra)

    def cb(self, msg):
        saida = Ranging()
        saida.anchorId = msg.anchorId
        saida.tagId = msg.tagId
        saida.seq = msg.seq
        saida.rss = msg.rss
        saida.errorEstimation = msg.errorEstimation

        range_m = msg.range / 1000.0
        if self.sigma_range_extra > 0.0:
            range_m += random.gauss(0.0, self.sigma_range_extra)
        saida.range = int(round(range_m * 1000.0))

        angle = msg.angle
        if self.sigma_angle_extra > 0.0 and not math.isnan(angle):
            angle += random.gauss(0.0, self.sigma_angle_extra)
        saida.angle = angle

        self.pub.publish(saida)


if __name__ == "__main__":
    rospy.init_node("ruido_uwb")
    RuidoUWB()
    rospy.spin()
