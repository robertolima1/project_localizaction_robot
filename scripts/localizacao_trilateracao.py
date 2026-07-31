#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localizacao_trilateracao.py — localização por trilateração básica usando
as âncoras UWB (ver scripts/exemplo_trilateracao.py para o algoritmo
isolado). Sem filtro nenhum: a cada leitura nova de /gtec/toa/ranging,
resolve (x, y) com a leitura mais recente de cada âncora (mínimo 3,
descartando as velhas demais) e publica direto — sem predição, sem
correção bayesiana, sem covariância real.

Terceira opção de localização, alternativa a robot_localization e a
ekf_uwb — nunca duas ao mesmo tempo (disputariam a mesma TF
odom->base_footprint). Escolhida pelo arg `metodo` de
launch/localizacao.launch (metodo:=trilateracao).

Serve de comparação didática com o ekf_uwb.py: mesma fonte de dados
(âncoras UWB), mas geometria pura em vez de Tabela 7.2 — dá pra ver o
quanto a predição/fusão do EKF melhora sobre a trilateração "crua",
principalmente enquanto menos de 3 âncoras estão visíveis (aqui o nó
simplesmente não publica; o EKF continua predizendo com odom+IMU).

Trilateração 2D só dá (x, y) — não tem informação de orientação. A
orientação (yaw) publicada vem direto da IMU, mesma fonte absoluta usada
pelos outros dois métodos deste projeto.
"""

import math
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gtec_msgs.msg import Ranging

# Mesmo mapa de âncoras do ekf_localizacao_uwb.py — ver worlds/campo_agricola.world.
ANCORAS_PADRAO = [
    {"id": 0, "x": -0.8, "y": -0.5},
    {"id": 1, "x": -0.4, "y": -0.5},
    {"id": 2, "x": 0.0, "y": -0.5},
    {"id": 3, "x": 0.4, "y": -0.5},
    {"id": 4, "x": 0.8, "y": -0.5},
    {"id": 5, "x": -0.8, "y": 0.5},
    {"id": 6, "x": -0.4, "y": 0.5},
    {"id": 7, "x": 0.0, "y": 0.5},
    {"id": 8, "x": 0.4, "y": 0.5},
    {"id": 9, "x": 0.8, "y": 0.5},
]


def trilatera(ancoras_xy, distancias):
    """ancoras_xy: lista de (x, y), pelo menos 3. distancias: mesma ordem.
    Retorna (x, y) estimado — ver scripts/exemplo_trilateracao.py."""
    ancoras_xy = np.asarray(ancoras_xy, dtype=float)
    distancias = np.asarray(distancias, dtype=float)
    n = len(ancoras_xy)

    x_ref, y_ref = ancoras_xy[-1]
    r_ref = distancias[-1]

    A = np.zeros((n - 1, 2))
    b = np.zeros(n - 1)
    for i in range(n - 1):
        x_i, y_i = ancoras_xy[i]
        r_i = distancias[i]
        A[i] = [2 * (x_ref - x_i), 2 * (y_ref - y_i)]
        b[i] = (r_i**2 - r_ref**2) - (x_i**2 - x_ref**2) - (y_i**2 - y_ref**2)

    posicao, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return float(posicao[0]), float(posicao[1])


class LocalizacaoTrilateracao(object):

    def __init__(self):
        self.lock = threading.Lock()

        origem_x = rospy.get_param("~origem_x", 0.0)
        origem_y = rospy.get_param("~origem_y", 0.0)
        origem_yaw = rospy.get_param("~origem_yaw", 0.0)

        # Mesma conversão mundo->odom do ekf_localizacao_uwb.py — ver nota
        # [N1] lá para o porquê dessa suposição (spawn em x=y=yaw=0).
        c, s = math.cos(origem_yaw), math.sin(origem_yaw)
        self.mapa = {}
        for anc in rospy.get_param("~ancoras", ANCORAS_PADRAO):
            wx, wy = anc["x"] - origem_x, anc["y"] - origem_y
            self.mapa[int(anc["id"])] = (c * wx + s * wy, -s * wx + c * wy)

        altura_ancora = rospy.get_param("~altura_ancora", 0.35)
        altura_tag = rospy.get_param("~altura_tag", 0.31)
        self.dz2 = (altura_ancora - altura_tag) ** 2

        self.min_ancoras = rospy.get_param("~min_ancoras", 3)
        self.max_idade_leitura = rospy.get_param("~max_idade_leitura", 1.5)
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.leituras = {}  # anchor_id -> (range_m, rospy.Time)
        self.yaw_imu = 0.0

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pub_odom = rospy.Publisher("/odometry/filtered", Odometry,
                                        queue_size=10)

        imu_topic = rospy.get_param("~imu_topic", "/imu")
        ranging_topic = rospy.get_param("~ranging_topic", "/gtec/toa/ranging")

        rospy.Subscriber(imu_topic, Imu, self.cb_imu, queue_size=50)
        rospy.Subscriber(ranging_topic, Ranging, self.cb_ranging, queue_size=20)

        rospy.loginfo("Localização por trilateração (sem filtro) | "
                      "%d âncoras no mapa, mínimo %d por solução",
                      len(self.mapa), self.min_ancoras)

    def cb_imu(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_imu = yaw

    def cb_ranging(self, msg):
        j = int(msg.anchorId)
        if j not in self.mapa:
            return

        r_medido = msg.range / 1000.0  # mm -> m
        r_horizontal_sq = r_medido * r_medido - self.dz2
        if r_horizontal_sq <= 0.0:
            return
        r_medido = math.sqrt(r_horizontal_sq)

        agora = rospy.Time.now()
        with self.lock:
            self.leituras[j] = (r_medido, agora)
            self._descarta_leituras_velhas(agora)
            if len(self.leituras) >= self.min_ancoras:
                self._localiza_e_publica(agora)

    def _descarta_leituras_velhas(self, agora):
        velhas = [aid for aid, (_, t) in self.leituras.items()
                 if (agora - t).to_sec() > self.max_idade_leitura]
        for aid in velhas:
            del self.leituras[aid]

    def _localiza_e_publica(self, stamp):
        ancoras_xy = [self.mapa[aid] for aid in self.leituras]
        distancias = [r for r, _ in self.leituras.values()]

        try:
            x, y = trilatera(ancoras_xy, distancias)
        except np.linalg.LinAlgError:
            return  # âncoras quase colineares — sistema mal-condicionado

        pose = Pose()
        pose.position = Point(x, y, 0.0)
        pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.yaw_imu))

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose = pose
        # Sem filtro = sem estimativa real de incerteza. Valor fixo só para
        # o campo não ficar zerado (RViz trata covariância zero como
        # "certeza absoluta").
        od.pose.covariance[0] = od.pose.covariance[7] = 0.05
        od.pose.covariance[35] = 0.05
        self.pub_odom.publish(od)

        if self.publish_tf:
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.odom_frame
            msg.child_frame_id = self.base_frame
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = 0.0
            msg.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(msg)


if __name__ == "__main__":
    rospy.init_node("localizacao_trilateracao")
    LocalizacaoTrilateracao()
    rospy.spin()
