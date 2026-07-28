#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ekf_localizacao_uwb.py — localização por EKF com correspondência conhecida
(Thrun, Burgard & Fox, "Probabilistic Robotics", Tabela 7.2), usando as
âncoras UWB do mundo (uwb_anchor0..9) como mapa de referência absoluta.

Alternativa ao ekf_localization_node (robot_localization): nunca os dois
ao mesmo tempo — disputariam a mesma TF odom->base_footprint. Escolhido
pelo arg `metodo` de launch/localizacao.launch.

    Estado    x_t = (x, y, theta)^T   no frame `odom`
    Controle  u_t = (v, w)^T          v de /odom (encoder), w de /imu (giro)
    Medida    z_t^i = (r, phi)^T      range-bearing por âncora (cap. 6.6)
    Mapa      m = {j: (m_j,x, m_j,y)} âncoras fixas — ver config/ekf_uwb.yaml
    Corresp.  c^i = j                 CONHECIDA: msg.anchorId (gtec_msgs/Ranging)

Sem a dimensão de assinatura "s" da Tabela 7.2: a correspondência já vem
resolvida no próprio dado (anchorId), então não sobra ambiguidade para uma
assinatura desambiguar. Cada mensagem de ranging traz uma âncora só, então
a correção roda uma âncora por vez (equivalente a rodar o "for" da tabela
com um elemento — não muda o resultado, só não há o que somar).

Este projeto não tem frame `map` (frame global é `odom`, ver premissa do
projeto) — este nó publica odom->base_footprint diretamente, do mesmo
jeito que o ekf_localization_node. Isso só é consistente se o frame odom
coincidir, em t=0, com o frame do Gazebo onde as âncoras estão declaradas
no .world — o que vale exatamente quando o robô nasce em x=y=yaw=0
(padrão do bringup.launch). Subindo com x/y/yaw != 0, repita os mesmos
valores em ~origem_x/~origem_y/~origem_yaw (o bringup.launch já faz isso
automaticamente — ver launch/localizacao.launch).
"""

import math
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from gtec_msgs.msg import Ranging


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class EKFLocalizacaoUWB(object):

    def __init__(self):
        self.lock = threading.Lock()

        origem_x = rospy.get_param("~origem_x", 0.0)
        origem_y = rospy.get_param("~origem_y", 0.0)
        origem_yaw = rospy.get_param("~origem_yaw", 0.0)

        self.mu = np.zeros((3, 1))
        self.Sigma = np.diag(rospy.get_param("~sigma0_diag", [0.25, 0.25, 0.05]))
        self.R = np.diag(rospy.get_param("~r_diag", [0.01, 0.01, 0.01]))
        sigma_r = rospy.get_param("~sigma_r", 0.05)
        sigma_phi = rospy.get_param("~sigma_phi", 0.0873)
        self.Q = np.diag([sigma_r ** 2, sigma_phi ** 2])

        # Ver nota [N1]: projeta o range 3D bruto do plugin (âncora e tag em
        # alturas diferentes) na distância horizontal que o modelo
        # range-bearing 2D espera.
        altura_ancora = rospy.get_param("~altura_ancora", 1.0)
        altura_tag = rospy.get_param("~altura_tag", 0.31)
        self.dz2 = (altura_ancora - altura_tag) ** 2

        self.max_dt = rospy.get_param("~max_dt", 1.0)
        self.eps_w = rospy.get_param("~eps_w", 1e-4)      # ver nota [N2]
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        # Mapa m: converte cada âncora do frame do mundo Gazebo (onde estão
        # declaradas no .world) para o frame odom, via a pose de spawn
        # origem_x/y/yaw — ver docstring acima e nota [N3].
        c, s = math.cos(origem_yaw), math.sin(origem_yaw)
        self.mapa = {}
        for anc in rospy.get_param("~ancoras", []):
            wx, wy = anc["x"] - origem_x, anc["y"] - origem_y
            self.mapa[int(anc["id"])] = (c * wx + s * wy, -s * wx + c * wy)

        self.u = np.zeros((2, 1))         # (v, w) montado a partir de odom+imu
        self.w_imu = 0.0
        self.last_stamp = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pub_odom = rospy.Publisher("/odometry/filtered", Odometry,
                                        queue_size=10)

        odom_topic = rospy.get_param("~odom_topic", "/odom")
        imu_topic = rospy.get_param("~imu_topic", "/imu")
        ranging_topic = rospy.get_param("~ranging_topic", "/gtec/toa/ranging")

        rospy.Subscriber(imu_topic, Imu, self.cb_imu, queue_size=50)
        rospy.Subscriber(odom_topic, Odometry, self.cb_odom, queue_size=50)
        rospy.Subscriber(ranging_topic, Ranging, self.cb_ranging, queue_size=20)

        rospy.loginfo("EKF UWB (Tabela 7.2, correspondência conhecida) | "
                      "%d âncoras no mapa", len(self.mapa))

    # ------------------------------------------------------------ u_t: IMU
    def cb_imu(self, msg):
        with self.lock:
            self.w_imu = msg.angular_velocity.z

    # ----------------------------------------------------- u_t: odometria
    # Dispara a predição (linhas 2-4 da Tabela 7.2) a cada mensagem de
    # odometria; só o twist é lido, a pose da odometria é ignorada (senão a
    # mesma informação entraria duas vezes no filtro).
    def cb_odom(self, msg):
        stamp = msg.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()

        with self.lock:
            self.u[0, 0] = msg.twist.twist.linear.x
            self.u[1, 0] = self.w_imu

            if self.last_stamp is None:
                self.last_stamp = stamp
                return
            dt = (stamp - self.last_stamp).to_sec()
            self.last_stamp = stamp
            if 0.0 < dt <= self.max_dt:
                self._predict(dt)
                self._publish(stamp)

    def _predict(self, dt):
        v, w = self.u[0, 0], self.u[1, 0]
        th = self.mu[2, 0]

        if abs(w) < self.eps_w:
            w = self.eps_w if w >= 0.0 else -self.eps_w      # nota [N2]

        r = v / w
        thp = th + w * dt

        # LINHA 2 (modelo de velocidade, cap. 5.3, derivação correta)
        dx = r * (math.sin(thp) - math.sin(th))
        dy = r * (math.cos(th) - math.cos(thp))
        dth = w * dt

        # LINHA 3: Jacobiano de g em relação a (x, y, theta). O sinal do
        # termo em theta aqui é o da derivada de fato (dg_x/dtheta,
        # dg_y/dtheta) — a Tabela 7.2 impressa no livro traz esse termo
        # com o sinal trocado; corrigido aqui.
        G = np.array([[1.0, 0.0, r * (math.cos(thp) - math.cos(th))],
                      [0.0, 1.0, dx],
                      [0.0, 0.0, 1.0]])

        self.mu = self.mu + np.array([[dx], [dy], [dth]])
        self.mu[2, 0] = normalize_angle(self.mu[2, 0])

        # LINHA 4: Sigma_barra_t = G_t Sigma_{t-1} G_t^T + R_t
        self.Sigma = G.dot(self.Sigma).dot(G.T) + self.R

    # =================================================== LINHAS 6-15 (uma
    # âncora por mensagem — ver docstring sobre o "for" de um elemento só)
    def cb_ranging(self, msg):
        j = int(msg.anchorId)
        if j not in self.mapa:
            return
        if math.isnan(msg.angle):
            return                                    # sem return_angle

        r_medido = msg.range / 1000.0                 # mm -> m
        r_horizontal_sq = r_medido * r_medido - self.dz2
        if r_horizontal_sq <= 0.0:
            return                                    # nota [N1]
        r_medido = math.sqrt(r_horizontal_sq)

        with self.lock:
            self._correct(r_medido, msg.angle, j)
            self._publish(rospy.Time.now())

    def _correct(self, z_r, z_phi, j):
        mx, my = self.mapa[j]

        # LINHA 8
        dx = mx - self.mu[0, 0]
        dy = my - self.mu[1, 0]

        # LINHA 9
        q = dx * dx + dy * dy
        if q < 1e-6:
            return
        sq = math.sqrt(q)

        # LINHA 10
        z_hat_r = sq
        z_hat_phi = normalize_angle(math.atan2(dy, dx) - self.mu[2, 0])

        # LINHA 11: H_t (2x3 — sem a linha de assinatura, ver docstring)
        H = np.array([[-dx / sq, -dy / sq, 0.0],
                      [dy / q, -dx / q, -1.0]])

        # LINHA 12
        S = H.dot(self.Sigma).dot(H.T) + self.Q
        K = self.Sigma.dot(H.T).dot(np.linalg.inv(S))

        innov = np.array([[z_r - z_hat_r],
                          [normalize_angle(z_phi - z_hat_phi)]])

        # LINHAS 14-15
        self.mu = self.mu + K.dot(innov)
        self.mu[2, 0] = normalize_angle(self.mu[2, 0])
        self.Sigma = (np.eye(3) - K.dot(H)).dot(self.Sigma)

    def _pose(self):
        p = Pose()
        p.position = Point(self.mu[0, 0], self.mu[1, 0], 0.0)
        p.orientation = Quaternion(*quaternion_from_euler(0, 0, self.mu[2, 0]))
        cov = [0.0] * 36
        idx = [0, 1, 5]     # x, y, yaw dentro da matriz 6x6 do ROS
        for a, ra in enumerate(idx):
            for b, rb in enumerate(idx):
                cov[ra * 6 + rb] = self.Sigma[a, b]
        return p, cov

    def _publish(self, stamp):
        pose, cov = self._pose()

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose, od.pose.covariance = pose, cov
        od.twist.twist.linear.x = self.u[0, 0]
        od.twist.twist.angular.z = self.u[1, 0]
        self.pub_odom.publish(od)

        if self.publish_tf:
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.odom_frame
            msg.child_frame_id = self.base_frame
            msg.transform.translation.x = pose.position.x
            msg.transform.translation.y = pose.position.y
            msg.transform.translation.z = 0.0
            msg.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(msg)


if __name__ == "__main__":
    rospy.init_node("ekf_localizacao_uwb")
    EKFLocalizacaoUWB()
    rospy.spin()


# ===========================================================================
# NOTAS DE IMPLEMENTAÇÃO
#
# [N1] O plugin gtec_uwb_plugin mede a distância 3D reta entre a âncora
#      (z=1,0 m, fixa no .world) e a tag (fundida no base_link + o
#      tag_z_offset=0,3 m do plugin, ~0,31 m acima do chão — ver
#      agrobot.gazebo.xacro). Essa diferença de altura (~0,69 m) é grande
#      demais para ignorar frente às distâncias horizontais do cenário
#      (as fileiras têm ~1,6 m). O range bruto é sempre >= 0,69 m mesmo
#      com o robô embaixo da âncora; sem essa projeção o filtro veria uma
#      distância horizontal artificialmente maior que a real. altura_ancora
#      e altura_tag em config/ekf_uwb.yaml são constantes conhecidas (não
#      estimadas), coerente com a hipótese de mapa conhecido da Tabela 7.2.
#
# [N2] v/w é indefinido em w=0; satura-se w perto de zero só para o código
#      não dividir por zero. Com eps_w=1e-4 o arco resultante é
#      indistinguível de uma reta na escala do robô.
#
# [N3] O frame `odom` deste projeto não tem origem física própria — ele é,
#      por convenção do ROS, onde quer que o robô esteja quando o nó de
#      odometria liga. Como o .world define as âncoras em coordenadas
#      absolutas do Gazebo, a única forma de usar esse mapa sem introduzir
#      um frame novo é assumir (e documentar) que odom coincide com o
#      mundo em t=0 — verdadeiro sempre que o spawn for x=y=yaw=0.
# ===========================================================================
