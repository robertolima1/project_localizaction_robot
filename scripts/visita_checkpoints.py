#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visita_checkpoints.py — leva o robô, sozinho, até cada âncora UWB (checkpoint)
em sequência, usando um controlador proporcional "vai-até-o-ponto" (gira até
apontar pro alvo, depois anda), com uma pausa em cada checkpoint. Posições
dos checkpoints vêm da mesma fonte única usada pelo resto do projeto
(scripts/layout_ancoras.py), então batem com onde as âncoras foram de fato
spawnadas (ver scripts/spawn_campo.py).

Pose atual lida de ~pose_topic (default /odometry/filtered) — funciona com
qualquer metodo de localização ativo (robot_localization, ekf_uwb ou
trilateracao publicam todos nesse tópico).
"""

import math
import os
import sys
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from layout_ancoras import N_ANCORAS_PADRAO, gerar_ancoras  # noqa: E402


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class VisitaCheckpoints(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None  # (x, y, yaw), None ate a 1a mensagem chegar

        n_ancoras = rospy.get_param("~n_ancoras", N_ANCORAS_PADRAO)
        ancoras = rospy.get_param("~ancoras", gerar_ancoras(n_ancoras))
        self.checkpoints = [(int(a["id"]), a["x"], a["y"])
                             for a in sorted(ancoras, key=lambda a: int(a["id"]))]


        meio = N_ANCORAS_PADRAO // 2
        self.checkpoints = self.checkpoints[meio:] + self.checkpoints[:meio][::-1]

        rospy.loginfo(f"Percurso será realizado pela rota {[x[0] for x  in self.checkpoints]}")
        self.tolerancia = rospy.get_param("~tolerancia", 0.10)       # m
        self.limiar_ang_giro = rospy.get_param("~limiar_ang_giro", 0.3)  # rad
        self.v_max = rospy.get_param("~v_max", 0.15)                 # m/s
        self.w_max = rospy.get_param("~w_max", 0.6)                  # rad/s
        self.kp_linear = rospy.get_param("~kp_linear", 0.5)
        self.kp_angular = rospy.get_param("~kp_angular", 1.5)
        self.taxa_hz = rospy.get_param("~taxa_hz", 10.0)
        self.pausa_s = rospy.get_param("~pausa_s", 2.0)

        pose_topic = rospy.get_param("~pose_topic", "/odometry/filtered")
        cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.pub_cmd = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        rospy.Subscriber(pose_topic, Odometry, self.cb_pose, queue_size=1)

        rospy.loginfo("visita_checkpoints: %d checkpoints, tolerancia=%.2fm, "
                       "pausa=%.1fs, pose de %s, cmd_vel em %s",
                       len(self.checkpoints), self.tolerancia, self.pausa_s,
                       pose_topic, cmd_vel_topic)

    def cb_pose(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self.lock:
            self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _pose_atual(self):
        with self.lock:
            return self.pose

    def _publica_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    def _para(self):
        self._publica_cmd(0.0, 0.0)

    def _vai_ate(self, alvo_x, alvo_y):
        rate = rospy.Rate(self.taxa_hz)
        while not rospy.is_shutdown():
            pose = self._pose_atual()
            if pose is None:
                rate.sleep()
                continue

            x, y, theta = pose
            dx, dy = alvo_x - x, alvo_y - y
            dist = math.hypot(dx, dy)
            if dist < self.tolerancia:
                return True

            erro_ang = normalize_angle(math.atan2(dy, dx) - theta)

            if abs(erro_ang) > self.limiar_ang_giro:
                v = 0.0
            else:
                v = min(self.kp_linear * dist, self.v_max)

            w = max(-self.w_max, min(self.w_max, self.kp_angular * erro_ang))

            self._publica_cmd(v, w)
            rate.sleep()

        return False

    def executar(self):
        rospy.loginfo("Aguardando primeira pose em %s...",
                       rospy.get_param("~pose_topic", "/odometry/filtered"))
        while self._pose_atual() is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        for cid, cx, cy in self.checkpoints:
            if rospy.is_shutdown():
                break
            rospy.loginfo("Indo para checkpoint %d em (%.2f, %.2f)", cid, cx, cy)
            chegou = self._vai_ate(cx, cy)
            self._para()
            if chegou:
                rospy.loginfo("Checkpoint %d alcancado — pausando %.1fs",
                               cid, self.pausa_s)
                rospy.sleep(self.pausa_s)
            else:
                rospy.loginfo("Encerrado antes de alcancar o checkpoint %d", cid)

        self._para()
        rospy.loginfo("Percurso concluido — todos os checkpoints visitados")


if __name__ == "__main__":
    rospy.init_node("visita_checkpoints")
    VisitaCheckpoints().executar()
