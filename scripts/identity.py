#!/usr/bin/env python3
from datetime import datetime
import math
from dataclasses import dataclass
from typing import Dict, Deque
from collections import deque

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from gtec_msgs.msg import Ranging  # ajuste conforme o pacote exato
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros

COUNT_MEAN = 1
ROBOT_NAME = "robot"


@dataclass
class Anchor:
    x: float
    y: float
    time: datetime
    count: int = 0  # não usado na média deslizante, mas mantido caso você queira logar


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def heading_to_checkpoint(xr, yr, theta_r, xc, yc):
    dx = xc - xr
    dy = yc - yr
    checkpoint_theta = math.atan2(dy, dx)
    error_theta = wrap_to_pi(checkpoint_theta - theta_r)
    return checkpoint_theta, error_theta


def get_yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw

from gazebo_msgs.msg import ModelStates


class IdentityNode(object):
    def __init__(self):

        self.latest_ranges: Dict[int, float] = {}
        self.latest_ranges_real: Dict[int, float] = {}

        # self.anchors: Dict[int, Anchor] = {
        #     0: Anchor(-1.5, 2.0, datetime.now(), 0),
        #     1: Anchor(-0.5, 2.0, datetime.now(), 0),
        #     2: Anchor( 0.5, 2.0, datetime.now(), 0),
        #     3: Anchor( 1.5, 2.0, datetime.now(), 0),
        #     4: Anchor( 2.5, 2.0, datetime.now(), 0),
        #     5: Anchor(-1.5, 0.5, datetime.now(), 0),
        #     6: Anchor(-0.5, 0.5, datetime.now(), 0),
        #     7: Anchor( 0.5, 0.5, datetime.now(), 0),
        #     8: Anchor( 1.5, 0.5, datetime.now(), 0),
        #     9: Anchor( 2.5, 0.5, datetime.now(), 0),
        # }

        # Posições copiadas de worlds/campo_agricola.world (mesmo x/y de cada
        # âncora uwb_anchor0..9 — ver config/ekf_uwb.yaml, que usa o mesmo mapa).
        self.anchors: Dict[int, Anchor] = {
            0: Anchor(-0.8, -0.6, datetime.now(), 0),  # uwb_anchor0
            1: Anchor(-0.4, -0.6, datetime.now(), 0),  # uwb_anchor1
            2: Anchor( 0.0, -0.6, datetime.now(), 0),  # uwb_anchor2
            3: Anchor( 0.4, -0.6, datetime.now(), 0),  # uwb_anchor3
            4: Anchor( 0.8, -0.6, datetime.now(), 0),  # uwb_anchor4

            5: Anchor(-0.8,  0.6, datetime.now(), 0),  # uwb_anchor5
            6: Anchor(-0.4,  0.6, datetime.now(), 0),  # uwb_anchor6
            7: Anchor( 0.0,  0.6, datetime.now(), 0),  # uwb_anchor7
            8: Anchor( 0.4,  0.6, datetime.now(), 0),  # uwb_anchor8
            9: Anchor( 0.8,  0.6, datetime.now(), 0),  # uwb_anchor9
        }
        # Janela deslizante por âncora (últimos 10)
        self.range_window: Dict[int, Deque[float]] = {
            anchor_id: deque(maxlen=COUNT_MEAN) for anchor_id in self.anchors.keys()
        }

        self.target_frame = "odom"
        self.source_frame = "base_footprint_noisy"
        self.robot_xy = None
        # Se quiser exigir janela completa p/ considerar "válido"
        self.require_full_window = False
        
        self.checkpoint_pub_ = rospy.Publisher(
            "/localizacao/checkpoint_validado", PoseStamped, queue_size=10
        )

        self.odom_sub_ = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.odomCallback, queue_size=5
        )
        self.uwb_sub_ = rospy.Subscriber(
            "/gtec/toa/ranging", Ranging, self.uwbCallback, queue_size=50
        )

        # self.models_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.cb_model_states, queue_size=1)



    def cb_model_states(self, msg: ModelStates):
        # robô (tag)
        try:            
            i = msg.name.index(ROBOT_NAME)
            p = msg.pose[i].position
            self.robot_xy = (p.x, p.y, p.z)
        except ValueError:
            self.robot_xy = None

    def odomCallback(self, odom: Odometry):
        if not self.latest_ranges:
            rospy.logwarn_throttle(2.0, "Ainda não recebi ranges UWB (/gtec/toa/ranging).")
            return
    
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        yaw_robot = get_yaw_from_odom(odom)

        checkpoint_min = min(self.latest_ranges, key=self.latest_ranges.get)
        value_min = float(self.latest_ranges[checkpoint_min])

        if self.require_full_window and len(self.range_window[checkpoint_min]) < COUNT_MEAN:
            rospy.logwarn_throttle(
                2.0,
                "Janela ainda não completa p/ anchor %d (%d/%d).",
                checkpoint_min, len(self.range_window[checkpoint_min]), COUNT_MEAN
            )
            return

        anchor = self.anchors.get(checkpoint_min)
        if anchor is None:
            return

        checkpoint_valid = value_min < 1350.0
        if not checkpoint_valid:
            return

        checkpoint_theta, _ = heading_to_checkpoint(x, y, yaw_robot, anchor.x, anchor.y)

        msg_out = PoseStamped()
        msg_out.header.stamp = odom.header.stamp
        msg_out.header.frame_id = self.target_frame
        msg_out.pose.position.x = anchor.x
        msg_out.pose.position.y = anchor.y
        msg_out.pose.position.z = 0.0
        msg_out.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, checkpoint_theta))
        self.checkpoint_pub_.publish(msg_out)


    def uwbCallback(self, msg: Ranging):
        # ajuste conforme seu .msg
        anchor_id = int(msg.anchorId)
        distance = float(msg.range)
        distanceReal = float(msg.errorEstimation) #gambiarra para obter o valor real da distancia

        w = self.range_window[anchor_id]
        w.append(distance)

        # média deslizante dos últimos N (N<=10 no começo)
        self.latest_ranges[anchor_id] = sum(w) / float(len(w))
        self.latest_ranges_real[anchor_id] = distanceReal


if __name__ == "__main__":
    rospy.init_node("identity_node")
    IdentityNode()
    rospy.spin()