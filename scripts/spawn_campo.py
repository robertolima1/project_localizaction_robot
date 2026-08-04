#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spawn_campo.py — spawna no Gazebo, em runtime, uma planta + uma âncora UWB
para cada posição gerada por layout_ancoras.gerar_ancoras(~n_ancoras).

Antes, plantas (planta_1_1..planta_2_5) e âncoras (uwb_anchor0..9) eram 20
blocos <model> fixos em worlds/campo_agricola.world — pra mudar a
quantidade de âncoras era preciso editar esse XML à mão, mantendo os pares
planta/âncora em sincronia com config/*.yaml e os ANCORAS_PADRAO hardcoded
nos scripts de localização. Agora o .world só tem chão/sol/física, e este
nó spawna os pares via serviço /gazebo/spawn_sdf_model, com layout_ancoras
como fonte única de posições — mesma lista que ekf_localizacao_uwb.py,
localizacao_trilateracao.py e identity.py usam como ANCORAS_PADRAO.

Uma âncora por planta (mesmo x/y): a âncora fica pousada na cabeça da
planta (altura_ancora=0.35, planta vai de 0 a 0.30 m — base da caixa de
0.1 m encosta no topo do cilindro, sem sobrepor). Ver worlds/campo_agricola.world
para o motivo do nome precisar começar com o anchor_prefix do plugin
("uwb_anchor") e terminar em número.
"""

import os
import sys

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from layout_ancoras import N_ANCORAS_PADRAO, gerar_ancoras


PLANTA_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{nome}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><cylinder><radius>0.05</radius><length>0.30</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>0.05</radius><length>0.30</length></cylinder></geometry>
        <material><ambient>0.13 0.45 0.13 1</ambient><diffuse>0.13 0.45 0.13 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""

ANCORA_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{nome}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.1 0.1 0.1</size></box></geometry>
        <material><ambient>1.0 0.5 0.0 1</ambient><diffuse>1.0 0.5 0.0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


def pose(x, y, z):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = 1.0
    return p


def main():
    rospy.init_node("spawn_campo")

    n_ancoras = rospy.get_param("~n_ancoras", N_ANCORAS_PADRAO)
    altura_planta = rospy.get_param("~altura_planta", 0.30)
    altura_ancora = rospy.get_param("~altura_ancora", 0.35)
    ancoras = rospy.get_param("~ancoras", gerar_ancoras(n_ancoras))

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    for anc in ancoras:
        aid, x, y = int(anc["id"]), anc["x"], anc["y"]
        spawn("planta_%d" % aid, PLANTA_SDF.format(nome="planta_%d" % aid),
              "", pose(x, y, altura_planta / 2.0), "world")
        spawn("uwb_anchor%d" % aid, ANCORA_SDF.format(nome="uwb_anchor%d" % aid),
              "", pose(x, y, altura_ancora), "world")

    rospy.loginfo("spawn_campo: %d pares planta+âncora spawnados (n_ancoras=%d)",
                  len(ancoras), n_ancoras)


if __name__ == "__main__":
    main()
