#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_para_csv.py — extrai x, y, yaw de /odom, /odometry/filtered e da pose
real do robô (/gazebo/model_states, filtrada pelo nome do modelo) de um
.bag gravado pelo bringup.launch, num único CSV em formato longo
(t, fonte, x, y, yaw) pronto para abrir numa planilha ou no pandas.

`t` é o tempo de gravação no bag (relativo ao início), não o header.stamp
de cada mensagem — /gazebo/model_states não tem header, então usar o
tempo de gravação é o jeito de colocar as três fontes na mesma régua de
tempo sem precisar casar timestamps na mão.

O bringup.launch, por padrão, NÃO grava /gazebo/model_states (só /odom,
/imu, /odometry/filtered, /cmd_vel — ver launch/bringup.launch). Sem esse
tópico em algum dos .bag informados, a coluna "fonte" simplesmente nunca
traz "ground_truth" — sem erro, sem aviso. Para ter a comparação com o
ground truth, grave-o à parte, num segundo terminal, enquanto dirige:

    rosbag record -O bags/ground_truth_<timestamp>.bag /gazebo/model_states

e passe os dois arquivos: este script aceita vários .bag de uma vez (útil
justamente para juntar o percurso com uma gravação separada de ground
truth) e alinha o tempo pelo instante de início mais antigo entre eles,
não pelo início de cada bag isoladamente.

Uso:
    rosrun projeto_agrobot_uwb bag_para_csv.py bags/percurso_XXXX.bag saida.csv
    rosrun projeto_agrobot_uwb bag_para_csv.py bags/percurso_XXXX.bag bags/ground_truth_XXXX.bag saida.csv --modelo agrobot
"""

import argparse
import csv

import rosbag
from tf.transformations import euler_from_quaternion

TOPICOS_ODOMETRIA = {
    "/odom": "odom",
    "/odometry/filtered": "filtrada",
}
TOPICO_GROUND_TRUTH = "/gazebo/model_states"


def extrair_linhas(caminhos_bag, modelo):
    topicos = list(TOPICOS_ODOMETRIA) + [TOPICO_GROUND_TRUTH]
    t0 = min(rosbag.Bag(c).get_start_time() for c in caminhos_bag)
    for caminho in caminhos_bag:
        with rosbag.Bag(caminho) as bag:
            for topic, msg, t in bag.read_messages(topics=topicos):
                tempo = t.to_sec() - t0

                if topic == TOPICO_GROUND_TRUTH:
                    if modelo not in msg.name:
                        continue
                    pose = msg.pose[msg.name.index(modelo)]
                    fonte = "ground_truth"
                else:
                    pose = msg.pose.pose
                    fonte = TOPICOS_ODOMETRIA[topic]

                q = pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                yield (tempo, fonte, pose.position.x, pose.position.y, yaw)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bags", nargs="+",
                    help="um ou mais .bag (ex.: o percurso e, separado, "
                         "uma gravação de /gazebo/model_states)")
    ap.add_argument("csv_saida", help="caminho do .csv a gerar")
    ap.add_argument("--modelo", default="agrobot",
                    help="nome do modelo do robô no Gazebo (padrão: agrobot, "
                         "ver -model em launch/gazebo.launch)")
    args = ap.parse_args()

    linhas = sorted(extrair_linhas(args.bags, args.modelo), key=lambda l: l[0])

    with open(args.csv_saida, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "fonte", "x", "y", "yaw"])
        w.writerows(linhas)

    print("%d linhas escritas em %s" % (len(linhas), args.csv_saida))


if __name__ == "__main__":
    main()
