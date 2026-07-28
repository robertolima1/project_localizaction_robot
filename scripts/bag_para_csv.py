#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_para_csv.py — extrai x, y, yaw de /odom, /odometry/filtered e da pose
real do robô (/gazebo/model_states, filtrada pelo nome do modelo) de um
.bag gravado pelo bringup.launch, num único CSV em **formato largo**: uma
linha por instante de tempo, com colunas separadas por fonte
(odom_x, odom_y, odom_yaw, filtrada_x, ..., ground_truth_yaw) — pronto
pra comparar as fontes lado a lado numa planilha ou no pandas, sem
precisar pivotar depois.

As três fontes publicam em taxas e instantes diferentes (raramente batem
o mesmo timestamp exato), então "agrupar pelo tempo" aqui significa: o
tempo é arredondado para o múltiplo mais próximo de --resolucao (padrão
0,05 s = 20 Hz, a taxa do teleop/EKF deste projeto) e todas as leituras
que caem no mesmo intervalo viram uma linha só — a última leitura de cada
fonte dentro do intervalo "vence". Uma resolução mais fina reduz esse
agrupamento (mais linhas, mais buracos); mais grossa agrupa mais fontes
por linha, mas perde granularidade temporal.

`t` é o tempo de gravação no bag (relativo ao início), não o header.stamp
de cada mensagem — /gazebo/model_states não tem header, então usar o
tempo de gravação é o jeito de colocar as três fontes na mesma régua de
tempo sem precisar casar timestamps na mão.

O bringup.launch, por padrão, NÃO grava /gazebo/model_states (só /odom,
/imu, /odometry/filtered, /cmd_vel — ver launch/bringup.launch). Sem esse
tópico em algum dos .bag informados, as colunas "ground_truth_*" saem
vazias em todas as linhas — sem erro, sem aviso. Para ter a comparação
com o ground truth, grave-o à parte, num segundo terminal, enquanto dirige:

    rosbag record -O bags/ground_truth_<timestamp>.bag /gazebo/model_states

e passe os dois arquivos: este script aceita vários .bag de uma vez (útil
justamente para juntar o percurso com uma gravação separada de ground
truth) e alinha o tempo pelo instante de início mais antigo entre eles,
não pelo início de cada bag isoladamente.

Uso:
    rosrun projeto_agrobot_uwb bag_para_csv.py bags/percurso_XXXX.bag saida.csv
    rosrun projeto_agrobot_uwb bag_para_csv.py bags/percurso_XXXX.bag bags/ground_truth_XXXX.bag saida.csv --modelo agrobot --resolucao 0.1
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

FONTES_ORDEM = ["odom", "filtrada", "ground_truth"]
CAMPOS = ["x", "y", "yaw"]
RESOLUCAO_PADRAO = 0.05


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


def agrupar_por_tempo(linhas, resolucao):
    """Pivota (t, fonte, x, y, yaw) em linhas largas por intervalo de tempo:
    uma coluna por (fonte, campo). Dentro do mesmo intervalo, a leitura mais
    recente de cada fonte sobrescreve a anterior."""
    grupos = {}
    for tempo, fonte, x, y, yaw in linhas:
        intervalo = round(tempo / resolucao) * resolucao
        grupos.setdefault(intervalo, {})[fonte] = (x, y, yaw)

    for intervalo in sorted(grupos):
        valores_por_fonte = grupos[intervalo]
        linha = [intervalo]
        for fonte in FONTES_ORDEM:
            linha.extend(valores_por_fonte.get(fonte, ("", "", "")))
        yield linha


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bags", nargs="+",
                    help="um ou mais .bag (ex.: o percurso e, separado, "
                         "uma gravação de /gazebo/model_states)")
    ap.add_argument("csv_saida", help="caminho do .csv a gerar")
    ap.add_argument("--modelo", default="agrobot",
                    help="nome do modelo do robô no Gazebo (padrão: agrobot, "
                         "ver -model em launch/gazebo.launch)")
    ap.add_argument("--resolucao", type=float, default=RESOLUCAO_PADRAO,
                    help="tamanho do intervalo de tempo (s) usado para "
                         "agrupar as fontes numa mesma linha (padrão: %s)"
                         % RESOLUCAO_PADRAO)
    args = ap.parse_args()

    linhas_longas = extrair_linhas(args.bags, args.modelo)
    linhas = list(agrupar_por_tempo(linhas_longas, args.resolucao))

    cabecalho = ["t"] + ["%s_%s" % (fonte, campo)
                        for fonte in FONTES_ORDEM for campo in CAMPOS]

    with open(args.csv_saida, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cabecalho)
        w.writerows(linhas)

    print("%d linhas escritas em %s" % (len(linhas), args.csv_saida))


if __name__ == "__main__":
    main()
