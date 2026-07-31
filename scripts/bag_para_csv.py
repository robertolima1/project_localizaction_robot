#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_para_csv.py — extrai x, y, yaw de /odom, /odometry/filtered e da pose
real do robô (/gazebo/model_states, filtrada pelo nome do modelo), mais
range/angle de cada âncora UWB (/gtec/toa/ranging), de um .bag gravado
pelo bringup.launch, num único CSV em **formato largo**: uma linha por
instante de tempo, com colunas separadas por fonte/âncora
(odom_x, odom_y, odom_yaw, filtrada_x, ..., ground_truth_yaw,
anchor0_range, anchor0_angle, anchor1_range, ...) — pronto pra comparar
tudo lado a lado numa planilha ou no pandas, sem precisar pivotar depois.

As fontes publicam em taxas e instantes diferentes (raramente batem o
mesmo timestamp exato), então "agrupar pelo tempo" aqui significa: o
tempo é arredondado para o múltiplo mais próximo de --resolucao (padrão
0,05 s = 20 Hz, a taxa do teleop/EKF deste projeto) e todas as leituras
que caem no mesmo intervalo viram uma linha só — a última leitura de cada
fonte/âncora dentro do intervalo "vence". Uma resolução mais fina reduz
esse agrupamento (mais linhas, mais buracos); mais grossa agrupa mais
fontes por linha, mas perde granularidade temporal. Como cada âncora só
publica de tempos em tempos (update_rate do plugin, ciclando entre as
âncoras), é normal a maioria das colunas anchorN_* saírem vazias na
maior parte das linhas — não é erro.

`t` é o tempo de gravação no bag (relativo ao início), não o header.stamp
de cada mensagem — /gazebo/model_states e o Ranging não têm o mesmo
referencial de header, então usar o tempo de gravação é o jeito de
colocar tudo na mesma régua de tempo sem precisar casar timestamps na mão.

As colunas ground_truth_* e anchorN_* só aparecem preenchidas se o .bag
tiver gravado /gazebo/model_states e /gtec/toa/ranging (o bringup.launch
grava os dois por padrão — ver launch/bringup.launch). Sem esses tópicos
em algum dos .bag informados, as colunas correspondentes saem vazias em
todas as linhas — sem erro, sem aviso. Se precisar juntar uma gravação
separada desses tópicos (ex.: um .bag antigo, gravado antes de eles
entrarem no bringup), passe vários arquivos: este script aceita vários
.bag de uma vez e alinha o tempo pelo instante de início mais antigo
entre eles, não pelo início de cada bag isoladamente.

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
TOPICO_RANGING = "/gtec/toa/ranging"

FONTES_ORDEM = ["odom", "filtrada", "ground_truth"]
CAMPOS_POSE = ["x", "y", "yaw"]
CAMPOS_RANGING = ["range", "angle"]
RESOLUCAO_PADRAO = 0.05


def extrair_linhas(caminhos_bag, modelo):
    """Gera (tempo, chave, valores). `chave` é uma fonte de pose
    ("odom"/"filtrada"/"ground_truth", valores = {x, y, yaw}) ou uma
    âncora ("anchorN", valores = {range (m), angle (rad)})."""
    topicos = list(TOPICOS_ODOMETRIA) + [TOPICO_GROUND_TRUTH, TOPICO_RANGING]
    t0 = min(rosbag.Bag(c).get_start_time() for c in caminhos_bag)
    for caminho in caminhos_bag:
        with rosbag.Bag(caminho) as bag:
            for topic, msg, t in bag.read_messages(topics=topicos):
                tempo = t.to_sec() - t0

                if topic == TOPICO_RANGING:
                    yield (tempo, "anchor%d" % msg.anchorId,
                          {"range": msg.range / 1000.0, "angle": msg.angle})
                    continue

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
                yield (tempo, fonte, {"x": pose.position.x,
                                     "y": pose.position.y,
                                     "yaw": yaw})


def agrupar_por_tempo(linhas, resolucao):
    """Agrupa (tempo, chave, valores) por intervalo de --resolucao. Dentro
    do mesmo intervalo, a leitura mais recente de cada chave sobrescreve a
    anterior. Retorna {intervalo: {chave: valores}}."""
    grupos = {}
    for tempo, chave, valores in linhas:
        intervalo = round(tempo / resolucao) * resolucao
        grupos.setdefault(intervalo, {})[chave] = valores
    return grupos


def montar_colunas(grupos):
    """(chave, campo) na ordem: fontes de pose fixas (FONTES_ORDEM x
    CAMPOS_POSE), depois as âncoras que apareceram nos dados, em ordem
    crescente de id (CAMPOS_RANGING cada)."""
    ids_ancoras = sorted({
        int(chave[len("anchor"):])
        for valores_por_chave in grupos.values()
        for chave in valores_por_chave
        if chave.startswith("anchor")
    })

    colunas = [(fonte, campo) for fonte in FONTES_ORDEM for campo in CAMPOS_POSE]
    colunas += [("anchor%d" % i, campo)
               for i in ids_ancoras for campo in CAMPOS_RANGING]
    return colunas


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

    grupos = agrupar_por_tempo(extrair_linhas(args.bags, args.modelo), args.resolucao)
    colunas = montar_colunas(grupos)

    cabecalho = ["t"] + ["%s_%s" % (chave, campo) for chave, campo in colunas]

    linhas_saida = []
    for intervalo in sorted(grupos):
        valores_por_chave = grupos[intervalo]
        linha = [intervalo]
        for chave, campo in colunas:
            linha.append(valores_por_chave.get(chave, {}).get(campo, ""))
        linhas_saida.append(linha)

    with open(args.csv_saida, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cabecalho)
        w.writerows(linhas_saida)

    print("%d linhas escritas em %s" % (len(linhas_saida), args.csv_saida))


if __name__ == "__main__":
    main()
