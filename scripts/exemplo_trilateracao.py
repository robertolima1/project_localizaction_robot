#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
exemplo_trilateracao.py — trilateração 2D básica: estima a posição (x, y)
de um robô a partir da distância medida até 3+ âncoras UWB de posição
conhecida.

Método: linearização clássica — subtrai a equação de uma âncora de
referência das demais (elimina os termos quadráticos x², y²) e resolve o
sistema linear resultante por mínimos quadrados. Com exatamente 3
âncoras e distâncias perfeitas dá solução exata; com mais âncoras, sobra
ruído nas medidas para o `lstsq` amenizar.

Não depende de ROS — é só o algoritmo, pra estudar/testar isolado. Rode
direto (`python3 exemplo_trilateracao.py`) pra ver o exemplo com âncoras
deste projeto (mesmas posições de config/ekf_uwb.yaml).
"""

import numpy as np


def trilatera(ancoras, distancias):
    """
    ancoras: lista de (x, y) — pelo menos 3.
    distancias: lista de distâncias medidas até cada âncora, mesma ordem.
    Retorna (x, y) estimado do robô.
    """
    ancoras = np.asarray(ancoras, dtype=float)
    distancias = np.asarray(distancias, dtype=float)
    n = len(ancoras)
    if n < 3:
        raise ValueError("Precisa de pelo menos 3 âncoras para trilateração 2D.")

    # Âncora de referência: a última da lista. As outras (n-1) equações
    # viram linhas do sistema linear A @ [x, y] = b.
    x_ref, y_ref = ancoras[-1]
    r_ref = distancias[-1]

    A = np.zeros((n - 1, 2))
    b = np.zeros(n - 1)
    for i in range(n - 1):
        x_i, y_i = ancoras[i]
        r_i = distancias[i]
        A[i] = [2 * (x_ref - x_i), 2 * (y_ref - y_i)]
        b[i] = (r_i**2 - r_ref**2) - (x_i**2 - x_ref**2) - (y_i**2 - y_ref**2)

    posicao, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return float(posicao[0]), float(posicao[1])


if __name__ == "__main__":
    # Âncoras deste projeto (ver config/ekf_uwb.yaml / worlds/campo_agricola.world)
    ancoras = [
        (-0.8, -0.5),  # uwb_anchor0
        (-0.4, -0.5),  # uwb_anchor1
        (0.0, -0.5),   # uwb_anchor2
        (0.8, 0.5),    # uwb_anchor9 (mais longe, pra melhorar a geometria)
    ]

    robo_real = (0.0, 0.0)

    # Distâncias "medidas" simuladas: distância real + ruído gaussiano pequeno
    rng = np.random.default_rng(0)
    distancias = [
        np.hypot(robo_real[0] - ax, robo_real[1] - ay) + rng.normal(0, 0.02)
        for ax, ay in ancoras
    ]

    x_est, y_est = trilatera(ancoras, distancias)

    print("Posição real:     (%.3f, %.3f)" % robo_real)
    print("Posição estimada: (%.3f, %.3f)" % (x_est, y_est))
    print("Erro:             %.3f m" % np.hypot(x_est - robo_real[0], y_est - robo_real[1]))
