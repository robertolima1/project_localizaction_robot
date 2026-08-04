#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
layout_ancoras.py — fonte única do layout das âncoras UWB (posições) e das
plantas correspondentes, dado só o número de âncoras.

Generaliza o layout original (10 âncoras: duas fileiras de 5, y=-0.5 e
y=+0.5, x de -0.8 a 0.8 espaçado 0.4 m) para qualquer N: divide N entre as
duas fileiras (a fileira de y_fileiras[0] fica com o excedente quando N é
ímpar) e espalha os X de cada fileira uniformemente entre x_min e x_max —
com N=10 reproduz exatamente o layout antigo, id a id.

Usado por três lugares que hoje precisavam ser editados manualmente em
sincronia: scripts/spawn_campo.py (spawna plantas+âncoras no Gazebo),
scripts/ekf_localizacao_uwb.py / scripts/localizacao_trilateracao.py
(ANCORAS_PADRAO, mapa de referência absoluta) e scripts/identity.py.
"""

N_ANCORAS_PADRAO = 10


def gerar_ancoras(n, x_min=-0.8, x_max=0.8, y_fileiras=(-0.5, 0.5)):
    """Retorna [{"id": int, "x": float, "y": float}, ...], n itens.

    n=3 é o mínimo aceito com sentido físico (trilateração 2D) — abaixo
    disso o resultado ainda é gerado, mas fica degenerado (n=1: uma
    âncora só; n=2: duas âncoras, uma por fileira)."""
    if n < 1:
        raise ValueError("n precisa ser >= 1 (recebido: %r)" % (n,))

    n_fileira_1 = -(-n // 2)  # ceil(n/2): fileira 1 fica com o excedente ímpar
    n_fileira_2 = n - n_fileira_1
    contagens = (n_fileira_1, n_fileira_2)

    ancoras = []
    id_ = 0
    for y, count in zip(y_fileiras, contagens):
        if count == 0:
            continue
        if count == 1:
            xs = [(x_min + x_max) / 2.0]
        else:
            passo = (x_max - x_min) / (count - 1)
            xs = [x_min + i * passo for i in range(count)]
        for x in xs:
            ancoras.append({"id": id_, "x": round(x, 6), "y": y})
            id_ += 1
    return ancoras
