#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
layout_ancoras.py — fonte única do layout das âncoras UWB (posições) e das
plantas correspondentes, dado só o número de âncoras.

Generaliza o layout original (10 âncoras: duas fileiras de 5, y=-0.5 e
y=+0.5, x de -0.8 a 0.8 espaçado 0.4 m) para qualquer N: divide N entre as
duas fileiras (a fileira de y_fileiras[0] fica com o excedente quando N é
ímpar) e espalha os X de cada fileira uniformemente entre x_min e x_max —
com N=10, gerar_plantas() reproduz exatamente o layout antigo, id a id.

Plantas e âncoras NÃO ficam mais na mesma posição: gerar_ancoras() parte
da mesma grade de gerar_plantas() e desloca cada âncora offset_corredor
metros em direção ao corredor central (y=0) — a âncora fica à frente da
planta, do ponto de vista de quem anda pelo corredor onde o robô passa,
em vez de logo acima dela. Isso também evita o raio de LOS do UwbPlugin
ficar raspando na vegetação/no próprio corpo da planta perto da âncora
(ver nota em scripts/spawn_campo.py sobre o bug de NLOS_S que isso
causava quando âncora e planta coincidiam exatamente).

Usado por quatro lugares que antes precisavam ser editados manualmente em
sincronia: scripts/spawn_campo.py (spawna plantas+âncoras no Gazebo),
scripts/ekf_localizacao_uwb.py / scripts/localizacao_trilateracao.py
(ANCORAS_PADRAO, mapa de referência absoluta) e scripts/identity.py.
"""

N_ANCORAS_PADRAO = 10


def _grade(n, x_min, x_max, y_fileiras):
    """Grade base: [{"id": int, "x": float, "y": float}, ...], n itens,
    duas fileiras em y_fileiras, x espalhado uniformemente entre x_min e
    x_max em cada uma (excedente ímpar vai pra fileira y_fileiras[0])."""
    if n < 1:
        raise ValueError("n precisa ser >= 1 (recebido: %r)" % (n,))

    n_fileira_1 = -(-n // 2)  # ceil(n/2): fileira 1 fica com o excedente ímpar
    n_fileira_2 = n - n_fileira_1
    contagens = (n_fileira_1, n_fileira_2)

    posicoes = []
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
            posicoes.append({"id": id_, "x": round(x, 6), "y": y})
            id_ += 1
    return posicoes


def gerar_plantas(n, x_min=-0.8, x_max=0.8, y_fileiras=(-0.5, 0.5)):
    """Posições das plantas — grade base, sem deslocamento algum.

    n=3 é o mínimo aceito com sentido físico (trilateração 2D) — abaixo
    disso o resultado ainda é gerado, mas fica degenerado (n=1: uma
    planta só; n=2: duas plantas, uma por fileira)."""
    return _grade(n, x_min, x_max, y_fileiras)


def gerar_ancoras(n, x_min=-0.8, x_max=0.8, y_fileiras=(-0.5, 0.5),
                   offset_corredor=0.15):
    """Posições das âncoras — mesma grade de gerar_plantas(), cada uma
    deslocada offset_corredor metros em direção ao corredor central
    (y=0): fileiras com y>0 descem, fileiras com y<0 sobem. Retorna
    [{"id": int, "x": float, "y": float}, ...], n itens."""
    ancoras = _grade(n, x_min, x_max, y_fileiras)
    for a in ancoras:
        if a["y"] > 0:
            a["y"] = round(a["y"] - offset_corredor, 6)
        elif a["y"] < 0:
            a["y"] = round(a["y"] + offset_corredor, 6)
    return ancoras
