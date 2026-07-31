#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Teleoperação por teclado do agrobot — único nó de controle do projeto."""

import sys
import tty
import select
import termios
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


# Mapa de tecla -> "direção" desejada. Cada direção define para onde o alvo
# de velocidade aponta; quem limita o quanto ele se move é a escala (+/-) e
# quem limita quão rápido a velocidade REAL alcança o alvo é a rampa de
# aceleração — ver GerenciadorTeleop.calcula_alvo() e aplica_rampa().
DIRECOES = {
    'w': 'frente',
    's': 're',
    'a': 'esquerda',
    'd': 'direita',
    'q': 'curva_esq',
    'e': 'curva_dir',
    'x': 'parado',
}

# Fração do máximo usada nas curvas combinadas (q/e): frente + giro ao
# mesmo tempo, mas cada componente um pouco reduzido para a curva não ficar
# violenta.
FRACAO_CURVA = 0.5


class LeitorTeclado:
    """Lê o teclado em modo raw (sem esperar Enter) e restaura o terminal
    no final, mesmo se algo der errado no meio do caminho."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.config_original = termios.tcgetattr(self.fd)

    def le_tecla(self, timeout):
        pronto, _, _ = select.select([sys.stdin], [], [], timeout)
        if pronto:
            return sys.stdin.read(1)
        return ''

    def entra_modo_raw(self):
        tty.setraw(self.fd)

    def restaura(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.config_original)


class GerenciadorTeleop:
    def __init__(self):
        self.vel_linear_max = rospy.get_param('~vel_linear_max', 0.22)
        self.vel_angular_max = rospy.get_param('~vel_angular_max', 2.84)
        self.aceleracao = rospy.get_param('~aceleracao', 1.0)   # m/s^2
        self.taxa = rospy.get_param('~taxa', 20.0)              # Hz
        self.watchdog_s = rospy.get_param('~watchdog', 0.5)     # s

        # Aceleração angular proporcional à linear, para a rampa "sentir"
        # igual nos dois eixos em vez de travar um e escorregar o outro.
        self.aceleracao_angular = self.aceleracao * (
            self.vel_angular_max / self.vel_linear_max
        )

        self.direcao = 'parado'
        self.escala = 1.0        # 10% a 100% do máximo, ajustada por +/-
        self.travado = False     # e-stop

        self.atual_linear = 0.0
        self.atual_angular = 0.0

        self.ultima_tecla_em = rospy.get_time()

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_estop = rospy.Publisher('/e_stop', Bool, queue_size=1)

    def processa_tecla(self, tecla):
        if tecla == '\x03':  # Ctrl+C em modo raw não gera SIGINT sozinho
            raise KeyboardInterrupt

        self.ultima_tecla_em = rospy.get_time()

        if tecla == ' ':
            self.travado = True
            self.direcao = 'parado'
            return

        if tecla == 'r':
            self.travado = False
            self.direcao = 'parado'
            return

        # Travado pelo e-stop: ignora qualquer tecla de movimento ou de
        # escala até o 'r' — só espaço e 'r' têm efeito aqui.
        if self.travado:
            return

        if tecla in DIRECOES:
            self.direcao = DIRECOES[tecla]
        elif tecla == '+':
            self.escala = min(1.0, self.escala + 0.1)
        elif tecla == '-':
            self.escala = max(0.1, self.escala - 0.1)

    def aplica_watchdog(self):
        if not self.travado and (rospy.get_time() - self.ultima_tecla_em) > self.watchdog_s:
            self.direcao = 'parado'

    def calcula_alvo(self):
        if self.travado or self.direcao == 'parado':
            return 0.0, 0.0

        vlin = self.vel_linear_max * self.escala
        vang = self.vel_angular_max * self.escala

        if self.direcao == 'frente':
            return vlin, 0.0
        if self.direcao == 're':
            return -vlin, 0.0
        if self.direcao == 'esquerda':
            return 0.0, vang
        if self.direcao == 'direita':
            return 0.0, -vang
        if self.direcao == 'curva_esq':
            return vlin * FRACAO_CURVA, vang * FRACAO_CURVA
        if self.direcao == 'curva_dir':
            return vlin * FRACAO_CURVA, -vang * FRACAO_CURVA
        return 0.0, 0.0

    def _rampa(self, atual, alvo, passo_max):
        diferenca = alvo - atual
        if diferenca > passo_max:
            return atual + passo_max
        if diferenca < -passo_max:
            return atual - passo_max
        return alvo

    def atualiza(self, dt):
        self.aplica_watchdog()

        if self.travado:
            # E-stop é imediato, não passa pela rampa.
            self.atual_linear = 0.0
            self.atual_angular = 0.0
        else:
            alvo_linear, alvo_angular = self.calcula_alvo()
            self.atual_linear = self._rampa(
                self.atual_linear, alvo_linear, self.aceleracao * dt
            )
            self.atual_angular = self._rampa(
                self.atual_angular, alvo_angular, self.aceleracao_angular * dt
            )

        # Saturação final: nunca ultrapassar os limites do burger, mesmo
        # que algum cálculo acima erre por causa de arredondamento.
        self.atual_linear = max(-self.vel_linear_max, min(self.vel_linear_max, self.atual_linear))
        self.atual_angular = max(-self.vel_angular_max, min(self.vel_angular_max, self.atual_angular))

    def publica(self):
        msg = Twist()
        msg.linear.x = self.atual_linear
        msg.angular.z = self.atual_angular
        self.pub_cmd_vel.publish(msg)
        self.pub_estop.publish(Bool(data=self.travado))

    def publica_zero(self):
        self.pub_cmd_vel.publish(Twist())

    def painel_status(self):
        estado = 'TRAVADO (espaço) — aperte r para rearmar' if self.travado else 'liberado'
        linha = (
            '\r\x1b[Kv_lin={:+.3f} m/s  v_ang={:+.3f} rad/s  escala={:.0f}%  e-stop: {}'
        ).format(self.atual_linear, self.atual_angular, self.escala * 100, estado)
        sys.stdout.write(linha)
        sys.stdout.flush()


def main():
    rospy.init_node('teleop_teclado')

    leitor = LeitorTeclado()
    gerenciador = GerenciadorTeleop()

    def ao_desligar():
        leitor.restaura()
        gerenciador.publica_zero()
        sys.stdout.write('\n')

    rospy.on_shutdown(ao_desligar)

    print('Teleop do agrobot — w/a/s/d: mover, q/e: curva, +/-: escala, '
          'x: parar suave, ESPAÇO: e-stop, r: rearmar, Ctrl+C: sair\n')

    try:
        leitor.entra_modo_raw()
        taxa_hz = gerenciador.taxa
        dt = 1.0 / taxa_hz

        while not rospy.is_shutdown():
            tecla = leitor.le_tecla(dt)
            if tecla:
                gerenciador.processa_tecla(tecla)

            gerenciador.atualiza(dt)
            gerenciador.publica()
            gerenciador.painel_status()
    except KeyboardInterrupt:
        pass
    finally:
        leitor.restaura()
        gerenciador.publica_zero()


if __name__ == '__main__':
    main()
