#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
perfil_recursos.py — amostra CPU e memória (RSS) do processo de um node ROS
já em execução, periodicamente, e grava num CSV (t, cpu_percent, mem_mb).

O ROS não expõe isso por node (`rosnode`/`rostopic` não têm essa métrica).
Este script acha o PID do node via XML-RPC: todo node ROS serve um método
`getPid` na própria URI (a mesma usada internamente por `rosnode kill`),
então a busca é pelo grafo ROS (nome do node), não por casar texto de
`ps aux` contra o nome do script — mais confiável (funciona igual em devel
e install space, não confunde processos com nome parecido).

Roda até Ctrl+C, até --duracao esgotar, ou até o node monitorado terminar
(detectado por psutil.NoSuchProcess) — o que vier primeiro. Cada amostra é
gravada e o arquivo é *flusheado* na hora, então mesmo interrompido no meio
o CSV parcial já gravado fica utilizável (não junta tudo num buffer só pra
escrever no final, como bag_para_csv.py faz com um .bag fechado — aqui a
fonte é um processo vivo, de duração desconhecida).

Mede só o processo do node em si (RSS + %CPU dele), não filhos que ele
eventualmente crie (nenhum dos nodes deste projeto cria subprocesso).

Uso:
    rosrun projeto_agrobot_uwb perfil_recursos.py /ekf_localizacao_uwb perfil_ekf.csv
    rosrun projeto_agrobot_uwb perfil_recursos.py /localizacao_trilateracao perfil.csv --intervalo 0.2 --duracao 60
"""

import argparse
import csv
import sys
import time
import xmlrpc.client

import psutil
import rosgraph

INTERVALO_PADRAO = 0.5  # s


def resolve_pid(node_name, caller_id="/perfil_recursos"):
    """Acha o PID do processo do node via XML-RPC (mesmo caminho que
    `rosnode kill` usa). Levanta RuntimeError com mensagem pronta pra
    imprimir se o node não existir ou não responder."""
    master = rosgraph.Master(caller_id)
    try:
        uri = master.lookupNode(node_name)
    except rosgraph.MasterError:
        raise RuntimeError("node %r não encontrado no ROS master "
                           "(rosnode list mostra os disponíveis)" % node_name)

    codigo, msg, pid = xmlrpc.client.ServerProxy(uri).getPid(caller_id)
    if codigo != 1:
        raise RuntimeError("node %r não respondeu getPid: %s" % (node_name, msg))
    return pid


def amostra(processo):
    """(cpu_percent desde a última chamada, mem_rss em MB)."""
    cpu = processo.cpu_percent(interval=None)
    mem_mb = processo.memory_info().rss / (1024.0 * 1024.0)
    return cpu, mem_mb


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("node", help="nome do node no grafo ROS (ex.: /ekf_localizacao_uwb"
                                 " — ver `rosnode list`)")
    ap.add_argument("csv_saida", help="caminho do .csv a gerar")
    ap.add_argument("--intervalo", type=float, default=INTERVALO_PADRAO,
                    help="período de amostragem em segundos (padrão: %s)"
                         % INTERVALO_PADRAO)
    ap.add_argument("--duracao", type=float, default=None,
                    help="para automaticamente após N segundos "
                         "(padrão: sem limite, roda até Ctrl+C ou o node terminar)")
    args = ap.parse_args()

    node_name = args.node if args.node.startswith("/") else "/" + args.node

    try:
        pid = resolve_pid(node_name)
        processo = psutil.Process(pid)
    except (RuntimeError, psutil.NoSuchProcess) as e:
        print("erro: %s" % e, file=sys.stderr)
        sys.exit(1)

    print("perfil_recursos: monitorando %s (pid %d) a cada %.2fs -> %s"
         % (node_name, pid, args.intervalo, args.csv_saida))

    processo.cpu_percent(interval=None)  # primeira chamada só "arma" a medição

    inicio = time.monotonic()
    n_amostras = 0
    with open(args.csv_saida, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "cpu_percent", "mem_mb"])
        try:
            while True:
                time.sleep(args.intervalo)
                t = time.monotonic() - inicio
                if args.duracao is not None and t >= args.duracao:
                    break

                cpu, mem_mb = amostra(processo)
                w.writerow([round(t, 3), cpu, round(mem_mb, 3)])
                f.flush()
                n_amostras += 1
        except KeyboardInterrupt:
            pass
        except psutil.NoSuchProcess:
            print("perfil_recursos: node terminou, encerrando.")

    print("%d amostras escritas em %s" % (n_amostras, args.csv_saida))


if __name__ == "__main__":
    main()
