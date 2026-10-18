#!/usr/bin/env bash
# rodar_experimentos_ruido.sh — roda em sequência os casos de ruído da
# tabela do README ("Casos de ruído para avaliação"), cada um numa
# simulação isolada, e separa o .bag e o .csv de cada rodada em
# output/<localizacao>/<caso>/ — separado por método de estimação
# (robot_localization/ekf_uwb/trilateracao) pra rodar o script várias
# vezes (uma por método) sem uma rodada sobrescrever a anterior.
#
# Para cada caso:
#   1) sobe bringup.launch headless (sem gui/rviz/teleop) com os
#      parâmetros do caso e gravação de bag ligada (gravar:=true);
#   2) espera /odometry/filtered aparecer (localização pronta);
#   3) roda visita_checkpoints.py em primeiro plano, com o MESMO
#      n_ancoras do caso (senão a rota não bate com as âncoras
#      spawnadas) — o nó termina sozinho ao concluir o percurso, o que
#      serve de condição de parada automática; um `timeout` evita travar
#      o lote inteiro se o robô ficar preso;
#   4) derruba o bringup (Gazebo, localização, rosbag record);
#   5) move o .bag mais recente de bags/ para
#      output/<localizacao>/<caso>/percurso.bag e gera
#      output/<localizacao>/<caso>/saida.csv via bag_para_csv.py.
#
# Uso (com o workspace catkin já sourced):
#   bash scripts/rodar_experimentos_ruido.sh [localizacao]
#     localizacao: robot_localization | ekf_uwb | ekf_simples | trilateracao (padrão: ekf_uwb)
#
# Variáveis de ambiente opcionais:
#   GUI=true          mostra o gzclient (padrão: headless)
#   RVIZ=true         mostra o RViz (padrão: desligado)
#   TIMEOUT_PERCURSO  segundos máx. de espera pelo percurso (padrão: 300)
#   ORIGEM_X/ORIGEM_Y spawn do robô no Gazebo (padrão: 0.0 / 0.0)

set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAGS_DIR="$PKG_DIR/bags"
LOCALIZACAO="${1:-ekf_uwb}"
SAIDA_DIR="$PKG_DIR/output/$LOCALIZACAO"
GUI="${GUI:-false}"
RVIZ="${RVIZ:-false}"
TIMEOUT_PERCURSO="${TIMEOUT_PERCURSO:-300}"
ORIGEM_X="${ORIGEM_X:-0.0}"  # spawn do robô no Gazebo (x/y) — repassado
ORIGEM_Y="${ORIGEM_Y:-0.0}"  # como origem_x/origem_y pra localização

# nome;n_ancoras;mu_rodas;gaussian_noise_imu
# (mesmos valores da tabela "Casos de ruído para avaliação" do README).
# sigma_range_extra e sigma_angle_extra ficam de fora — sempre no padrão
# do bringup.launch (0.0, sem ruído extra de UWB além do que o plugin já
# injeta nativamente).
CASOS=(
  "default;10;0.10;0.01"
  "simples;6;0.6;0.01"
  "medio;8;0.02;0.02"
  "elevado;10;0.003;0.03"
)

mkdir -p "$BAGS_DIR" "$SAIDA_DIR"

esperar_topico() {
    local topico="$1" tentativas="${2:-60}"
    for ((i = 0; i < tentativas; i++)); do
        if rostopic list 2>/dev/null | grep -qx "$topico"; then
            return 0
        fi
        sleep 1
    done
    echo "ERRO: tópico $topico não apareceu em ${tentativas}s" >&2
    return 1
}

for caso in "${CASOS[@]}"; do
    IFS=';' read -r nome n_ancoras mu_rodas gaussian_noise_imu <<< "$caso"

    echo "=== Caso '$nome' (n_ancoras=$n_ancoras, mu_rodas=$mu_rodas, gaussian_noise_imu=$gaussian_noise_imu) ==="

    caso_dir="$SAIDA_DIR/$nome"
    mkdir -p "$caso_dir"

    roslaunch projeto_agrobot_uwb bringup.launch \
        localizacao:="$LOCALIZACAO" gui:="$GUI" rviz:="$RVIZ" teleop:=false gravar:=true \
        x:="$ORIGEM_X" y:="$ORIGEM_Y" \
        n_ancoras:="$n_ancoras" mu_rodas:="$mu_rodas" gaussian_noise_imu:="$gaussian_noise_imu" \
        > "$caso_dir/bringup.log" 2>&1 &
    bringup_pid=$!

    echo "Aguardando localização subir..."
    if ! esperar_topico "/odometry/filtered"; then
        echo "Abortando caso '$nome' — ver $caso_dir/bringup.log" >&2
        kill -INT "$bringup_pid" 2>/dev/null || true
        wait "$bringup_pid" 2>/dev/null || true
        continue
    fi
    sleep 3  # margem extra pro Gazebo estabilizar a física antes do percurso começar

    echo "Rodando percurso autônomo (visita_checkpoints.py, timeout ${TIMEOUT_PERCURSO}s)..."
    set +e
    timeout "$TIMEOUT_PERCURSO" rosrun projeto_agrobot_uwb visita_checkpoints.py \
        _n_ancoras:="$n_ancoras" \
        > "$caso_dir/visita_checkpoints.log" 2>&1
    percurso_status=$?
    set -e
    if [ "$percurso_status" -eq 124 ]; then
        echo "AVISO: percurso do caso '$nome' não terminou em ${TIMEOUT_PERCURSO}s — usando o bag parcial" >&2
    elif [ "$percurso_status" -ne 0 ]; then
        echo "AVISO: visita_checkpoints.py saiu com código $percurso_status no caso '$nome' — ver $caso_dir/visita_checkpoints.log" >&2
    fi

    echo "Encerrando bringup..."
    kill -INT "$bringup_pid" 2>/dev/null || true
    wait "$bringup_pid" 2>/dev/null || true
    sleep 3  # dá tempo do rosbag record fechar o arquivo (.bag.active -> .bag) e do gzserver liberar a porta

    bag_mais_recente="$(ls -t "$BAGS_DIR"/percurso_*.bag 2>/dev/null | head -n1)"
    if [ -z "$bag_mais_recente" ]; then
        echo "ERRO: nenhum .bag encontrado em $BAGS_DIR para o caso '$nome' — pulando geração do CSV" >&2
        continue
    fi
    mv "$bag_mais_recente" "$caso_dir/percurso.bag"

    echo "Gerando CSV..."
    rosrun projeto_agrobot_uwb bag_para_csv.py "$caso_dir/percurso.bag" "$caso_dir/saida.csv"

    echo "Caso '$nome' concluído -> $caso_dir/{percurso.bag,saida.csv}"
    echo
done

echo "Todos os casos processados. Resultados em $SAIDA_DIR/{default,simples,medio,elevado}/"
echo "(rode de novo com outro método de localização pra comparar — cada um cai numa pasta output/<localizacao>/ separada)"
