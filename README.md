# projeto_agrobot_uwb

Pacote ROS 1 Noetic derivado do **TurtleBot3 burger**, com localização por
EKF — `robot_localization` (dead reckoning, IMU + odometria de rodas) ou
um EKF próprio por correspondência conhecida usando âncoras UWB como
referência absoluta, alternáveis por parâmetro (ver seção "Localização")
—, mundo Gazebo próprio (fileiras de plantio em escala reduzida) e
teleoperação manual por teclado. Sensoriamento: **IMU + odometria de
rodas + UWB** — sem LiDAR, câmera ou GPS. Controle: **100% manual**, sem
waypoints nem `move_base`.

Ambiente testado: Ubuntu 20.04 (WSL2/WSLg), ROS Noetic, catkin, Gazebo 11.

## Pacotes usados para montar o ambiente

O projeto não reimplementa nada que o ROS/TurtleBot3 já oferecem prontos —
ele **reaproveita** pacotes existentes e só adiciona a camada própria (EKF,
mundo, teleop). Cada dependência abaixo tem um papel específico:

### Instalados via `apt` (binários do ROS Noetic)

| Pacote | Para que serve aqui |
|---|---|
| `ros-noetic-turtlebot3` | Meta-pacote do TurtleBot3 — puxa as dependências principais da família (bringup, description, teleop, etc.). |
| `ros-noetic-turtlebot3-msgs` | Mensagens customizadas do TurtleBot3 (sensores/atuadores específicos da plataforma real). |
| `ros-noetic-turtlebot3-simulations` | Meta-pacote de simulação — traz junto o `turtlebot3_gazebo`, com mundos de exemplo e plugins Gazebo já testados para o burger. |
| `ros-noetic-turtlebot3-description` | **A peça mais importante**: contém os arquivos `.urdf.xacro`/`.gazebo.xacro` originais do burger (malhas, dimensões, plugins) que `agrobot.urdf.xacro`/`agrobot.gazebo.xacro` copiam e corrigem. Sem ele, `urdf/agrobot.urdf.xacro` não encontra `package://turtlebot3_description/meshes/...`. |
| `ros-noetic-robot-localization` | O pacote do EKF (`ekf_localization_node`) que funde IMU + odometria de rodas em `/odometry/filtered`. |
| `ros-noetic-tf2-tools` | Só a ferramenta `view_frames.py`, usada para inspecionar/validar a árvore de transformadas (TF) e confirmar que não há frames duplicados. |
| `xterm` | Terminal X usado pelo `teleop.launch` (`launch-prefix="xterm -e"`) para abrir uma janela própria e interativa para o teleop — ver explicação na seção "Como rodar". |

### Dependências do próprio pacote (declaradas no `package.xml`, já vêm com o `ros-noetic-desktop-full` ou com o Noetic base + as acima)

| Pacote | Para que serve aqui |
|---|---|
| `rospy` | Nó de teleop (`teleop_teclado.py`) é Python puro. |
| `xacro` | Processa os arquivos `.xacro` (URDF com macros) em URDF puro antes do spawn no Gazebo. |
| `gazebo_ros` | Ponte ROS↔Gazebo: sobe o simulador (`empty_world.launch`) e spawna o modelo (`spawn_model`). |
| `robot_localization` | (repetido acima, é dependência de build e de execução). |
| `tf2_ros` | Infra de transformadas usada pelo `robot_state_publisher` e pelo EKF. |
| `sensor_msgs` | Tipo de mensagem da IMU (`sensor_msgs/Imu`). |
| `nav_msgs` | Tipo de mensagem da odometria (`nav_msgs/Odometry`), tanto `/odom` quanto `/odometry/filtered`. |
| `geometry_msgs` | Tipo de mensagem do comando de velocidade (`geometry_msgs/Twist` em `/cmd_vel`). |
| `std_msgs` | Tipo de mensagem do status do e-stop (`std_msgs/Bool` em `/e_stop`). |
| `robot_state_publisher` | Publica as TFs fixas do robô (rodas, IMU, chassi) a partir do `robot_description` e do `/joint_states`. |
| `rviz` | Visualização 3D (TF, modelo do robô, as duas trilhas de odometria). |

### Simulador

- **Gazebo 11** (`gazebo11`, pacote do sistema, não do ROS) — o motor de física e o simulador propriamente dito. `gazebo_ros` só faz a ponte com ele.

## Pré-requisitos e build

```bash
sudo apt update && sudo apt install -y \
  ros-noetic-turtlebot3 \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3-simulations \
  ros-noetic-turtlebot3-description \
  ros-noetic-robot-localization \
  ros-noetic-tf2-tools \
  xterm
```

Se algum pacote `ros-noetic-turtlebot3-*` não existir como binário para a
sua distro, clone a branch `noetic-devel` de `ROBOTIS-GIT/turtlebot3` e de
`turtlebot3_simulations` dentro de `~/catkin_ws/src/` e rode
`rosdep install --from-paths src --ignore-src -r -y` antes de compilar.

```bash
cd ~/catkin_ws
catkin_make        # ou: catkin build, se você usa catkin_tools
source devel/setup.bash
```

`source devel/setup.bash` precisa ser repetido em **todo terminal novo**
que for usar `roslaunch`/`rosrun` deste pacote (ou coloque essa linha no
seu `~/.bashrc`).

## Como rodar a simulação — passo a passo

**1. Compile e carregue o ambiente** (se ainda não fez):

```bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

**2. Suba tudo com um único comando:**

```bash
roslaunch projeto_agrobot_uwb bringup.launch
```

Isso, na ordem, faz o `bringup.launch` (ver `launch/bringup.launch`):

1. Abre o **Gazebo** (via `launch/gazebo.launch`) carregando
   `worlds/campo_agricola.world` e "spawna" o robô a partir do xacro local
   (`urdf/agrobot.urdf.xacro`) — você deve ver a janela do Gazebo com o
   chão marrom, duas fileiras de "plantas" (cilindros verdes) e o robô
   entre elas.
2. Sobe o **`robot_state_publisher`** e o **RViz** (via
   `launch/display.launch`), com a TF, o modelo do robô e duas trilhas de
   odometria já configuradas (vermelha = `/odom` bruta, verde =
   `/odometry/filtered`, ambas com rastro de até 500 poses).
3. Sobe o **EKF** (via `launch/localizacao.launch`), que começa a publicar
   `/odometry/filtered` e a TF `odom → base_footprint`.
4. Abre uma **janela xterm separada** (via `launch/teleop.launch`) rodando
   `scripts/teleop_teclado.py` — é **nessa janela xterm**, não no terminal
   onde você rodou o `roslaunch`, que as teclas de movimento funcionam.
   Clique na janela xterm para dar foco a ela antes de digitar.

**3. Dirija o robô** na janela xterm que abriu (tabela completa de teclas
mais abaixo): `w`/`s` para frente/ré, `a`/`d` para girar, espaço para
parar tudo na hora.

**4. Para encerrar**, feche a janela xterm (ou `Ctrl+C` nela) e depois
`Ctrl+C` no terminal onde rodou o `roslaunch` — isso derruba Gazebo, RViz,
EKF e `robot_state_publisher` juntos.

### Variações úteis

```bash
# Sem interface gráfica nenhuma (útil se o WSL2 não tiver X server/WSLg
# configurado, ou para rodar mais leve/mais rápido que tempo real)
roslaunch projeto_agrobot_uwb bringup.launch gui:=false rviz:=false

# Só a simulação + EKF, sem teleop automático — você abre o teleop
# manualmente num terminal à parte (útil se preferir não depender do
# xterm, ou se estiver rodando de um ambiente sem X server)
roslaunch projeto_agrobot_uwb bringup.launch teleop:=false
# em outro terminal:
source ~/catkin_ws/devel/setup.bash
rosrun projeto_agrobot_uwb teleop_teclado.py

# Spawnar o robô em outra posição/orientação inicial
roslaunch projeto_agrobot_uwb bringup.launch x:=1.0 y:=0.5 yaw:=1.57
```

### Argumentos do `bringup.launch`

| Argumento | Padrão | O que faz |
|---|---|---|
| `mundo` | `worlds/campo_agricola.world` deste pacote | Caminho do mundo `.world` a carregar no Gazebo. |
| `x`, `y`, `yaw` | `0.0`, `0.0`, `0.0` | Pose inicial do robô ao ser spawnado. |
| `gui` | `true` | Mostra ou não a janela do Gazebo (`gzclient`); o servidor de física (`gzserver`) sobe de qualquer jeito. |
| `rviz` | `true` | Abre ou não o RViz. |
| `teleop` | `true` | Sobe ou não a janela xterm com o teleop automaticamente. |
| `gravar` | `true` | Grava ou não o percurso em `bags/percurso_<timestamp>.bag` (ver seção "Gravando um percurso"). |
| `localizacao` | `robot_localization` | Qual EKF usa a TF `odom → base_footprint`: `robot_localization` ou `ekf_uwb` (ver seção "Localização"). |

### Verificando se está tudo certo (em outro terminal, com o `bringup.launch` já rodando)

```bash
source ~/catkin_ws/devel/setup.bash

rostopic list                        # deve ter /odom, /imu, /cmd_vel — sem /scan, sem /camera/*
rostopic hz /odom                    # deve ficar estável perto de 30 Hz
rostopic hz /imu                     # deve ficar estável perto de 100 Hz
rosrun tf2_tools view_frames.py      # gera frames.pdf; confira que só o
                                      # ekf_localization_node publica odom -> base_footprint
roswtf                               # não deve reportar nenhum ERRO (avisos sobre
                                      # tópicos do Gazebo sem assinante são normais)
```

### Solução de problemas comuns

- **A janela xterm do teleop não abre / erro sobre `DISPLAY`**: seu
  ambiente não tem servidor X acessível. Rode com `teleop:=false` e use
  `rosrun projeto_agrobot_uwb teleop_teclado.py` num terminal comum (não
  precisa de X, só de um terminal interativo de verdade).
- **Teclado não responde dentro da janela xterm**: clique na janela para
  dar foco a ela — o Gazebo/RViz não roubam o foco do teclado sozinhos.
- **`package 'projeto_agrobot_uwb' not found`**: esqueceu de rodar
  `source devel/setup.bash` nesse terminal depois do build.
- **Erro de mesh/`package://turtlebot3_description/...` não encontrado**:
  falta instalar `ros-noetic-turtlebot3-description` (ver seção de
  pré-requisitos).

### Teclas do teleop

| Tecla | Ação |
|---|---|
| `w` / `s` | frente / ré |
| `a` / `d` | girar à esquerda / direita |
| `q` / `e` | curva combinada (frente + giro) |
| `+` / `-` | aumentar / reduzir a escala de velocidade em 10 pontos percentuais |
| `x` | parar suavemente (rampa até zero) |
| **espaço** | **parada de emergência imediata** (zera o Twist e trava até `r`) |
| `r` | rearmar após e-stop |
| `Ctrl+C` | encerrar publicando Twist zerado |

## Tópicos publicados / assinados

| Tópico | Tipo | Direção | Nó |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | publica | `teleop_teclado.py` |
| `/e_stop` | `std_msgs/Bool` | publica (status observável) | `teleop_teclado.py` |
| `/odom` | `nav_msgs/Odometry` | publica | plugin `libgazebo_ros_diff_drive` |
| `/imu` | `sensor_msgs/Imu` | publica | plugin `libgazebo_ros_imu` |
| `/odometry/filtered` | `nav_msgs/Odometry` | publica | `ekf_localization_node` **ou** `ekf_localizacao_uwb` (ver `localizacao`) |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | publica | Gazebo (pose real, só para comparação manual) |
| `/gtec/toa/ranging` | `gtec_msgs/Ranging` | publica | plugin `libgtec_uwb_plugin` (externo) |
| `/gtec/toa/ranging` | `gtec_msgs/Ranging` | assina, só com `localizacao:=ekf_uwb` | `ekf_localizacao_uwb.py` |
| `/gtec/toa/anchors` | `visualization_msgs/MarkerArray` | publica | plugin `libgtec_uwb_plugin` (externo) |

## Localização

`launch/localizacao.launch` publica a TF `odom → base_footprint` e o
tópico `/odometry/filtered`, através de **um entre dois métodos
alternativos** (nunca os dois ao mesmo tempo — disputariam a mesma TF),
escolhido pelo argumento `localizacao` do `bringup.launch`:

| `localizacao` | Nó | Fontes fundidas | Referência absoluta? |
|---|---|---|---|
| `robot_localization` (padrão) | `ekf_localization_node` | `/odom` + `/imu` | Não — dead reckoning puro (ver "Limitações do dead reckoning"). |
| `ekf_uwb` | `scripts/ekf_localizacao_uwb.py` | `/odom` + `/imu` + `/gtec/toa/ranging` | Sim — as âncoras UWB fecham o laço. |

```bash
# EKF de dead reckoning (padrão)
roslaunch projeto_agrobot_uwb bringup.launch

# EKF com correspondência conhecida (Thrun, Tabela 7.2), usando as âncoras
# UWB como mapa de referência absoluta
roslaunch projeto_agrobot_uwb bringup.launch localizacao:=ekf_uwb
```

`ekf_uwb` é a implementação da Tabela 7.2 de *Probabilistic Robotics*
(Thrun, Burgard & Fox) com correspondência conhecida: a correspondência
`c^i` vem pronta no `anchorId` de cada `gtec_msgs/Ranging`, então não há
dimensão de assinatura a resolver. O controle `u_t = (v, w)` usa o mesmo
par de fontes do `robot_localization` (`v` de `/odom`, `w` de `/imu`); a
correção usa `(r, phi)` — distância e ângulo relativo de cada âncora —
com o mapa de âncoras fixo em `config/ekf_uwb.yaml` (posições copiadas do
`.world`). Detalhes e as três notas de implementação (projeção do range
3D para 2D, saturação de `w` perto de zero, e a suposição de que `odom`
coincide com o frame do mundo no spawn) estão comentados no próprio
script.

Parâmetros de `ekf_uwb.yaml` que provavelmente valem a pena calibrar:
`sigma_r`/`sigma_phi` (ruído de medida) e `r_diag` (ruído de processo).

## Sensor UWB

O mundo tem 10 âncoras UWB fixas (`uwb_anchor0..9`, uma por planta,
encostada no lado de fora de cada uma, a 0,15 m de altura — mesma altura
do centro da planta) e o robô
tem uma tag (`uwb_tag_link`) com o plugin
[`uwb_gazebosensorplugins`](https://github.com/AUVSL/UWB-Gazebo-Plugin)
(externo — clonado à parte, não faz parte deste pacote nem do repositório).

- **Participa da localização só quando `localizacao:=ekf_uwb`** (ver seção
  "Localização" acima). Com o padrão (`robot_localization`), é só um
  sensor simulado ativo, publicando ranging real (com modelo de LOS/NLOS,
  inclusive reflexão em obstáculos) para inspeção manual
  (`rostopic echo /gtec/toa/ranging`), sem entrar em nenhum EKF.
- **Dependências externas** (não vêm com este pacote): clone em
  `~/catkin_ws/src/`:
  ```bash
  git clone https://github.com/AUVSL/rosmsgs.git gtec_msgs
  git clone https://github.com/AUVSL/UWB-Gazebo-Plugin.git uwb_gazebosensorplugins
  cd ~/catkin_ws && catkin_make
  ```
- **Detalhe de implementação**: o `uwb_tag_link` é um link vazio (sem
  geometria) ligado por joint fixa, e o Gazebo funde esse tipo de link no
  `base_link` ao converter URDF→SDF — o que faz o plugin usar a pose do
  robô inteiro como origem do raio de UWB, bem dentro da própria colisão
  do chassi, bloqueando todo raio de linha de visão. Corrigido com
  `tag_z_offset=0.3` no plugin (ver comentário em `agrobot.gazebo.xacro`),
  que levanta essa origem para fora do robô sem precisar resolver a fusão
  do link.

## Árvore TF

```
odom -> base_footprint   (publicado só pelo método de localização ativo:
                          ekf_localization_node OU ekf_localizacao_uwb,
                          nunca os dois — ver seção "Localização")
base_footprint -> base_link
base_link -> wheel_left_link
base_link -> wheel_right_link
base_link -> caster_back_link
base_link -> imu_link
base_link -> base_scan     (link mantido por massa/inércia; sem sensor de LiDAR)
```

Validar com: `rosrun tf2_tools view_frames.py`.

## Gravando um percurso para analisar a deriva

O `bringup.launch` já grava automaticamente, sem comando manual: um nó
`rosbag record` sobe junto com o resto e salva cada percurso em
`bags/percurso_<timestamp>.bag`, com os tópicos `/odom`, `/imu`,
`/odometry/filtered` e `/cmd_vel`.

```bash
# comportamento padrão: grava
roslaunch projeto_agrobot_uwb bringup.launch

# desativar a gravação (ex.: teste rápido, sem interesse em analisar depois)
roslaunch projeto_agrobot_uwb bringup.launch gravar:=false
```

Cada execução do launch gera um `.bag` novo (o `rosbag record` nunca
sobrescreve o anterior). Para inspecionar depois:

```bash
rosbag info bags/percurso_<timestamp>.bag
rosbag play bags/percurso_<timestamp>.bag
```

Os `.bag` não são versionados no git (ver `.gitignore`) — geralmente são
pesados demais para isso.

### Extraindo o percurso para CSV

`scripts/bag_para_csv.py` lê um ou mais `.bag` e escreve um único CSV em
**formato largo** — uma linha por instante de tempo, com colunas
separadas por fonte (`odom_x, odom_y, odom_yaw, filtrada_x, filtrada_y,
filtrada_yaw, ground_truth_x, ground_truth_y, ground_truth_yaw`, `yaw` já
convertido do quaternion), pronto para comparar as fontes lado a lado
numa planilha ou no pandas sem precisar pivotar depois:

```bash
rosrun projeto_agrobot_uwb bag_para_csv.py bags/percurso_XXXX.bag saida.csv
```

As três fontes publicam em instantes diferentes e raramente batem o
timestamp exato, então o script arredonda `t` para o múltiplo mais
próximo de `--resolucao` (padrão `0.05` s = 20 Hz) e agrupa nesse
intervalo — a leitura mais recente de cada fonte dentro do intervalo é a
que fica na linha. Uma fonte sem leitura naquele intervalo sai com as
colunas vazias, não com erro. Ajuste com `--resolucao 0.1`, por exemplo,
se quiser linhas mais "cheias" ao custo de granularidade temporal.

Por padrão o `bringup.launch` **não** grava `/gazebo/model_states`
(ground truth) — só `/odom`, `/imu`, `/odometry/filtered`, `/cmd_vel`. Sem
esse tópico, as colunas `ground_truth_*` saem vazias em todas as linhas,
sem erro nem aviso. Para incluir a comparação com o ground truth,
grave-o à parte, num segundo terminal, enquanto dirige:

```bash
rosbag record -O bags/ground_truth_XXXX.bag /gazebo/model_states
```

e passe os dois arquivos ao script — ele aceita vários `.bag` de uma vez
e alinha o tempo pelo início mais antigo entre eles:

```bash
rosrun projeto_agrobot_uwb bag_para_csv.py bags/percurso_XXXX.bag bags/ground_truth_XXXX.bag saida.csv
```
