# projeto_agrobot_uwb

Pacote ROS 1 Noetic derivado do **TurtleBot3 burger**, com localização por
EKF (`robot_localization`) fundindo IMU + odometria de rodas, mundo Gazebo
próprio (fileiras de plantio em escala reduzida) e teleoperação manual por
teclado. Sensoriamento: **somente IMU + odometria de rodas** — sem LiDAR,
câmera ou GPS. Controle: **100% manual**, sem waypoints nem `move_base`.

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
| `/odometry/filtered` | `nav_msgs/Odometry` | publica | `ekf_localization_node` |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | publica | Gazebo (pose real, só para comparação manual) |
| `/gtec/toa/ranging` | `gtec_msgs/Ranging` | publica | plugin `libgtec_uwb_plugin` (externo) |
| `/gtec/toa/anchors` | `visualization_msgs/MarkerArray` | publica | plugin `libgtec_uwb_plugin` (externo) |

## Sensor UWB (ativo, fora da localização)

O mundo tem 10 âncoras UWB fixas (`uwb_anchor0..9`, uma por planta, a 1.0 m
de altura acima de cada uma) e o robô
tem uma tag (`uwb_tag_link`) com o plugin
[`uwb_gazebosensorplugins`](https://github.com/AUVSL/UWB-Gazebo-Plugin)
(externo — clonado à parte, não faz parte deste pacote nem do repositório).

- **Não participa do EKF nem de nenhuma parte da localização** — é só um
  sensor simulado ativo, publicando ranging real (com modelo de LOS/NLOS,
  inclusive reflexão em obstáculos) para inspeção manual
  (`rostopic echo /gtec/toa/ranging`). Se um dia quiser fundir isso à
  localização, precisaria escrever um nó de trilateração à parte (o plugin
  só entrega distância por âncora, não uma pose pronta).
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
odom -> base_footprint   (publicado só pelo ekf_localization_node)
base_footprint -> base_link
base_link -> wheel_left_link
base_link -> wheel_right_link
base_link -> caster_back_link
base_link -> imu_link
base_link -> base_scan     (link mantido por massa/inércia; sem sensor de LiDAR)
```

Validar com: `rosrun tf2_tools view_frames.py`.

## O que foi alterado em relação ao TurtleBot3 original

Arquivos copiados de `turtlebot3_description` para dentro deste pacote
(`urdf/agrobot.urdf.xacro` e `urdf/agrobot.gazebo.xacro`), com as seguintes
correções no `.gazebo.xacro` (necessárias para o EKF fazer sentido):

- **`publishOdomTF`**: `true` → `false`. Sem isso, o plugin de rodas e o EKF
  publicariam `odom -> base_footprint` ao mesmo tempo, e a TF ficaria
  oscilando entre as duas fontes.
- **`odometrySource`**: `world` → `encoder`. Com `world` o Gazebo entrega a
  pose real (ground truth) em `/odom`, sem deriva — o EKF não teria o que
  corrigir. Com `encoder`, a odometria é integrada das rodas e deriva de
  forma realista.
- **`robotBaseFrame`**: já era `base_footprint` (confirmado — é o mesmo
  frame usado em `base_link_frame` no `ekf_local.yaml`; misturar
  `base_link`/`base_footprint` é a causa nº1 de erro silencioso de offset).
- **Ruído da IMU**: o `gaussianNoise` do plugin `libgazebo_ros_imu.so`
  estava zerado. O bloco `<imu><noise>...` que parecia configurar bias e
  desvio-padrão por eixo **nunca funcionou** — confirmado no header
  `gazebo_ros_imu.h`, que só declara um único `gaussian_noise_`. Esse
  plugin não suporta bias nem valores separados para giro/acelerômetro;
  usamos `gaussianNoise = 0.01` como valor único aplicado aos dois. Como a
  IMU é a única fonte de correção do projeto, esse número define
  diretamente o quanto o EKF tem para corrigir — para ruído por eixo com
  bias real seria necessário trocar para `libgazebo_ros_imu_sensor.so`
  (formato SDF nativo do sensor IMU do Gazebo).
- **LiDAR removido**: o bloco `<sensor type="ray">` + plugin
  `libgazebo_ros_laser` foi apagado inteiro. O link/joint/malha de
  `base_scan` continuam no URDF só para preservar massa e inércia
  originais — não há `/scan`.
- **Nenhum plugin de ground truth** foi adicionado (ex.: `libgazebo_ros_p3d`).
  Pose real, quando necessária, vem de `/gazebo/model_states`.

## Raciocínio do `process_noise_covariance` (EKF)

Os valores de `x`/`y` (índices 0,1) ficam moderados (`0.05`) porque essas
posições nunca são medidas diretamente — só integradas de `vx` — então o
filtro precisa admitir incerteza crescente nelas. `yaw`/`vyaw` (índices 5,
11) usam um processo mais permissivo que o padrão de exemplo do
`robot_localization`, coerente com o `gaussianNoise = 0.01` da IMU: se o
processo fosse mais "confiante" que o próprio sensor, o filtro reagiria
devagar demais às correções de orientação, que são o ponto central deste
projeto. Os demais estados ficam nos valores de exemplo padrão do pacote.

## Limitações do dead reckoning

Com apenas IMU e encoders não existe nenhuma referência de posição
absoluta:

- O erro de **posição** cresce de forma cumulativa e ilimitada. O EKF
  reduz a taxa de crescimento (o giroscópio corrige o yaw, a maior fonte
  de erro em um robô diferencial), mas não zera.
- Derrapagem de roda é o pior inimigo: cada patinada vira erro permanente.
  Como as plantas têm colisão, esbarrar em uma delas arruína a odometria
  dali em diante.
- Percursos longos terminam visivelmente fora do lugar no RViz, mesmo
  dirigindo bem — é o comportamento correto do sistema, não um bug.
- Para fechar o laço algum dia: LiDAR + AMCL, GPS/RTK + segundo EKF, ou
  âncoras UWB publicando pose absoluta como uma terceira fonte —
  entrariam como `pose0` em `config/ekf_local.yaml`. Não implementado
  nesta versão.

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
