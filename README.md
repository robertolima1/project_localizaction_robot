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
| `localizacao` | `robot_localization` | Qual método usa a TF `odom → base_footprint`: `robot_localization`, `ekf_uwb` ou `trilateracao` (ver seção "Localização"). |
| `mu_rodas` | `0.1` | Atrito das rodas/caster — drift de odometria (ver "Parametrizando ruído para experimentos"). |
| `gaussian_noise_imu` | `0.01` | Ruído do plugin da IMU. |
| `sigma_range_extra` | `0.0` | Ruído extra (m) somado ao range UWB. |
| `sigma_angle_extra` | `0.0` | Ruído extra (rad) somado ao angle UWB. |

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
| `/odometry/filtered` | `nav_msgs/Odometry` | publica | `ekf_localization_node`, `ekf_localizacao_uwb` **ou** `localizacao_trilateracao` (ver `localizacao`) |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | publica | Gazebo (pose real, só para comparação manual) |
| `/gtec/toa/ranging` | `gtec_msgs/Ranging` | publica | plugin `libgtec_uwb_plugin` (externo) |
| `/gtec/toa/ranging` | `gtec_msgs/Ranging` | assina | `ruido_uwb.py` (sempre ativo, ver "Parametrizando ruído para experimentos") |
| `/gtec/toa/ranging_ruidoso` | `gtec_msgs/Ranging` | publica | `ruido_uwb.py` |
| `/gtec/toa/ranging_ruidoso` | `gtec_msgs/Ranging` | assina, só com `localizacao:=ekf_uwb` | `ekf_localizacao_uwb.py` |
| `/gtec/toa/ranging_ruidoso` | `gtec_msgs/Ranging` | assina, só com `localizacao:=trilateracao` | `localizacao_trilateracao.py` |
| `/gtec/toa/anchors` | `visualization_msgs/MarkerArray` | publica | plugin `libgtec_uwb_plugin` (externo) |

## Localização

`launch/localizacao.launch` publica a TF `odom → base_footprint` e o
tópico `/odometry/filtered`, através de **um entre três métodos
alternativos** (nunca dois ao mesmo tempo — disputariam a mesma TF),
escolhido pelo argumento `localizacao` do `bringup.launch`:

| `localizacao` | Nó | Fontes fundidas | Referência absoluta? |
|---|---|---|---|
| `robot_localization` (padrão) | `ekf_localization_node` | `/odom` + `/imu` | Não — dead reckoning puro (ver "Ruído de drift nas rodas"). |
| `ekf_uwb` | `scripts/ekf_localizacao_uwb.py` | `/odom` + `/imu` + `/gtec/toa/ranging` | Sim — EKF (Tabela 7.2) fundindo as âncoras UWB. |
| `trilateracao` | `scripts/localizacao_trilateracao.py` | `/imu` (só orientação) + `/gtec/toa/ranging` | Sim, mas sem filtro nenhum — geometria pura, sem predição nem covariância real. |

```bash
# EKF de dead reckoning (padrão)
roslaunch projeto_agrobot_uwb bringup.launch

# EKF com correspondência conhecida (Thrun, Tabela 7.2), usando as âncoras
# UWB como mapa de referência absoluta
roslaunch projeto_agrobot_uwb bringup.launch localizacao:=ekf_uwb

# Trilateração básica (sem filtro), só com as âncoras UWB
roslaunch projeto_agrobot_uwb bringup.launch localizacao:=trilateracao
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

`trilateracao` é o algoritmo de geometria pura em
`scripts/exemplo_trilateracao.py` (ver docstring lá) plugado no ROS: a
cada leitura de `/gtec/toa/ranging`, resolve `(x, y)` com a leitura mais
recente de cada âncora (mínimo `min_ancoras`, padrão 3; descarta leituras
mais velhas que `max_idade_leitura`) e publica direto, sem EKF nem
suavização — como trilateração 2D não dá orientação, o `yaw` publicado
vem direto da IMU. Serve de comparação didática com o `ekf_uwb`: mesma
fonte de dados, mas sem predição nem fusão bayesiana — dá pra ver o
quanto o EKF melhora sobre a solução geométrica "crua", principalmente
com menos de 3 âncoras visíveis (aí o `ekf_uwb` continua predizendo com
odom+IMU; o `trilateracao` simplesmente para de publicar).

## Sensor UWB

O mundo tem 10 âncoras UWB fixas (`uwb_anchor0..9`, uma por planta, na
cabeça de cada uma, a 0,35 m de altura — a planta vai até 0,30 m) e o
robô tem uma tag (`uwb_tag_link`) com o plugin
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

## Ruído de drift nas rodas

Não existe um parâmetro único chamado "drift" — a deriva da odometria de
rodas neste projeto é **física**, não uma injeção de ruído (diferente do
ruído extra de UWB, esse sim injetado — ver seção "Parametrizando ruído
para experimentos"). Vem de dois fatores:

**1. Atrito das rodas com o chão** —
[`urdf/agrobot.gazebo.xacro:17-53`](urdf/agrobot.gazebo.xacro#L17-L53)

```xml
<gazebo reference="wheel_left_link">
  <mu1>$(arg mu_rodas)</mu1>
  <mu2>$(arg mu_rodas)</mu2>
  ...
```

Repetido para `wheel_right_link` e `caster_back_link`. `mu1`/`mu2` são os
coeficientes de atrito ODE (longitudinal/lateral) — quanto **menor**,
mais a roda pode escorregar fisicamente em vez de rolar sem deslizar.
Parametrizado via `xacro:arg mu_rodas` (repassado pelo `bringup.launch`
— ver seção "Parametrizando ruído para experimentos"), padrão `0.1`,
mesmo valor herdado sem alteração do `turtlebot3_burger.gazebo.xacro`
original.

**2. `odometrySource: encoder`** no plugin `libgazebo_ros_diff_drive`
(mesmo arquivo, bloco do controlador) — o `/odom` é calculado integrando
a velocidade angular das juntas das rodas, assumindo rolamento sem
deslizar. Quando a roda escorrega de verdade (por causa do atrito baixo
acima), a física "sabe" que o robô não andou tanto quanto o encoder
registrou — e é exatamente essa diferença que vira erro acumulado em
`/odom`, o que o EKF tenta (parcialmente) corrigir com o giroscópio.

**Não tem** ruído gaussiano artificial somado à odometria de roda em si
(diferente da IMU, que tem `gaussianNoise` — também parametrizado, ver
abaixo). O único jeito de aumentar o drift de `/odom` é via `mu_rodas`
(mais escorregão físico) — não existe um segundo mecanismo separado de
ruído aditivo na odometria.

### `wheelRadius` / `wheelSeparation`

Existem em **dois lugares** que precisam ficar consistentes manualmente
— o plugin não lê a geometria do URDF automaticamente:

- **O que o `libgazebo_ros_diff_drive` usa de fato** para calcular
  `/odom` —
  [`urdf/agrobot.gazebo.xacro:84-85`](urdf/agrobot.gazebo.xacro#L84-L85):
  ```xml
  <wheelSeparation>0.160</wheelSeparation>
  <wheelDiameter>0.066</wheelDiameter>
  ```
- **A geometria física real das rodas**, em `agrobot.urdf.xacro`:
  - Raio: `<cylinder length="0.018" radius="0.033"/>` — linha 64 (roda
    esquerda, igual na direita). Bate com `wheelDiameter` do plugin
    (`0,033 × 2 = 0,066`).
  - Separação: não é um valor único, é derivado do `origin` das juntas —
    `wheel_left_joint` (linha 48, `y=0.08`) e `wheel_right_joint` (linha
    80, `y=-0.080`) → distância total `0,08 − (−0,08) = 0,16 m`, batendo
    com `wheelSeparation`.

Se mudar o raio/posição da roda no `agrobot.urdf.xacro` sem atualizar
`wheelDiameter`/`wheelSeparation` no `agrobot.gazebo.xacro`, o `/odom`
calculado descola da física real — uma fonte extra de drift, separada do
atrito, e bem mais fácil de esquecer.

## Parametrizando ruído para experimentos

As três fontes de ruído do projeto (roda, IMU, UWB) são ajustáveis por
argumento do `bringup.launch`, sem editar nenhum arquivo — pensado para
rodar várias rodadas de experimento variando só esses valores:

| Argumento | Padrão | Afeta |
|---|---|---|
| `mu_rodas` | `0.1` | Atrito das rodas/caster (`mu1`/`mu2`) — quanto menor, mais escorregão físico e mais drift em `/odom` (ver "Ruído de drift nas rodas"). |
| `gaussian_noise_imu` | `0.01` | `gaussianNoise` do plugin `libgazebo_ros_imu` — mesmo valor aplicado ao giroscópio (rad/s) e ao acelerômetro (m/s²), limitação do plugin (ver comentário em `agrobot.gazebo.xacro`). |
| `sigma_range_extra` | `0.0` | Desvio-padrão (m) do ruído gaussiano **extra** somado ao `range` de cada `gtec_msgs/Ranging`, além do que o plugin UWB já injeta. |
| `sigma_angle_extra` | `0.0` | Mesma ideia, para o `angle` (rad). |

```bash
# Mais drift de roda (mais escorregão) e IMU mais ruidosa
roslaunch projeto_agrobot_uwb bringup.launch mu_rodas:=0.03 gaussian_noise_imu:=0.03

# UWB com ruído extra, além do que o plugin já simula (LOS/NLOS)
roslaunch projeto_agrobot_uwb bringup.launch localizacao:=ekf_uwb sigma_range_extra:=0.1 sigma_angle_extra:=0.1
```

`mu_rodas` e `gaussian_noise_imu` viram `xacro:arg` (ver
`urdf/agrobot.gazebo.xacro`) repassados pelo `launch/gazebo.launch` na
hora de gerar o `robot_description` — mudam a física/o sensor simulado
de verdade, não só um número num arquivo de config.

`sigma_range_extra`/`sigma_angle_extra` funcionam diferente: o plugin
UWB (`libgtec_uwb_plugin`, fonte externa) **não expõe nenhum parâmetro
de ruído** via SDF — os desvios-padrão de range e o erro angular de ~5°
estão fixos no código C++ dele. Pra poder variar mesmo assim, o
`localizacao.launch` sempre sobe um nó `scripts/ruido_uwb.py` entre o
plugin e o EKF/trilateração: ele lê o `/gtec/toa/ranging` bruto, soma o
ruído extra configurado, e republica em `/gtec/toa/ranging_ruidoso` —
que é o tópico que `ekf_uwb.py` e `localizacao_trilateracao.py` assinam
de verdade (`~ranging_topic`). Com os dois em `0.0` (padrão), a saída é
idêntica à entrada — passa-through, sem mudar o comportamento de antes.

## Gravando um percurso para analisar a deriva

O `bringup.launch` já grava automaticamente, sem comando manual: um nó
`rosbag record` sobe junto com o resto e salva cada percurso em
`bags/percurso_<timestamp>.bag`, com os tópicos `/odom`, `/imu`,
`/odometry/filtered`, `/cmd_vel`, `/gtec/toa/ranging` e
`/gazebo/model_states` (ground truth).

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

### Reproduzindo um percurso gravado

`rosbag play` só republica as mensagens gravadas nos tópicos originais —
não sobe Gazebo, não simula física, não recria nada visual sozinho. Pra
dirigir de novo a mesma "receita" de comandos numa simulação nova (com a
física recalculada do zero), suba um `bringup.launch` sem teleop
automático e reproduza só o `/cmd_vel` do bag antigo nele:

```bash
# terminal 1: simulação nova, sem teleop automático (senão as duas fontes
# de /cmd_vel — o teleop e o rosbag play — disputariam o mesmo tópico)
roslaunch projeto_agrobot_uwb bringup.launch teleop:=false

# terminal 2: reproduz só os comandos de velocidade do percurso gravado
source ~/catkin_ws/devel/setup.bash
rosbag play bags/percurso_XXXX.bag --topics /cmd_vel
```

Isso repete a mesma sequência de comandos de velocidade, mas o resultado
fica **parecido, não idêntico** ao percurso original — a física do
Gazebo e o ruído da IMU/UWB não são determinísticos, então a trajetória
real pode variar um pouco a cada execução, mesma sequência de comandos ou
não. Se o percurso original não foi gravado com o robô no spawn padrão
(`x:=0 y:=0 yaw:=0`), suba o `bringup.launch` novo com os mesmos
`x`/`y`/`yaw` do original, senão o robô começa de um lugar diferente.

### Extraindo o percurso para CSV

`scripts/bag_para_csv.py` lê um ou mais `.bag` e escreve um único CSV em
**formato largo** — uma linha por instante de tempo, com colunas
separadas por fonte de pose (`odom_x, odom_y, odom_yaw, filtrada_x,
filtrada_y, filtrada_yaw, ground_truth_x, ground_truth_y,
ground_truth_yaw`, `yaw` já convertido do quaternion) e por âncora UWB
(`anchor0_range, anchor0_angle, anchor1_range, ...` — uma coluna por
âncora que aparecer no `.bag`, `range` já em metros), pronto para
comparar tudo lado a lado numa planilha ou no pandas sem precisar
pivotar depois:

```bash
rosrun projeto_agrobot_uwb bag_para_csv.py bags/percurso_XXXX.bag saida.csv
```

As fontes publicam em instantes diferentes e raramente batem o timestamp
exato, então o script arredonda `t` para o múltiplo mais próximo de
`--resolucao` (padrão `0.05` s = 20 Hz) e agrupa nesse intervalo — a
leitura mais recente de cada fonte/âncora dentro do intervalo é a que
fica na linha. Uma fonte/âncora sem leitura naquele intervalo sai com as
colunas vazias, não com erro (as âncoras publicam ciclando, então é
normal a maioria das células `anchorN_*` saírem vazias). Ajuste com
`--resolucao 0.1`, por exemplo, se quiser linhas mais "cheias" ao custo
de granularidade temporal.

O `bringup.launch` grava `/gazebo/model_states` (ground truth) e
`/gtec/toa/ranging` (âncoras) por padrão desde a versão atual — se algum
`.bag` mais antigo não tiver esses tópicos, as colunas `ground_truth_*` e
`anchorN_*` correspondentes saem vazias em todas as linhas, sem erro nem
aviso. Para juntar uma gravação separada desses tópicos a um `.bag`
antigo, o script aceita vários `.bag` de uma vez e alinha o tempo pelo
início mais antigo entre eles:

```bash
rosrun projeto_agrobot_uwb bag_para_csv.py bags/percurso_XXXX.bag bags/ground_truth_XXXX.bag saida.csv
```

## Perfilando CPU/memória de um node em execução

O ROS não expõe consumo de CPU/memória por node em nenhum `rostopic`/
`rosnode` — `scripts/perfil_recursos.py` cobre isso: amostra periodicamente
o processo de um node já rodando e grava um CSV (`t, cpu_percent, mem_mb`).
Acha o processo pelo grafo ROS (nome do node), não casando texto contra
`ps aux` — chama o método XML-RPC `getPid` que todo node ROS expõe na
própria URI (o mesmo mecanismo usado internamente por `rosnode kill`), o
que funciona igual em devel e install space e não confunde processos de
nome parecido.

```bash
# com o bringup.launch (ou qualquer node) já rodando em outro terminal:
rosrun projeto_agrobot_uwb perfil_recursos.py /ekf_localizacao_uwb perfil_ekf.csv

# amostrando mais rápido e com duração limitada
rosrun projeto_agrobot_uwb perfil_recursos.py /localizacao_trilateracao perfil.csv --intervalo 0.2 --duracao 60
```

| Argumento | Padrão | Descrição |
|---|---|---|
| `node` | — (obrigatório) | Nome do node no grafo ROS (ex.: `/ekf_localizacao_uwb`, `/localizacao_trilateracao`, `/ruido_uwb` — ver `rosnode list` para os disponíveis). Aceita com ou sem a `/` inicial. |
| `csv_saida` | — (obrigatório) | Caminho do `.csv` a gerar. |
| `--intervalo` | `0.5` | Período de amostragem, em segundos. |
| `--duracao` | sem limite | Para automaticamente após N segundos. Sem esse argumento, roda até Ctrl+C ou até o node monitorado terminar (o que vier primeiro). |

Cada amostra é gravada e o arquivo é *flusheado* na hora — não junta tudo
num buffer só pra escrever no final, então mesmo interrompido no meio o
CSV parcial já gravado fica utilizável. Mede só o processo do node em si
(nenhum node deste projeto cria subprocesso, então não há filhos a somar).
Se o node não existir no grafo ROS, o script termina com uma mensagem de
erro clara em vez de travar esperando.
