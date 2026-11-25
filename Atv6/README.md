<p align="center">
  <img src="../assets/logo-pucpr.png" alt="Logo da PUCPR" width="220">
</p>

# Atividade 6 - Navegação autônoma com Nav2

Pacote ROS 2: `turtlebot3_nav2_tour`

O tutorial da disciplina usa o launch de demonstração do Nav2, que inicia o
TurtleBot3 simulado, o mapa, o AMCL e o Nav2. Não é necessário iniciar
Cartographer nem salvar um mapa para este fluxo.

O nó do pacote publica quatro metas `geometry_msgs/msg/PoseStamped` no tópico
`/goal_pose`. Ele acompanha `/odom`, transforma a pose de `odom` para `map`
usando TF2 e publica a próxima meta depois que o robô permanece dentro da
tolerância configurada.

## 1. Iniciar a simulação e o Nav2

Pare simulações antigas com `Ctrl+C`. Em um terminal do Ubuntu/WSL:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

Esse launch abre o Gazebo e o RViz com Nav2. No RViz, selecione **2D Pose
Estimate** e indique a pose inicial do robô no mapa. Aguarde a localização
antes de iniciar o percurso.

## 2. Compilar o pacote

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd "/mnt/c/Users/Gabriel Valerio/Documents/Universidade-Geral/PUCPR/Sistemas Roboticos/Atividade-Avaliativa/Atv6"
colcon build --packages-select turtlebot3_nav2_tour
source install/setup.bash
```

## 3. Executar as quatro metas

Depois de marcar a pose inicial no RViz, no mesmo terminal do build:

```bash
ros2 launch turtlebot3_nav2_tour tour.launch.py
```

O nó aguarda a localização TF `map` → `odom` e o assinante Nav2 de `/goal_pose`.
As metas de exemplo estão em `src/turtlebot3_nav2_tour/launch/tour.launch.py`.
Edite `x`, `y` e `yaw` para que correspondam a posições livres do mapa. As
coordenadas são em metros e o ângulo `yaw` é em radianos, no frame `map`.

## Verificação

Em outro terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic info /goal_pose
ros2 topic echo /odom
```

O nó informa a distância até a meta atual e publica a próxima meta após
confirmar que a posição recebida por `/odom`, transformada para `map`, está
dentro da tolerância.
