<p align="center">
  <img src="assets/logo-pucpr.png" alt="Logo da PUCPR" width="220">
</p>   

<h1 align="center">Atividades — Sistemas Robóticos</h1>

<p align="center">PUCPR · Pontifícia Universidade Católica do Paraná</p>

## Sobre

Repositório das atividades da disciplina de Sistemas Robóticos.

## Atividade 1 - Fundamentos de ROS 2 e Turtlesim
A atividade trata-se de obvserver a comunicação entre o nós do turlesim ,utilizando conceitos aprendidos como o Publisher,Subscriber que estão dentro do  Topic.

lembrando que nós(popularmente chamado em inglês como: Node) é um programa independente que executa uma tarefa específica no robô, como ler um sensor ou acionar um motor.
Cada nó tem uma única responsabilidade o que lhe confere baixo acoplamento e dependências.


Um Tópico (Topic) no ROS 2 é um canal de comunicação assíncrono baseado no modelo publicar/assinar, usado para transmitir informações continuamente de um nó para outro no robô


O Tópico é o Mural: É o local com um nome fixo e específico, onde a informação fica disponível. O mural em si não cria nem lê dados, ele serve apenas como o canal.

O Publisher (Publicador) é a Instituição que lança o Edital: É o nó responsável por gerar os dados (ex: o nó da câmera) e pregar a informação no mural.

O Subscriber (Inscrito) é o Candidato atento ao Mural: É o nó interessado naquele assunto (ex: o nó de IA). Ele fica inscrito no tópico e "lê" a informação assim que uma nova publicação é colada.É o nó que assina

A Mensagem é o Papel/Documento do Edital: É o texto com os campos preenchidos de acordo com um formulário oficial  contendo os dados concretos
em outras palavras: é a estrutura de dados;funciona como uma struct em C , agrupando campos com tipos bem definidos


Service é a comunicação síncrono (Request/Response) usado para interações pontuais do tipo "pergunta e resposta" ou "comando e confirmação" entre dois nós
ex: client quer saber a localização atual do robô
server: ele é quem calciula e ja responde devolvendo a posição


Action (Ação) é o mecanismo de comunicação assíncrono projetado para comandos e tarefas de longa duração que levam tempo para serem concluídos

Parâmetros: Fazem parte da programação dinâmica,altera o comportamento de um nó sem recompilação

# Atividade 2 - LiDAR e navegação reativa

## Objetivo

Ler as distâncias do LiDAR do TurtleBot3, visualizar os obstáculos ao redor e
implementar uma reação local que permita avançar em áreas livres e desviar
quando um obstáculo aparece à frente.

## Desenvolvimento e arquitetura

O pacote `turtlebot_lidar` contém dois nós independentes. `lidar_plotter`
assina `/scan`, converte cada leitura polar válida para coordenadas cartesianas
e atualiza uma visualização 2D. `reactive_navigation` também assina `/scan` e
publica comandos `geometry_msgs/msg/TwistStamped` em `/cmd_vel`.

O controlador divide o scan em setores frontal, esquerdo e direito. Se a
distância frontal mínima cair abaixo de `0.55 m`, ele gira para o lado com maior
distância lateral média. Só volta a avançar quando a frente ultrapassa `0.75 m`.
Essa diferença entre os limiares é uma histerese que evita alternância rápida
entre avançar e desviar.

## Código-fonte

- `Atv2/src/turtlebot_lidar/turtlebot_lidar/lidar_plotter.py`: conversão de
  `LaserScan` polar para pontos XY e atualização do gráfico.
- `Atv2/src/turtlebot_lidar/turtlebot_lidar/reactive_navigation.py`:
  filtragem de leituras inválidas, divisão angular do scan, escolha do lado de
  desvio e publicação de velocidades.
- `Atv2/src/turtlebot_lidar/setup.py`: registra os comandos
  `lidar_plotter` e `reactive_navigation`.

O QoS `qos_profile_sensor_data` é usado no scan porque dados de sensor são
contínuos e podem priorizar atualização recente em vez de manter longas filas.

## Instruções de execução

Em todos os terminais:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
```

Dependências da simulação e dos gráficos:

```bash
sudo apt update
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo \
  python3-numpy python3-matplotlib
```

Terminal 1 - simulação:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2 - compilação e visualização LiDAR:

```bash
cd "/mnt/c/Users/Gabriel Valerio/Documents/Universidade-Geral/PUCPR/Sistemas Roboticos/Atividade-Avaliativa/Atv2"
colcon build --packages-select turtlebot_lidar
source install/setup.bash
ros2 run turtlebot_lidar lidar_plotter
```

Terminal 3 - navegação reativa:

```bash
source /opt/ros/jazzy/setup.bash
source "/mnt/c/Users/Gabriel Valerio/Documents/Universidade-Geral/PUCPR/Sistemas Roboticos/Atividade-Avaliativa/Atv2/install/setup.bash"
ros2 run turtlebot_lidar reactive_navigation
```

Não execute outro controlador que publique em `/cmd_vel` ao mesmo tempo.

## Resultados e evidências

Para demonstrar a atividade, registre uma captura do gráfico com o robô no
centro e os obstáculos do scan, além de uma captura do Gazebo mostrando o
desvio. O log periódico informa a distância frontal e se o robô está andando
reto ou desviando. As capturas ainda devem ser anexadas ao repositório.

## Discussão e perguntas

- **Por que usar a distância mínima à frente?** Uma única leitura curta no
  corredor frontal já indica risco de colisão.
- **Por que comparar as médias laterais?** O lado com maior espaço médio tende
  a oferecer uma direção de desvio menos obstruída.
- **O que fazem `0.55 m` e `0.75 m`?** São os limiares de entrada e saída do
  modo de desvio; a separação cria histerese.
- **Qual é uma limitação?** É uma reação local, sem planejamento global de rota;
  obstáculos e geometrias complexas podem exigir uma estratégia mais completa.

# Atividade 3 - Transformações TF2 e mapeamento com LiDAR

## Objetivo

Usar a árvore TF2 para converter leituras do LiDAR do referencial do sensor
para os referenciais do robô e da odometria, formando uma visualização global
de pontos e da trajetória percorrida.

## Desenvolvimento e arquitetura

`tf2_lidar_mapper` converte as leituras válidas de `LaserScan` em pontos
cartesianos. TF2 fornece as transformações entre o frame do sensor,
`base_link` e `odom`. Os pontos transformados para `odom` são acumulados numa
nuvem global; uma grade de voxels de `0.03 m` evita inserir repetidamente
pontos na mesma célula. O gráfico também apresenta o scan atual, a posição do
robô e sua trajetória.

O arquivo `robot_tf.launch.py` publica transformações estáticas do `base_link`
para frames do footprint, LiDAR e rodas. `include_footprint:=false` deve ser
usado quando o simulador já publica essa transformação, para evitar duas
autoridades para o mesmo frame.

## Código-fonte

- `Atv3/src/robot_tf2/robot_tf2/tf2_lidar_mapper.py`:
  `transform_points` aplica rotação e translação 3D; `scan_cb` filtra o scan,
  consulta TF2 e integra os pontos; `update_plot` redesenha mapa e trajetória.
- `Atv3/src/robot_tf2/launch/robot_tf.launch.py`: publica transformações
  estáticas dos frames e permite desativar a transformação duplicada do
  footprint.
- `Atv3/src/robot_tf2/setup.py`: instala o launch e registra os executáveis.

Este mapa é uma acumulação visual em `odom`; não é um mapa de ocupação SLAM
nem implementa fechamento de loop.

## Instruções de execução

Dependências ROS 2 e de visualização:

```bash
sudo apt update
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-tf2-ros python3-numpy python3-matplotlib
```

Terminal 1 - simulação:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2 - compilar e publicar TFs estáticas:

```bash
source /opt/ros/jazzy/setup.bash
cd "/mnt/c/Users/Gabriel Valerio/Documents/Universidade-Geral/PUCPR/Sistemas Roboticos/Atividade-Avaliativa/Atv3"
colcon build --packages-select robot_tf2
source install/setup.bash
ros2 launch robot_tf2 robot_tf.launch.py include_footprint:=false
```

Terminal 3 - visualização global:

```bash
source /opt/ros/jazzy/setup.bash
source "/mnt/c/Users/Gabriel Valerio/Documents/Universidade-Geral/PUCPR/Sistemas Roboticos/Atividade-Avaliativa/Atv3/install/setup.bash"
ros2 run robot_tf2 tf2_lidar_mapper
```

## Resultados e evidências

Registre uma captura do gráfico mostrando pontos globais, scan atual e
trajetória, além do terminal com a contagem de pontos. As evidências visuais
ainda devem ser anexadas.

## Discussão e perguntas

- **Por que TF2?** A pose do sensor não é a mesma do robô; a transformação
  posiciona os pontos no referencial comum `odom`.
- **Por que discretizar em voxels?** A resolução de `3 cm` reduz duplicações
  ao acumular scans sucessivos.
- **Por que o frame `odom`?** Ele acompanha o deslocamento local sem exigir um
  mapa global corrigido por fechamento de loop.
- **Qual é uma limitação?** Erros de odometria acumulam e deformam a nuvem com
  o tempo; a visualização não substitui SLAM.

# Atividade 4 - Controle PID, fuzzy e híbrido

## Objetivo

Comparar estratégias de controle para levar o TurtleBot3 a uma meta usando
odometria, e avaliar como uma lógica fuzzy pode ajudar a desviar de obstáculos
detectados pelo LiDAR.

## Desenvolvimento e arquitetura

Os três nós assinam `/odom` e publicam `TwistStamped` em `/cmd_vel`. O
controlador PID calcula erro de distância e de orientação; limita a integral
para reduzir windup, limita as velocidades e gira antes de avançar quando o
erro angular é grande. O fuzzy transforma erros de distância e ângulo em graus
de pertinência e combina regras com operador mínimo e saídas singleton cuja
média ponderada produz as velocidades. O híbrido usa PID para seguir a meta e
troca para desvio fuzzy quando o LiDAR detecta obstáculo à frente.

No controlador híbrido, `enter_fuzzy_dist=0.42 m` inicia o desvio e
`exit_fuzzy_dist=0.52 m` permite voltar ao PID; essa histerese reduz oscilações
de modo. `fuzzy_hold_ticks=12` mantém o desvio por pelo menos 1,2 s no ciclo de
0,1 s. Os ganhos `kp`, `ki` e `kd` definem as contribuições proporcional,
integral e derivativa.

## Código-fonte

- `Atv4/src/robot_controllers/robot_controllers/pid_controller.py`:
  odometria, erro de posição/rumo e PID com saturação.
- `Atv4/src/robot_controllers/robot_controllers/fuzzy_controller.py`:
  funções triangulares/trapezoidais de pertinência, regras fuzzy e
  defuzzificação por média ponderada.
- `Atv4/src/robot_controllers/robot_controllers/hybrid_controller.py`:
  leitura do LiDAR, alternância PID/Fuzzy e controle de velocidade.
- `Atv4/src/robot_controllers/setup.py`: registra os três nós.

Cada controlador deve ser executado sozinho: todos publicam em `/cmd_vel`.

## Instruções de execução

Dependências da simulação:

```bash
sudo apt update
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo
```

Terminal 1 - simulação:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2 - compilar:

```bash
source /opt/ros/jazzy/setup.bash
cd "/mnt/c/Users/Gabriel Valerio/Documents/Universidade-Geral/PUCPR/Sistemas Roboticos/Atividade-Avaliativa/Atv4"
colcon build --packages-select robot_controllers
source install/setup.bash
```

Escolha apenas um comando para o Terminal 2:

```bash
ros2 run robot_controllers pid_controller --ros-args -p goal_x:=0.5 -p goal_y:=0.5
ros2 run robot_controllers fuzzy_controller --ros-args -p goal_x:=-0.5 -p goal_y:=-0.5
ros2 run robot_controllers hybrid_controller --ros-args -p goal_x:=1.5 -p goal_y:=0.5
```

## Resultados e evidências

Para comparar os controladores, registre capturas do Gazebo e do log com
posição, erro até a meta e velocidades. No híbrido, registre também a mudança
de modo quando o LiDAR detecta um obstáculo. As evidências ainda devem ser
anexadas; não se presume um resultado sem executar cada controlador.

## Discussão e perguntas

- **O que cada termo PID faz?** P reage ao erro atual, I acumula erro residual
  e D reage à sua variação; os limites ajudam a manter comandos seguros.
- **Como a lógica fuzzy decide?** Funções de pertinência ativam regras em graus
  diferentes; uma média ponderada converte as saídas em velocidades.
- **Por que usar um híbrido?** O PID é adequado ao seguimento da meta e o fuzzy
  muda a resposta quando o LiDAR indica um obstáculo frontal.
- **Por que não executar os três juntos?** Publicariam comandos concorrentes
  no mesmo tópico `/cmd_vel`.

# Atividade 5
Objetivo: 
Realizar a implementação prática de um sistema de detecção de objetos em visão computacional por meio do retreinamento (fine-tuning) da arquitetura YOLO26 para reconhecer uma classe personalizada inexistente no dataset COCO padrão: um mini-skate de dedo (Tech Deck, rotulado como skate-de-dedo). O trabalho abrange desde a coleta e anotação de imagens próprias até o treinamento com ajuste dinâmico de taxa de aprendizado, avaliação por métricas quantitativas e inferência em vídeo.

Desenvolvimento:
Arquitetura do Sistema
O pipeline foi estruturado de forma modular em 6 células no ambiente Google Colab com aceleração por GPU (NVIDIA T4), dividido em quatro etapas principais:

Ingestão e Padronização de Dados: Extração automatizada do conjunto de dados exportado do Roboflow (31 imagens reais capturadas em diferentes ângulos e iluminações), convertendo anotações do formato COCO JSON para o formato nativo YOLO (coordenadas normalizadas).

Transfer Learning (Fine-Tuning): Utilização dos pesos pré-treinados da rede yolo26s.pt (Small), descartando intencionalmente variantes ultraleves (Nano) para garantir maior capacidade de extração de características geométricas e texturas finas (lixa, shape, trucks e rodinhas).

Otimização e Regularização: Aplicação de Data Augmentation em tempo de execução (rotação de 
±20 
∘
 , variação de escala de 
50% e ajustes no espaço de cores HSV de saturação e brilho) combinada ao escalonador ReduceLROnPlateau do PyTorch, acoplado ao ciclo de treinamento via callback (on_fit_epoch_end), reduzindo a taxa de aprendizado pela metade sempre que a perda de validação estagna.

Pós-processamento e Inferência: Filtragem das predições em vídeo utilizando limiar de confiança (conf), limite máximo de detecções por quadro (max_det=1) e Non-Maximum Suppression (NMS) (iou=0.45) para eliminar caixas delimitadoras (bounding boxes) duplicadas ou sobrepostas sobre o mesmo objeto.


Algoritmos usados:
Non-Maximum Suppression (NMS): Algoritmo de pós-processamento que ordena as caixas candidatas pelo escore de confiança e suprime aquelas cuja interseção sobre união (IoU) com a caixa principal ultrapassa o limiar definido (0.45).

ReduceLROnPlateau: Estratégia adaptativa de otimização que monitora a soma das perdas de validação (val/box_loss + val/cls_loss + val/dfl_loss) e reduz o learning rate (factor=0.5, patience=6, min_lr=1e-6) para estabilizar a convergência em datasets compactos.

Gargalo na Rotulagem Automática e Formato de Exportação do Roboflow:

Problema: O recurso de Auto Label em nuvem do Roboflow apresentou lentidão na fila gratuita, e o arquivo exportado veio estruturado no padrão COCO JSON em vez do arquivo data.yaml esperado nativamente pelo YOLO.

Solução: Realização da rotulagem manual precisa das 31 imagens e implementação de um conversor automático em Python (preparar_dataset_31_fotos) que transforma as coordenadas COCO em rótulos YOLO normalizados.

Falsos Positivos no Fundo e Caixas Delimitadoras Duplicadas:

Problema: Nos testes iniciais utilizando variantes Nano (yolo11n/yolo26n) e limiares padrão de supressão, o detector gerava múltiplas caixas fragmentadas sobre o mesmo skate (separando shape e rodinhas) e falsos positivos em texturas do ambiente (cortina e borda do mousepad).

Solução: Substituição pela arquitetura yolo26s.pt (Small), ajuste do limiar de IoU do NMS (iou=0.45) e restrição de detecções máximas por quadro (max_det=1).

Divergência de Escala (Enquadramento) entre Treino e Vídeo de Teste:

Problema: As 31 imagens do dataset foram capturadas em close-up (com o skate ocupando ~80% da altura da imagem), enquanto o primeiro vídeo de teste foi gravado com a câmera distante (640x384), fazendo com que o modelo não detectasse o objeto pequeno com limiar conf=0.35.

Solução: Ajuste do limiar de confiança na inferência (conf=0.10) e gravação do vídeo de demonstração em enquadramento próximo (close-up), alinhando a distribuição espacial da inferência aos dados de treinamento.

Oscilação das Perdas em Dataset Compacto:

Problema: Com apenas 31 imagens, taxas de aprendizado fixas ou decaimento cosseno padrão provocavam picos de instabilidade na perda de validação.

Solução: Implementação do callback com ReduceLROnPlateau, reduzindo o learning rate dinamicamente diante de platôs na perda de validação e permitindo convergência suave nas épocas finais.

# Atividade 6 - Navegação autônoma com Nav2

## Objetivo

Configurar o TurtleBot3 simulado com Nav2, localizar o robô no mapa e percorrer
quatro posições programadas, monitorando a chegada e enviando automaticamente
a próxima meta.

## Desenvolvimento e arquitetura

O launch de demonstração `tb3_simulation_launch.py` inicia o Gazebo, o mapa,
AMCL, Nav2 e RViz. No RViz, **2D Pose Estimate** inicializa a pose e permite
que o Nav2 estabeleça a transformação `map` → `odom`. O pacote da atividade
publica cada `geometry_msgs/msg/PoseStamped` em `/goal_pose`, conforme o
tutorial, e lê `nav_msgs/msg/Odometry` em `/odom`. TF2 transforma a posição
odométrica para `map`, garantindo que a pose atual e as metas sejam comparadas
no mesmo referencial.

O nó aguarda a localização e a assinatura Nav2 de `/goal_pose`. Considera a
meta alcançada após três amostras consecutivas dentro de `0.25 m`; interrompe
o percurso após `180 s` sem progresso. Nav2 realiza o planejamento e o
desvio, enquanto o pacote sequencia as metas.

## Código-fonte

- `Atv6/src/turtlebot3_nav2_tour/turtlebot3_nav2_tour/nav2_four_goals.py`:
  armazena quatro metas, publica `PoseStamped`, transforma `/odom` com TF2,
  monitora tolerância e timeout e avança a sequência.
- `Atv6/src/turtlebot3_nav2_tour/launch/tour.launch.py`: define as quatro
  coordenadas de exemplo e ativa `use_sim_time` para compatibilidade com
  relógio do Gazebo.
- `Atv6/src/turtlebot3_nav2_tour/setup.py`: registra o nó e instala o launch.

As coordenadas em `tour.launch.py` são exemplos em metros e radianos no frame
`map`; devem ser conferidas no mapa antes da execução.

## Instruções de execução

Instale o Nav2 e o simulador de demonstração caso ainda não estejam instalados:

```bash
sudo apt update
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-minimal-tb3-sim
```

Pare simulações anteriores. Terminal 1 - iniciar a simulação integrada conforme
o tutorial:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

No RViz, use **2D Pose Estimate** e aguarde a localização. Terminal 2 -
compilar e executar:

```bash
source /opt/ros/jazzy/setup.bash
cd "/mnt/c/Users/Gabriel Valerio/Documents/Universidade-Geral/PUCPR/Sistemas Roboticos/Atividade-Avaliativa/Atv6"
colcon build --packages-select turtlebot3_nav2_tour
source install/setup.bash
ros2 launch turtlebot3_nav2_tour tour.launch.py
```

Não inicie outro Gazebo ou Cartographer ao mesmo tempo: o launch integrado já
fornece a simulação e o Nav2.

## Resultados e evidências

Registre uma captura do RViz com o mapa, a pose localizada e o percurso, além
do terminal mostrando as quatro metas sendo publicadas e confirmadas. As
evidências ainda devem ser anexadas ao repositório.

## Discussão e perguntas

- **Por que usar `/goal_pose`?** É o tópico `PoseStamped` usado pelo tutorial
  para enviar uma meta ao Nav2.
- **Por que transformar `/odom` para `map`?** O Nav2 planeja no frame global;
  comparar coordenadas de frames diferentes daria uma distância incorreta.
- **Por que usar três amostras?** Evita avançar por uma única leitura ruidosa
  que entre temporariamente na tolerância.
- **O que faz o Nav2 e o que faz o nó da atividade?** Nav2 localiza, planeja e
  controla o movimento; o nó publica as quatro metas e monitora a sequência.

## Vídeos, imagens e saídas de execução

- [Vídeos e imagens das atividades no Google Drive](https://drive.google.com/drive/folders/1jHMVbB9rEWsodq1rjxyefh8WXPZSaqKZ?usp=sharing)
- [Saídas da execução no Google Colab — notebook 1](https://colab.research.google.com/drive/1znd6R_C7RavS7i_9WEIi5CH2W7hqREUy?usp=sharing)
- [Saídas da execução no Google Colab — notebook 2](https://colab.research.google.com/drive/1CnaD1bUFr40MM7gsZ9oxuKpqeHKCumYN?usp=sharing)

## Atividades

- [Atividade 1](Atv1/)
- [Atividade 2](Atv2/)
- [Atividade 3](Atv3/)
- [Atividade 4](Atv4/)
- [Atividade 5](Atv5/)
- [Atividade 6](Atv6/)