# eq2_ros — Teleoperação Kinova Gen3 Lite (ROS 2)

Pacote ROS 2 (Humble) do sistema de pick-and-place híbrido do braço Kinova
Gen3 Lite: visão computacional identifica cubos numa estante, o operador
sorteia um cubo pelo front-end ([eq2_front](../eq2_front)), e a pega final é
feita por gesto (fechar a mão), com alinhamento verificado via TF2.

## Arquitetura

O sistema roda em **dois launches separados**, propositalmente:

| Launch | O que sobe | Por quê |
|---|---|---|
| `frontend.launch.py` | `rosbridge_server`, `supervisor` (FSM), `vision_node`, `joints_control`, republishers de câmera | Fala com o front-end Flutter e cuida do ciclo HOME → SCANNING → TELEOP → DROP |
| `claw_machine.launch.py` | MoveIt Servo, `servo_adapter`, `cam_teleop`, `gripper_client`, `gripper_control`, `hand_node` | Controle físico de verdade: braço em malha fechada (TF2) e garra |

Os dois precisam rodar **juntos** para o sistema completo funcionar — e
**não devem ser duplicados** (rodar `servo_adapter`/`cam_teleop` duas vezes,
por exemplo, cria dois controladores brigando pelo mesmo `/cmd_vel`).

Além dos dois, é preciso o **driver do braço físico** (fora deste pacote,
vem do workspace `ros2_kortex` dentro do container):

```bash
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=<IP_DO_ROBO> launch_rviz:=false
```

Fluxo da máquina de estados (`fs1/supervisor.py`):

```
IDLE -> HOME_INIT -> SCANNING -> TELEOP -> (cam_teleop pega o cubo)
  -> sucesso -> DROP -> IDLE
  -> falha   -> FAILURE -> IDLE
```

## Pré-requisitos

- Docker, com a imagem `kortex_humble:1.2.4` já disponível.
- Uma câmera para a estante/efetuador (ex. Logitech BRIO) e uma webcam
  para o operador detectar gestos.
- Braço Kinova Gen3 Lite acessível por Ethernet (IP fixo).

## Build

```bash
docker exec kortex_humble_4 bash -lc \
  'source /opt/ros/humble/setup.bash && cd /app/project_ws && \
   colcon build --symlink-install --packages-select fs1'
```

## Rodando tudo

O script `start_demo.sh` automatiza a maior parte disso (container, build,
detecção de câmera, os dois launches e o `hand_node`):

```bash
./start_demo.sh                 # cria/reaproveita o container kortex_humble_4
./start_demo.sh --no-gpu        # usa run_no_gpu.sh em vez de run.sh
```

Ele **não** sobe o driver do braço (precisa do IP do robô, que varia por
bancada) — suba-o manualmente antes, como mostrado acima.

Para rodar manualmente, passo a passo, veja os comentários no topo de
`frontend.launch.py`, `claw_machine.launch.py` e do próprio
`start_demo.sh`.

### Câmeras

`vision_node` (câmera do efetuador) e `hand_node` (webcam do operador)
aceitam o parâmetro `camera_index` como número (`0`, `1`...) ou como um
caminho estável `/dev/v4l/by-id/usb-<algo>-video-indexN` — prefira o
caminho estável, já que o número pode mudar se a câmera for trocada de
porta USB.

## Testes

```bash
docker exec kortex_humble_4 bash -lc \
  'source /opt/ros/humble/setup.bash && source /app/project_ws/install/setup.bash && \
   cd /app/project_ws/src/eq2_ros/fs1 && python3 -m pytest test/'
```

`test/test_z_hand.py` precisa do `mediapipe`, que só existe na venv
isolada `/opt/handvenv` (ver abaixo) — ignore-o (`--ignore=test/test_z_hand.py`)
ao rodar com o Python do sistema.

Cobertura de docstrings (`interrogate`):

```bash
docker exec kortex_humble_4 bash -lc \
  'cd /app/project_ws/src/eq2_ros && interrogate fs1/fs1'
```

## O `hand_node` e a venv do mediapipe

O `mediapipe` precisa de `numpy>=2`, que quebra o ABI do `cv_bridge`/OpenCV
do sistema (compilados para `numpy 1.x`). Por isso o `hand_node` roda numa
venv isolada, criada uma vez por container:

```bash
python3 -m venv --system-site-packages /opt/handvenv
/opt/handvenv/bin/pip install mediapipe==1.0.1
/opt/handvenv/bin/pip install --ignore-installed matplotlib
```

E executado com o Python dessa venv, não o `ros2 run` normal:

```bash
/opt/handvenv/bin/python3 -c "
import rclpy
from fs1.hand_node import HandNode
rclpy.init()
rclpy.spin(HandNode())
"
```

`start_demo.sh` já faz essa instalação automaticamente na primeira vez.
