#!/bin/bash
# Sobe tudo de uma vez: container (se precisar), backend ROS (vision_node,
# supervisor, rosbridge...), o controle físico (MoveIt Servo, cam_teleop,
# garra) e o hand_node (webcam do operador).
#
# NÃO sobe o driver do braço (kortex_bringup/gen3_lite.launch.py) -- isso
# precisa do IP real do robô, que este script não conhece. Suba-o à parte,
# ANTES de rodar este script.
#
# Rode este script no HOST (fora do container), de dentro da pasta eq2_ros.
# Ele entra e sai do container sozinho via `docker exec` — você não precisa
# digitar nada dentro dele manualmente.
#
# Uso:
#   ./start_demo.sh                 # cria/reaproveita o container "kortex_humble_4"
#   ./start_demo.sh --no-gpu        # usa run_no_gpu.sh em vez de run.sh
#   ROS_DOMAIN_ID=19 TEAM=2 ./start_demo.sh   # pula as perguntas do run.sh
#
# Reexecutar é seguro: ele reaproveita o container se já existir, só
# recompila e resobe os nós.
set -uo pipefail


CONTAINER=kortex_humble_4
DOMAIN="${ROS_DOMAIN_ID:-19}"
TEAM="${TEAM:-2}"
RUN_SCRIPT=./run.sh
[ "${1:-}" = "--no-gpu" ] && RUN_SCRIPT=./run_no_gpu.sh


log() { echo -e "\n\033[1;36m==> $*\033[0m"; }
fail() { echo -e "\033[1;31mERRO:\033[0m $*" >&2; exit 1; }


# ---------------------------------------------------------------------------
log "1/7 Container"
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
   if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
       echo "Container parado, iniciando..."
       docker start "$CONTAINER" || fail "não consegui iniciar o container existente"
   else
       [ -x "$RUN_SCRIPT" ] || fail "$RUN_SCRIPT não encontrado/executável nesta pasta"
       echo "Criando container novo ($RUN_SCRIPT, domain=$DOMAIN, equipe=$TEAM)..."
       printf '%s\n%s\n' "$DOMAIN" "$TEAM" | "$RUN_SCRIPT" || fail "run.sh falhou"
   fi
   sleep 2
else
   echo "Container já está rodando."
fi
docker ps --format '{{.Names}}\t{{.Status}}' | grep "$CONTAINER"


dexec() { docker exec "$CONTAINER" bash -lc "$1"; }
dexec_bg() { docker exec -d "$CONTAINER" bash -lc "$1"; }


# ---------------------------------------------------------------------------
log "2/7 Compilando fs1"
dexec 'source /opt/ros/humble/setup.bash && cd /app/project_ws && colcon build --symlink-install --packages-select fs1' \
   || fail "colcon build falhou"


# ---------------------------------------------------------------------------
log "3/7 Detectando a câmera do robô"
# Procura qualquer by-id de câmera que NÃO seja a webcam integrada, testando
# cada interface até achar uma que realmente devolve um frame. Se não achar
# nenhuma externa, cai no índice 0 (só a integrada mesmo).
CAMERA_INDEX=$(dexec '
source /opt/ros/humble/setup.bash >/dev/null 2>&1
python3 - <<"PYEOF"
import glob, re
import cv2


candidates = []
for path in sorted(glob.glob("/dev/v4l/by-id/*-video-index*")):
   if "Integrated_Camera" in path:
       continue  # webcam do notebook, não é essa que queremos aqui
   candidates.append(path)


for path in candidates:
   # CAP_V4L2 explícito: sem isso, uma string (device path) cai no backend
   # GStreamer e falha com "uridecodebin" em vez de abrir o device de verdade,
   # fazendo a detecção achar que a câmera não funciona.
   cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
   ok, _ = cap.read()
   cap.release()
   if ok:
       print(path)
       raise SystemExit


print(0)  # nada externo funcionou: usa a webcam integrada mesmo
PYEOF
' 2>/dev/null | tail -1)


[ -z "$CAMERA_INDEX" ] && CAMERA_INDEX=0
echo "Usando camera_index=$CAMERA_INDEX para o vision_node (câmera do robô)."


# ---------------------------------------------------------------------------
log "4/7 Subindo o backend (rosbridge, supervisor, vision_node...)"
dexec 'pkill -f "[r]os2 launch fs1 frontend.launch.py"; pkill -f "[i]nstall/fs1/lib/fs1"; pkill -f "[r]osbridge_websocket"; pkill -f "[i]mage_transport/republish"' >/dev/null 2>&1
sleep 1
dexec_bg "source /opt/ros/humble/setup.bash && source /app/project_ws/install/setup.bash && ros2 launch fs1 frontend.launch.py camera_index:=$CAMERA_INDEX > /tmp/frontend.log 2>&1"
sleep 6
dexec 'source /opt/ros/humble/setup.bash; ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 1; ros2 node list | sort'


# ---------------------------------------------------------------------------
log "5/7 Subindo o controle físico (MoveIt Servo, cam_teleop, garra)"
# Requer o driver do braço (kortex_bringup) já rodando à parte, com o IP
# real do robô -- este script não sabe qual IP vocês usam, então isso fica
# de fora (ver instruções de laboratório).
dexec 'pkill -f "[r]os2 launch fs1 claw_machine.launch.py"; pkill -f "[s]ervo_node_main"; pkill -f "[s]ervo_adapter"; pkill -f "[C]amTeleop"; pkill -f "[g]ripper_client"; pkill -f "[g]ripper_control"' >/dev/null 2>&1
sleep 1
dexec_bg "source /opt/ros/humble/setup.bash && source /app/workspace/ros2_kortex_ws/install/setup.bash && source /app/project_ws/install/setup.bash && ros2 launch fs1 claw_machine.launch.py > /tmp/claw_machine.log 2>&1"
sleep 3


# ---------------------------------------------------------------------------
log "6/7 Preparando o hand_node (câmera do operador)"
if ! dexec 'test -x /opt/handvenv/bin/python3' 2>/dev/null; then
   echo "Ambiente do mediapipe não existe ainda neste container — instalando (só acontece uma vez por container)..."
   dexec 'apt-get update -qq' || fail "apt-get update falhou (sem internet?)"
   dexec 'apt-get install -y -qq python3-venv python3-pip' || fail "apt-get install falhou"
   dexec 'python3 -m venv --system-site-packages /opt/handvenv' || fail "venv falhou"
   dexec '/opt/handvenv/bin/pip install -q mediapipe==1.0.1' || fail "pip install mediapipe falhou"
   dexec '/opt/handvenv/bin/pip install -q --ignore-installed matplotlib' || fail "pip install matplotlib falhou"
else
   echo "Ambiente do mediapipe já existe, pulando instalação."
fi


xhost +local:docker >/dev/null 2>&1 || echo "(xhost falhou — se a janela do OpenCV não abrir, rode 'xhost +local:docker' no host)"


dexec 'pkill -f "/opt/handvenv.*hand_node"' >/dev/null 2>&1
sleep 1
dexec_bg 'source /opt/ros/humble/setup.bash && source /app/project_ws/install/setup.bash && /opt/handvenv/bin/python3 -c "
import rclpy
from fs1.hand_node import HandNode
rclpy.init()
node = HandNode()
rclpy.spin(node)
" > /tmp/hand_node.log 2>&1'
sleep 3
dexec 'source /opt/ros/humble/setup.bash; ros2 topic pub --once /switchHandDetection std_msgs/msg/String "{data: \"{}\"}"' >/dev/null


# ---------------------------------------------------------------------------
log "7/7 Pronto"
echo "Backend:       docker exec $CONTAINER tail -f /tmp/frontend.log"
echo "Controle físico: docker exec $CONTAINER tail -f /tmp/claw_machine.log"
echo "Hand node:     docker exec $CONTAINER tail -f /tmp/hand_node.log"
echo
echo "Agora abra o front, em outro terminal:"
echo "  cd ../eq2_front && flutter run -d chrome"



