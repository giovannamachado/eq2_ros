"""
Supervisor node: finite state machine (FSM) that connects the Flutter front-end
to the robot and vision nodes.

Contract with the front-end (through rosbridge):

* ``/supervisor/command`` (``std_msgs/String``): ``"start"`` or ``"stop"``.
* ``/supervisor/state`` (``std_msgs/String``): JSON published on every
  transition and as a 2 Hz heartbeat, e.g.
  ``{"state": "TELEOP", "selected_slot": 3, "message": "...", "error": null}``.

It reuses the topics of the existing nodes instead of talking to the robot
directly:

* ``/shelf_state`` (``std_msgs/String``, JSON list of
  ``{"position", "occupied", "color"}``) published by ``my_node``.
* ``/posicoes_garra`` (JSON list of 6 joint angles, in degrees) consumed by
  ``joints_control``.
* ``/controlador_garra`` (``"abrir"``/``"fechar"``) consumed by
  ``gripper_control``.
* ``/escolher_cubo`` (``std_msgs/String``, e.g. ``"Cubo 04"``) consumed by
  ``fs1.cam_teleop``: tells it which of the 8 known cube positions is the
  one picked by the sorteio, published as soon as TELEOP starts.
* ``/pick_result`` (``std_msgs/String``, ``"success"``/``"failure"``)
  published by ``fs1.cam_teleop`` once its own closed-loop pick sequence
  (triggered by the same closed-hand gesture, via ``/hand_status``) finishes
  — "success" if it was aligned with the right cube when the gesture fired,
  "failure" otherwise (RF#07). The supervisor does not react to
  ``/hand_status`` directly anymore; see the note below.

Full cycle: IDLE -> HOME_INIT -> SCANNING -> TELEOP -> (cam_teleop grips the
cube and reports success) -> DROP -> place, release -> IDLE, or -> (reports
failure) -> FAILURE -> HOME -> IDLE. The actual grip+retreat motion (RF#06)
and the alignment check (RF#07) both live in ``fs1.cam_teleop``
(closed-loop, real end-effector position via TF2) — the supervisor only
tells it which cube to aim for and reacts to the outcome. It no longer runs
its own open-loop pick attempt: with ``fs1.cam_teleop`` also listening to
``/hand_status`` and driving the arm via ``/cmd_vel``, having the supervisor
*also* send joint-trajectory commands on the same gesture would be two
controllers fighting over the same motion. Only the drop leg (DROP: moving
the already-grasped piece to the bin) stays joint-trajectory, since by then
cam_teleop's own sequence has finished and the arm is free again.
"""

import json
import random
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fs1.kinova_api import home, place, pre_place

# Tempos padrão (segundos) de cada etapa do drop (a pega em si é do
# fs1.cam_teleop agora; ver docstring do módulo), copiados de
# fs1/kinova_api.py (KinovaApi.put_in_box_function), que já tinha essas
# posições calibradas na bancada. Viram parâmetros ROS (ver
# Supervisor.__init__), então dá para ajustar a velocidade sem mexer em
# código, ex.: ros2 run fs1 supervisor --ros-args -p drop_approach_s:=5.0
DEFAULT_DROP_APPROACH_S = 9.0
DEFAULT_DROP_PLACE_S = 9.0
DEFAULT_DROP_RETREAT_S = 7.0

# fs1.cam_teleop usa "Cubo 01".."Cubo 08" (ver self.posicoes_cubos lá) para
# os 8 slots; o sorteio aqui usa posição 1..8. Essa função traduz entre os
# dois sem precisar duplicar os nomes em nenhum dos dois arquivos.
def cube_label(position):
    """Translate a slot position (1..8) to fs1.cam_teleop's "Cubo 0N" name."""
    return f'Cubo {position:02d}'

MIN_PERSISTENCE_S = 1.0  # RNF#02: minimum window before confirming a slot.
STATE_HEARTBEAT_S = 0.5


class State(str, Enum):
    """States of the supervisor FSM (see section 2.5 of the requirements)."""

    IDLE = 'IDLE'
    HOME_INIT = 'HOME_INIT'
    SCANNING = 'SCANNING'
    TELEOP = 'TELEOP'
    DROP = 'DROP'
    FAILURE = 'FAILURE'


def confirm_occupied_slots(samples, min_ratio=0.6):
    """
    Confirm which slots hold a piece from a window of ``/shelf_state`` samples.

    Args:
        samples (list): Each item is a parsed ``/shelf_state`` message (list
            of dicts with ``position``, ``occupied`` and ``color``).
        min_ratio (float): Fraction of samples in which a slot must appear
            occupied, with the same color, to be confirmed.

    Returns:
        dict: ``{position: color}`` of the confirmed slots.
    """
    if not samples:
        return {}

    votes = {}
    for sample in samples:
        for slot in sample:
            if slot.get('occupied') and slot.get('color'):
                key = (slot['position'], slot['color'])
                votes[key] = votes.get(key, 0) + 1

    confirmed = {}
    for (position, color), count in votes.items():
        if count / len(samples) >= min_ratio:
            confirmed[position] = color
    return confirmed


def parse_command(raw):
    """Return the normalized command (``"start"``/``"stop"``) or ``None``."""
    command = str(raw).strip().lower()
    return command if command in ('start', 'stop') else None


class Supervisor(Node):
    """ROS 2 node that owns the FSM and exposes it to the front-end."""

    def __init__(self):
        """Create publishers, subscribers and the heartbeat timer."""
        super().__init__('supervisor')

        self.declare_parameter('home_settle_s', 7.0)
        self.declare_parameter('scan_window_s', 2.0)
        self.declare_parameter('drop_approach_s', DEFAULT_DROP_APPROACH_S)
        self.declare_parameter('drop_place_s', DEFAULT_DROP_PLACE_S)
        self.declare_parameter('drop_retreat_s', DEFAULT_DROP_RETREAT_S)

        self.state = State.IDLE
        self.selected_slot = None
        self.message = 'Sistema em repouso.'
        self.error = None
        self._samples = []
        self._transition_timer = None
        self._picking = False

        self.state_pub = self.create_publisher(String, '/supervisor/state', 10)
        self.joints_pub = self.create_publisher(String, '/posicoes_garra', 10)
        self.gripper_pub = self.create_publisher(
            String, '/controlador_garra', 10)
        # Diz ao fs1.cam_teleop qual dos 8 cubos é o sorteado (ver docstring
        # do módulo).
        self.choose_cube_pub = self.create_publisher(
            String, '/escolher_cubo', 10)

        self.create_subscription(
            String, '/supervisor/command', self._on_command, 10)
        self.create_subscription(
            String, '/shelf_state', self._on_shelf_state, 10)
        # Resultado do próprio ciclo de pega do fs1.cam_teleop (RF#06/RF#07):
        # "success" ou "failure". Só é considerado durante TELEOP.
        self.create_subscription(
            String, '/pick_result', self._on_pick_result, 10)

        self.create_timer(STATE_HEARTBEAT_S, self._publish_state)
        self._publish_state()
        self.get_logger().info('Supervisor ready (IDLE).')

    # -- helpers -----------------------------------------------------------

    def _publish_state(self):
        """Publish the current FSM snapshot as JSON."""
        msg = String()
        msg.data = json.dumps({
            'state': self.state.value,
            'selected_slot': self.selected_slot,
            'message': self.message,
            'error': self.error,
        })
        self.state_pub.publish(msg)

    def _set_state(self, state, message, error=None):
        """Transition to ``state`` and notify the front-end immediately."""
        self.get_logger().info(f'{self.state.value} -> {state.value}: {message}')
        self.state = state
        self.message = message
        self.error = error
        self._publish_state()

    def _after(self, seconds, callback):
        """Run ``callback`` once after ``seconds`` without blocking the executor."""
        self._cancel_timer()

        def fire():
            self._cancel_timer()
            callback()

        self._transition_timer = self.create_timer(seconds, fire)

    def _cancel_timer(self):
        """Cancel the pending one-shot transition timer, if any."""
        if self._transition_timer is not None:
            self._transition_timer.cancel()
            self.destroy_timer(self._transition_timer)
            self._transition_timer = None

    def _go_home(self, open_gripper):
        """Send the robot to the HOME pose, optionally opening the gripper."""
        joints = String()
        joints.data = json.dumps(home)
        self.joints_pub.publish(joints)
        if open_gripper:
            gripper = String()
            gripper.data = 'abrir'
            self.gripper_pub.publish(gripper)

    # -- callbacks ---------------------------------------------------------

    def _on_shelf_state(self, msg):
        """Collect vision samples while the FSM is scanning."""
        if self.state is not State.SCANNING:
            return
        try:
            self._samples.append(json.loads(msg.data))
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid /shelf_state payload ignored.')

    def _on_pick_result(self, msg):
        """
        React to fs1.cam_teleop's own pick/failure outcome (RF#06/RF#07).

        cam_teleop watches /hand_status itself and runs the actual
        grip-or-retry motion with real TF2 feedback; the supervisor just
        waits for the verdict. "success" means the piece is in the gripper
        and the arm is back where teleop left it -> go place it (DROP).
        "failure" means the gesture fired away from the target cube -> go
        home and let the operator try again.
        """
        if self.state is not State.TELEOP or self._picking:
            return
        result = msg.data.strip().lower()
        if result == 'success':
            self._picking = True
            self._drop_approach()
        elif result == 'failure':
            self._picking = True
            self._set_state(
                State.FAILURE,
                f'Gesto fora do slot {self.selected_slot}. Voltando para HOME.')
            self._go_home(open_gripper=False)
            self._after(self.get_parameter('home_settle_s').value, self._finish_pick_cycle)
        else:
            self.get_logger().warn(f'/pick_result desconhecido ignorado: {msg.data!r}')

    def _drop_approach(self):
        """HOME -> DROP: move to the pre-place pose."""
        self._set_state(State.DROP, 'Levando a peça até o drop...')
        self.joints_pub.publish(String(data=json.dumps(pre_place)))
        self._after(self.get_parameter('drop_approach_s').value, self._drop_place)

    def _drop_place(self):
        """Move into the final place pose, over the drop."""
        self.joints_pub.publish(String(data=json.dumps(place)))
        self._after(self.get_parameter('drop_place_s').value, self._drop_release)

    def _drop_release(self):
        """Open the gripper, release the piece, and head back to HOME."""
        self.gripper_pub.publish(String(data='abrir'))
        self.joints_pub.publish(String(data=json.dumps(home)))
        self._after(self.get_parameter('drop_retreat_s').value, self._finish_pick_cycle)

    def _finish_pick_cycle(self):
        """DROP/FAILURE -> IDLE: cycle complete, ready for the next start."""
        self._picking = False
        self.selected_slot = None
        if self.state is not State.FAILURE:
            self._set_state(State.IDLE, 'Peça entregue. Sistema em repouso.')
        else:
            self._set_state(State.IDLE, 'Sistema em repouso.')

    def _on_command(self, msg):
        """Handle ``start``/``stop`` commands from the front-end."""
        command = parse_command(msg.data)
        if command == 'start':
            self._start()
        elif command == 'stop':
            self._stop()
        else:
            self.get_logger().warn(f'Unknown command ignored: {msg.data!r}')

    # -- transitions -------------------------------------------------------

    def _start(self):
        """IDLE -> HOME_INIT: go to HOME and open the gripper."""
        if self.state is not State.IDLE:
            self.get_logger().warn(f'start ignored while in {self.state.value}.')
            return
        self.selected_slot = None
        self._go_home(open_gripper=True)
        self._set_state(State.HOME_INIT, 'Indo para HOME e abrindo a garra.')
        self._after(self.get_parameter('home_settle_s').value, self._begin_scan)

    def _begin_scan(self):
        """HOME_INIT -> SCANNING: collect shelf samples for the window."""
        self._samples = []
        self._set_state(State.SCANNING, 'Escaneando a estante.')
        window = max(
            MIN_PERSISTENCE_S, self.get_parameter('scan_window_s').value)
        self._after(window, self._finish_scan)

    def _finish_scan(self):
        """SCANNING -> TELEOP (or back to IDLE when the shelf is empty)."""
        occupied = confirm_occupied_slots(self._samples)
        if not occupied:
            self._set_state(
                State.IDLE, 'Nenhuma peça encontrada na estante.',
                error='no_piece')
            return
        self.selected_slot = random.choice(sorted(occupied))
        color = occupied[self.selected_slot]
        self._set_state(
            State.TELEOP,
            f'Pegue a peça {color} do slot {self.selected_slot}.')
        # Avisa o fs1.cam_teleop qual dos 8 cubos é o sorteado, para ele
        # saber se o gesto de fechar a mão aconteceu no lugar certo.
        self.choose_cube_pub.publish(
            String(data=cube_label(self.selected_slot)))

    def _stop(self):
        """Any state -> IDLE: cancel pending work and return to HOME."""
        self._cancel_timer()
        self.selected_slot = None
        self._picking = False
        self._go_home(open_gripper=False)
        self._set_state(State.IDLE, 'Operação interrompida. Robô em HOME.')


def main(args=None):
    """Entry point for the supervisor node."""
    rclpy.init(args=args)
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':# pragma: no cover
    main()
