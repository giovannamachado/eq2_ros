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
* ``/hand_status`` (``std_msgs/String``, JSON with a ``"closed"`` bool)
  published by ``fs1.hand_node``: the closed-hand gesture, watched only
  during TELEOP, triggers the automatic pick/drop routine (RF#06).

Full cycle: IDLE -> HOME_INIT -> SCANNING -> TELEOP -> (closed-hand gesture)
-> advance into the slot, grip, retreat -> DROP -> place, release -> IDLE.
The pick/place joint positions come from ``fs1.kinova_api`` (already
calibrated on the bench). FAILURE (RF#07, wrong alignment) is not
implemented: telling a good pick from a missed one needs either the real
end-effector position relative to the slot or a grip-success signal, and
neither exists yet, so every closed-hand gesture during TELEOP takes the
success path.
"""

import json
import random
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fs1.kinova_api import cubos, home, place, pre_place

# Tempos padrão (segundos) de cada etapa da rotina automática de pega e
# drop, copiados de fs1/kinova_api.py (KinovaApi.pick_one_cube /
# put_in_box_function), que já tinham essas posições calibradas na bancada.
# Viram parâmetros ROS (ver Supervisor.__init__), então dá para ajustar a
# velocidade da pega sem mexer em código, ex.:
#   ros2 run fs1 supervisor --ros-args -p pick_advance_s:=5.0
DEFAULT_PICK_ADVANCE_S = 9.0
DEFAULT_PICK_GRIP_S = 3.0
DEFAULT_PICK_RETREAT_S = 7.0
DEFAULT_DROP_APPROACH_S = 9.0
DEFAULT_DROP_PLACE_S = 9.0
DEFAULT_DROP_RETREAT_S = 7.0

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
        self.declare_parameter('pick_advance_s', DEFAULT_PICK_ADVANCE_S)
        self.declare_parameter('pick_grip_s', DEFAULT_PICK_GRIP_S)
        self.declare_parameter('pick_retreat_s', DEFAULT_PICK_RETREAT_S)
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
        self._hand_was_closed = False

        self.state_pub = self.create_publisher(String, '/supervisor/state', 10)
        self.joints_pub = self.create_publisher(String, '/posicoes_garra', 10)
        self.gripper_pub = self.create_publisher(
            String, '/controlador_garra', 10)

        self.create_subscription(
            String, '/supervisor/command', self._on_command, 10)
        self.create_subscription(
            String, '/shelf_state', self._on_shelf_state, 10)
        # Vem do fs1.hand_node (rastreamento de mão): {"closed": bool, ...}.
        # Só é usado durante TELEOP, para disparar a pega automática (RF#06).
        self.create_subscription(
            String, '/hand_status', self._on_hand_status, 10)

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

    def _on_hand_status(self, msg):
        """
        Watch for the closed-hand gesture while teleoperating (RF#06).

        Only reacts during TELEOP, and only on the closed-hand *edge* (open
        -> closed), so holding the hand closed doesn't retrigger the pick.

        Limitation: this always takes the success path (advance, grip,
        return, drop). Telling a correctly-aligned pick from a missed one
        (RF#07) needs to know the end-effector's real position relative to
        the slot (e.g. via TF2, the way fs1.cam_teleop checks its workspace
        limits) or a grip-success signal from the gripper; neither exists
        yet, so a wrong alignment today still runs the same "success" motion
        instead of the FAILURE recovery in fs1's FSM diagram.
        """
        if self.state is not State.TELEOP or self._picking:
            return
        try:
            closed = bool(json.loads(msg.data).get('closed', False))
        except json.JSONDecodeError:
            return

        if closed and not self._hand_was_closed:
            self._start_pick_sequence()
        self._hand_was_closed = closed

    def _start_pick_sequence(self):
        """TELEOP -> advance into the slot and grip (first half of RF#06)."""
        self._picking = True
        slot = self.selected_slot
        self._set_state(
            State.TELEOP, f'Pegando a peça do slot {slot}...')
        self.joints_pub.publish(String(data=json.dumps(cubos[slot - 1])))
        self._after(self.get_parameter('pick_advance_s').value, self._pick_close_gripper)

    def _pick_close_gripper(self):
        """Close the gripper once the arm has reached the cube."""
        self.gripper_pub.publish(String(data='fechar'))
        self._after(self.get_parameter('pick_grip_s').value, self._pick_retreat)

    def _pick_retreat(self):
        """Retreat to HOME with the piece before heading to the drop."""
        self.joints_pub.publish(String(data=json.dumps(home)))
        self._after(self.get_parameter('pick_retreat_s').value, self._drop_approach)

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
        """DROP -> IDLE: cycle complete, ready for the next start command."""
        self._picking = False
        self._hand_was_closed = False
        self.selected_slot = None
        self._set_state(State.IDLE, 'Peça entregue. Sistema em repouso.')

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

    def _stop(self):
        """Any state -> IDLE: cancel pending work and return to HOME."""
        self._cancel_timer()
        self.selected_slot = None
        self._picking = False
        self._hand_was_closed = False
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


if __name__ == '__main__':
    main()
