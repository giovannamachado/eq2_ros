"""Tests for the supervisor FSM (pure helpers and node transitions)."""

import json
import os
import time

# Hermetic: use a private DDS domain so a running stack (or a colleague's
# nodes) can never feed these tests through the real /shelf_state topic.
os.environ['ROS_DOMAIN_ID'] = '231'

import pytest  # noqa: E402
import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from fs1.kinova_api import home  # noqa: E402
from fs1.supervisor import (  # noqa: E402
    State,
    Supervisor,
    confirm_occupied_slots,
    parse_command,
)


def shelf(**occupied):
    """Build a /shelf_state sample, e.g. ``shelf(p2='green')``."""
    slots = []
    for position in range(1, 9):
        color = occupied.get(f'p{position}')
        slots.append({
            'position': position,
            'occupied': color is not None,
            'color': color,
        })
    return slots


class Recorder:
    """Stand-in for a publisher that stores what was published."""

    def __init__(self):
        """Start with no published messages."""
        self.messages = []

    def publish(self, msg):
        """Record ``msg``."""
        self.messages.append(msg)


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialise rclpy once for all node tests."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    """Supervisor with fast timings and recording publishers."""
    supervisor = Supervisor()
    supervisor.set_parameters([
        rclpy.parameter.Parameter(
            'home_settle_s', rclpy.Parameter.Type.DOUBLE, 0.05),
        rclpy.parameter.Parameter(
            'scan_window_s', rclpy.Parameter.Type.DOUBLE, 0.05),
    ])
    supervisor.state_pub = Recorder()
    supervisor.joints_pub = Recorder()
    supervisor.gripper_pub = Recorder()
    yield supervisor
    supervisor.destroy_node()


def spin_until(node, condition, timeout=3.0):
    """Spin ``node`` until ``condition()`` holds or ``timeout`` expires."""
    deadline = time.time() + timeout
    while not condition() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
    return condition()


def command(node, text):
    """Deliver a command string to the supervisor."""
    node._on_command(String(data=text))


def feed(node, sample):
    """Deliver a /shelf_state sample to the supervisor."""
    node._on_shelf_state(String(data=json.dumps(sample)))


def test_parse_command():
    assert parse_command('start') == 'start'
    assert parse_command('  STOP ') == 'stop'
    assert parse_command('jump') is None
    assert parse_command('') is None


def test_confirm_occupied_slots_needs_persistence():
    assert confirm_occupied_slots([]) == {}
    samples = [shelf(p2='green')] * 3
    assert confirm_occupied_slots(samples) == {2: 'green'}


def test_confirm_occupied_slots_discards_flicker():
    samples = [shelf(p2='green'), shelf(), shelf(), shelf(), shelf()]
    assert confirm_occupied_slots(samples) == {}


def test_confirm_occupied_slots_requires_same_color():
    samples = [shelf(p3='blue'), shelf(p3='green'), shelf(p3='white')]
    assert confirm_occupied_slots(samples) == {}


def test_initial_state_is_idle(node):
    assert node.state is State.IDLE
    assert node.selected_slot is None


def test_publish_state_is_json(node):
    node._publish_state()
    payload = json.loads(node.state_pub.messages[-1].data)
    assert payload == {
        'state': 'IDLE',
        'selected_slot': None,
        'message': 'Sistema em repouso.',
        'error': None,
    }


def test_start_goes_home_then_scans_then_teleop(node):
    command(node, 'start')

    assert node.state is State.HOME_INIT
    assert json.loads(node.joints_pub.messages[-1].data) == home
    assert node.gripper_pub.messages[-1].data == 'abrir'

    assert spin_until(node, lambda: node.state is State.SCANNING)
    feed(node, shelf(p2='green', p5='blue'))
    feed(node, shelf(p2='green', p5='blue'))
    feed(node, shelf(p2='green', p5='blue'))
    assert spin_until(node, lambda: node.state is State.TELEOP)

    assert node.selected_slot in (2, 5)
    assert 'slot' in node.message
    assert node.error is None


def test_scan_window_never_below_minimum_persistence(node):
    command(node, 'start')
    assert spin_until(node, lambda: node.state is State.SCANNING)
    # scan_window_s=0.05 must be raised to the 1.0 s minimum (RNF#02).
    assert not spin_until(node, lambda: node.state is not State.SCANNING, 0.5)
    assert spin_until(node, lambda: node.state is not State.SCANNING, 1.5)


def test_empty_shelf_returns_to_idle_with_error(node):
    command(node, 'start')
    assert spin_until(node, lambda: node.state is State.SCANNING)
    feed(node, shelf())
    assert spin_until(node, lambda: node.state is State.IDLE, timeout=3.0)

    assert node.error == 'no_piece'
    assert node.selected_slot is None


def test_stop_returns_home_and_idle_from_any_state(node):
    command(node, 'start')
    assert spin_until(node, lambda: node.state is State.SCANNING)
    node.joints_pub.messages.clear()

    command(node, 'stop')

    assert node.state is State.IDLE
    assert json.loads(node.joints_pub.messages[-1].data) == home
    # The pending scan timer must be gone: no late transition after Stop.
    assert not spin_until(node, lambda: node.state is not State.IDLE, 1.5)


def test_start_is_ignored_while_busy(node):
    command(node, 'start')
    published = len(node.joints_pub.messages)

    command(node, 'start')

    assert node.state is State.HOME_INIT
    assert len(node.joints_pub.messages) == published


def test_unknown_command_is_ignored(node):
    command(node, 'dance')
    assert node.state is State.IDLE


def test_shelf_samples_ignored_outside_scanning(node):
    feed(node, shelf(p1='blue'))
    assert node._samples == []


def test_invalid_shelf_payload_is_ignored(node):
    command(node, 'start')
    assert spin_until(node, lambda: node.state is State.SCANNING)
    node._on_shelf_state(String(data='not json'))
    assert node._samples == []


def test_restart_after_finished_cycle(node):
    command(node, 'start')
    assert spin_until(node, lambda: node.state is State.SCANNING)
    feed(node, shelf(p4='white'))
    assert spin_until(node, lambda: node.state is State.TELEOP)
    command(node, 'stop')
    assert node.state is State.IDLE

    command(node, 'start')
    assert node.state is State.HOME_INIT
    assert node.selected_slot is None
