#!/usr/bin/env python3
"""End-effector CAN bridge — can_msgs/Frame over ros2_socketcan, no raw socket.
Same split as rover_peripherals/rover_lighting_node.py.

Topics:
  Sub  end_effector_controller/command  (std_msgs/String): open, close,
       drill_up, drill_down, stop_step, stop_drill, lock, unlock, read_ph
  Pub  end_effector_controller/state    (indomitus_interfaces/EndEffectorState)
  Pub  /to_can_bus, Sub /from_can_bus   (can_msgs/Frame)

CAN IDs (docs/hardware/can_bus.md): jaw 0x1A/0x1B, astrobio 0x1C/0x1D,
drill_sampling 0x1E/0x1F. astrobio command bytes (open=suck on, close=suck
off, read_ph=request pH reading) confirmed live via cansend — see
_on_astrobio_command. Its 0x1D reply's byte layout is NOT confirmed yet, so
_on_can_frame only logs it raw for now rather than guessing a decode.
"""

import struct
import time

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from std_msgs.msg import String
from indomitus_interfaces.msg import EndEffectorState

JAW_CMD_ID = 0x1A
JAW_ACK_ID = 0x1B
JAW_CMD_SAFE_OPEN = 3
JAW_CMD_SAFE_CLOSE = 4
JAW_CMD_READ_LOAD_SENSORS = 7

DRILL_SAMPLING_CMD_ID = 0x1E
DRILL_CMD_CLOSE = 1
DRILL_CMD_OPEN = 2
DRILL_CMD_STOP_STEP = 3
DRILL_CMD_DOWN = 4
DRILL_CMD_UP = 5
DRILL_CMD_STOP_DRILL = 6
DRILL_CMD_LOCK = 7
DRILL_CMD_UNLOCK = 8

ASTROBIO_CMD_ID = 0x1C
ASTROBIO_REPLY_ID = 0x1D
ASTROBIO_CMD_SUCK_ON = 1
ASTROBIO_CMD_SUCK_OFF = 2
ASTROBIO_CMD_READ_PH = 4

_VALID_COMMANDS = frozenset({
    'open', 'close', 'drill_up', 'drill_down',
    'stop_step', 'stop_drill', 'lock', 'unlock', 'read_ph',
})

_STALE_REPLY_POLL_PERIODS = 3.0  # poll periods before a jaw reply is 'stale'


class EndEffectorCanNode(Node):
    def __init__(self):
        super().__init__('end_effector_can_node')

        self._declare_parameters()
        self._load_parameters()

        self._can_tx_pub = self.create_publisher(Frame, '/to_can_bus', 10)
        self._can_rx_sub = self.create_subscription(
            Frame, '/from_can_bus', self._on_can_frame, 10)

        self._cmd_sub = self.create_subscription(
            String, 'end_effector_controller/command', self._on_command, 10)
        self._state_pub = self.create_publisher(
            EndEffectorState, 'end_effector_controller/state', 10)

        self._last_right_g = 0.0
        self._last_left_g = 0.0
        self._last_reply_time = None
        self._warned_bad_values = set()

        if self._end_effector == 'jaw':
            self.create_timer(self._load_poll_period_sec, self._on_load_poll_timer)

        self.get_logger().info(f"EndEffectorCanNode ready (end_effector='{self._end_effector}')")

    def _declare_parameters(self):
        self.declare_parameter('end_effector', 'jaw')
        self.declare_parameter('load_poll_period_sec', 0.1)

    def _load_parameters(self):
        self._end_effector = self.get_parameter('end_effector').value
        self._load_poll_period_sec = self.get_parameter('load_poll_period_sec').value

    def _on_command(self, msg: String):
        command = msg.data

        if command not in _VALID_COMMANDS:
            if command not in self._warned_bad_values:
                self._warned_bad_values.add(command)
                self.get_logger().warn(f"Ignoring unrecognized command '{command}'.")
            return

        if self._end_effector == 'jaw':
            self._on_jaw_command(command)
        elif self._end_effector == 'drill_sampling':
            self._on_drill_sampling_command(command)
        elif self._end_effector == 'astrobio':
            self._on_astrobio_command(command)

    def _on_jaw_command(self, command: str):
        if command == 'open':
            self._send_cmd(JAW_CMD_ID, [JAW_CMD_SAFE_OPEN])
        elif command == 'close':
            self._send_cmd(JAW_CMD_ID, [JAW_CMD_SAFE_CLOSE])
        else:
            self.get_logger().warn(
                f"'{command}' not supported for end_effector=jaw.", throttle_duration_sec=1.0)

    def _on_drill_sampling_command(self, command: str):
        if command == 'open':
            self._send_cmd(DRILL_SAMPLING_CMD_ID, [DRILL_CMD_OPEN, 0, 0, 0, 0])
        elif command == 'close':
            self._send_cmd(DRILL_SAMPLING_CMD_ID, [DRILL_CMD_CLOSE, 0, 0, 0, 0])
        elif command == 'drill_up':
            self._send_cmd(DRILL_SAMPLING_CMD_ID, [DRILL_CMD_UP, 0, 0, 0, 0])
        elif command == 'drill_down':
            self._send_cmd(DRILL_SAMPLING_CMD_ID, [DRILL_CMD_DOWN, 0, 0, 0, 0])
        elif command == 'stop_step':
            self._send_cmd(DRILL_SAMPLING_CMD_ID, [DRILL_CMD_STOP_STEP])
        elif command == 'stop_drill':
            self._send_cmd(DRILL_SAMPLING_CMD_ID, [DRILL_CMD_STOP_DRILL])
        elif command == 'lock':
            self._send_cmd(DRILL_SAMPLING_CMD_ID, [DRILL_CMD_LOCK])
        elif command == 'unlock':
            self._send_cmd(DRILL_SAMPLING_CMD_ID, [DRILL_CMD_UNLOCK])

    def _on_astrobio_command(self, command: str):
        # 'open'/'close' reused as suck-on/suck-off — same D-pad UP/LEFT
        # bindings as jaw's gripper open/close (gamepad_servo_node), no
        # separate command name needed since only one end_effector is ever
        # active. Confirmed live via cansend can0 01C#01/02/04.
        if command == 'open':
            self._send_cmd(ASTROBIO_CMD_ID, [ASTROBIO_CMD_SUCK_ON])
        elif command == 'close':
            self._send_cmd(ASTROBIO_CMD_ID, [ASTROBIO_CMD_SUCK_OFF])
        elif command == 'read_ph':
            self._send_cmd(ASTROBIO_CMD_ID, [ASTROBIO_CMD_READ_PH])
        else:
            self.get_logger().warn(
                f"'{command}' not supported for end_effector=astrobio.", throttle_duration_sec=1.0)

    def _send_cmd(self, can_id: int, data: list):
        msg = Frame()
        msg.id = can_id
        msg.dlc = len(data)
        msg.data = data + [0] * (8 - len(data))
        msg.is_extended = False
        msg.is_rtr = False
        msg.is_error = False
        self._can_tx_pub.publish(msg)
        self.get_logger().info(f"TX CAN 0x{can_id:03X}  data={[f'0x{b:02X}' for b in data]}")

    def _on_can_frame(self, msg: Frame):
        if self._end_effector == 'astrobio':
            self._on_astrobio_frame(msg)
            return
        if self._end_effector != 'jaw':
            return
        if msg.id != JAW_ACK_ID:
            return
        # DLC 4 = load reply, DLC 2 = open/close ACK (no tag byte on this ID).
        if msg.dlc == 2:
            return
        if msg.dlc != 4:
            return

        right_g, left_g = struct.unpack('<hh', bytes(msg.data[:4]))
        self._last_right_g = float(right_g)
        self._last_left_g = float(left_g)
        self._last_reply_time = time.monotonic()
        self._publish_state(self._last_right_g, self._last_left_g, connected=True)

    def _on_astrobio_frame(self, msg: Frame):
        if msg.id != ASTROBIO_REPLY_ID:
            return
        # Byte layout of the pH reply is not confirmed yet (only the request
        # side, 01C#04, was) — log it raw rather than guess a decode here.
        # Once the layout is known, parse + publish it (new message/field;
        # EndEffectorState's load_right_g/load_left_g are jaw-specific, not
        # a fit) instead of just logging.
        self.get_logger().info(
            f"RX CAN 0x{msg.id:03X}  data={[f'0x{b:02X}' for b in msg.data[:msg.dlc]]}"
        )

    def _on_load_poll_timer(self):
        self._send_cmd(JAW_CMD_ID, [JAW_CMD_READ_LOAD_SENSORS])
        now = time.monotonic()
        stale = (
            self._last_reply_time is None
            or (now - self._last_reply_time) > _STALE_REPLY_POLL_PERIODS * self._load_poll_period_sec
        )
        if stale:
            self._publish_state(self._last_right_g, self._last_left_g, connected=False)

    def _publish_state(self, right_g: float, left_g: float, connected: bool):
        msg = EndEffectorState()
        msg.load_right_g = right_g
        msg.load_left_g = left_g
        msg.connected = connected
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
