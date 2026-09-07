#!/usr/bin/env python3
"""
ROS2 node - lights control via CAN bus (ros2_socketcan).

This node owns the light state. Two operators command the lights and they ask
differently: the onboard joystick has momentary buttons and calls the /toggle
services, the ground station has latching switches and calls the SetBool ones
with an absolute value. Neither keeps a copy, so neither can drift; both read
the truth back off lights/state.

Every light below (except /lights/traffic_light) is served by a matched pair:

  Service: lights/<name>          (std_srvs/srv/SetBool)  - absolute
  Service: lights/<name>/toggle   (std_srvs/srv/Trigger)  - invert

  lights/spotlight        - both spotlight pins together
  lights/spotlight_left   - left spotlight pin only
  lights/spotlight_right  - right spotlight pin only
  lights/beautiful        - decorative animation, all 4 pins
  lights/beautiful_1..4   - one decorative pin, static (fights the animation
                             if it is running - that is a firmware quirk, not
                             something this node papers over)
  lights/traffic_red      - traffic-head red pin only
  lights/traffic_green    - traffic-head green pin only
  lights/traffic_blue     - traffic-head blue pin only
  lights/buzzer           - buzzer
  lights/tower            - all three traffic-head pins together

Service: lights/traffic_light     (indomitus_interfaces/srv/SetTrafficLight)
  request:  int8 red/green/blue, each KEEP | OFF | ON
  response: bool success, string message
  The traffic head is red/green/blue only - the firmware has no yellow LED.

Topic:   lights/state             (indomitus_interfaces/msg/LightsState)
  Latched. Republished on a 2 Hz heartbeat so a console can tell live state
  from a dead publisher, and pushed out at once on a real change so an
  operator's switch does not appear to lag - see _heartbeat and _publish_change.

CAN TX (PC -> ESP32)  ID cmd_id:
  Any single-pin light: byte 0 = cmd_<name>_on | cmd_<name>_off
  Traffic light:        byte 0 = cmd_traffic_light, byte 1 = bitmask
                         (R=bit0 G=bit1 B=bit2)

CAN RX (ESP32 -> PC):
  ID resp_id  byte 0 = echo cmd, byte 1 = 0x00 OK | 0x01 ERROR
"""

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from can_msgs.msg import Frame
from std_srvs.srv import SetBool, Trigger
from indomitus_interfaces.msg import LightsState as LightsStateMsg
from indomitus_interfaces.srv import SetTrafficLight

import signal
import threading
import time
from dataclasses import replace
from functools import partial
from typing import Callable, NamedTuple

from rover_peripherals.lights_state import (
    LightsState,
    describe_traffic,
    resolve_traffic,
    traffic_mask,
)


class LightSpec(NamedTuple):
    """One SetBool/toggle-served light: what it is called on the wire, and
    which LightsState fields flip together when it fires.

    `toggle_from` reads the target off the current state - that is what makes
    /toggle atomic: the read and the write both happen inside _can_lock, so
    two toggles racing cannot both see the same 'before'.
    """

    label: str
    on_cmd: int
    off_cmd: int
    fields: tuple
    toggle_from: Callable[[LightsState], bool]


class LightsCanNode(Node):
    def __init__(self):
        super().__init__("lights_can_node")

        self._declare_parameters()
        self._load_parameters()

        # --- callback groups ---
        # Every light service shares one group: they are serialised by
        # _can_lock anyway, and each one blocks for up to ack_s waiting on the
        # ESP32. The CAN subscription gets its own group so it stays free to
        # deliver the very ACK a blocked service is waiting for — putting it in
        # with the services would deadlock every call until the timeout.
        self._sub_cbg      = MutuallyExclusiveCallbackGroup()
        self._service_cbg  = MutuallyExclusiveCallbackGroup()
        self._state_cbg    = MutuallyExclusiveCallbackGroup()

        # --- state ---
        self._state = LightsState()
        # Serialises a whole send -> wait -> commit transaction. _resp_lock
        # below only guards the response fields; without this one, two
        # concurrent calls would overwrite each other's _resp_pending_cmd and
        # a toggle could read a state another call was midway through changing.
        self._can_lock = threading.Lock()
        self._state_dirty = True
        self._state_period = 1.0 / max(0.1, self._state_pub_rate)
        # Floor between out-of-turn publishes. A real change goes out at once
        # rather than waiting for the heartbeat; this only stops a burst of
        # changes from turning into a burst of packets.
        self._state_min_period = 0.05
        self._last_state_publish = 0.0

        self._lights = self._build_light_specs()
        # Every command byte this node is the author of. power_monitor_can_node
        # drives the same ESP32 over the same cmd/resp id pair, so the response
        # stream carries ACKs that are none of our business - this is what
        # tells one from the other.
        self._own_cmds = {self._cmd_traffic_light}
        for spec in self._lights.values():
            self._own_cmds.update((spec.on_cmd, spec.off_cmd))

        # --- CAN pub/sub ---
        self._pub = self.create_publisher(Frame, "to_can_bus", 10)
        self._sub = self.create_subscription(
            Frame, "from_can_bus", self._on_can_msg, 10,
            callback_group=self._sub_cbg,
        )

        # --- State topic ---
        # TRANSIENT_LOCAL on top of the heartbeat: a late subscriber (the
        # ground station UI after a reconnect) gets the last value on the spot
        # rather than up to a heartbeat period later.
        self._state_pub = self.create_publisher(
            LightsStateMsg,
            "lights/state",
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._state_timer = self.create_timer(
            self._state_period,
            self._publish_state,
            callback_group=self._state_cbg,
        )

        # --- Services ---
        # One SetBool + one Trigger per light in self._lights, both wired to
        # the same LightSpec so they can never disagree about what they send.
        self._light_services = {}
        for name, spec in self._lights.items():
            set_srv = self.create_service(
                SetBool, f"lights/{name}", partial(self._on_light_set, name),
                callback_group=self._service_cbg,
            )
            toggle_srv = self.create_service(
                Trigger, f"lights/{name}/toggle", partial(self._on_light_toggle, name),
                callback_group=self._service_cbg,
            )
            self._light_services[name] = (set_srv, toggle_srv)

        self._traffic_srv = self.create_service(
            SetTrafficLight, "lights/traffic_light", self._on_traffic_request,
            callback_group=self._service_cbg,
        )

        # --- response state ---
        self._resp_lock  = threading.Lock()
        self._resp_event = threading.Event()
        self._resp_pending_cmd: int | None = None
        self._resp_status: int | None = None

        self._publish_state()

        service_lines = "\n".join(
            f"  /lights/{name:<16} (Service) - SetBool, absolute\n"
            f"  /lights/{name}/toggle (Service) - Trigger, invert"
            for name in self._lights
        )
        self.get_logger().info(
            f"LightsCanNode ready\n"
            f"  CAN TX id=0x{self._cmd_id:03X}\n"
            f"  CAN RX id=0x{self._resp_id:03X}\n"
            f"{service_lines}\n"
            f"  /lights/traffic_light     (Service) - per-colour KEEP/OFF/ON\n"
            f"  /lights/state             (Topic)   - latched, {self._state_pub_rate} Hz + on change\n"
        )

    # =======================================================================
    # Parameters
    # =======================================================================

    def _declare_parameters(self):
        self.declare_parameter("can.cmd_id",              0x300)
        self.declare_parameter("can.resp_id",             0x301)
        self.declare_parameter("can.cmd_spotlight_on",    0x01)
        self.declare_parameter("can.cmd_spotlight_off",   0x02)
        self.declare_parameter("can.cmd_traffic_light",   0x03)
        self.declare_parameter("can.cmd_beautiful_light_on",  0x04)
        self.declare_parameter("can.cmd_beautiful_light_off", 0x05)
        self.declare_parameter("can.cmd_red_on",           0x06)
        self.declare_parameter("can.cmd_red_off",          0x07)
        self.declare_parameter("can.cmd_green_on",         0x08)
        self.declare_parameter("can.cmd_green_off",        0x09)
        self.declare_parameter("can.cmd_blue_on",          0x0A)
        self.declare_parameter("can.cmd_blue_off",         0x0B)
        self.declare_parameter("can.cmd_buzzer_on",        0x0C)
        self.declare_parameter("can.cmd_buzzer_off",       0x0D)
        self.declare_parameter("can.cmd_spotlight_left_on",   0x20)
        self.declare_parameter("can.cmd_spotlight_left_off",  0x21)
        self.declare_parameter("can.cmd_spotlight_right_on",  0x22)
        self.declare_parameter("can.cmd_spotlight_right_off", 0x23)
        self.declare_parameter("can.cmd_beautiful_1_on",   0x24)
        self.declare_parameter("can.cmd_beautiful_1_off",  0x25)
        self.declare_parameter("can.cmd_beautiful_2_on",   0x26)
        self.declare_parameter("can.cmd_beautiful_2_off",  0x27)
        self.declare_parameter("can.cmd_beautiful_3_on",   0x28)
        self.declare_parameter("can.cmd_beautiful_3_off",  0x29)
        self.declare_parameter("can.cmd_beautiful_4_on",   0x2A)
        self.declare_parameter("can.cmd_beautiful_4_off",  0x2B)
        self.declare_parameter("can.cmd_tower_on",         0x2C)
        self.declare_parameter("can.cmd_tower_off",        0x2D)
        self.declare_parameter("can.status_ok",           0x00)
        self.declare_parameter("can.status_error",        0x01)
        self.declare_parameter("timeouts.ack_s",          2.0)
        # Capped low on purpose: lights/state crosses the Wi-Fi link to the
        # ground station, and nothing downstream needs it faster than the
        # joystick repaints its light bar.
        self.declare_parameter("state_pub_rate",          2.0)

    def _load_parameters(self):
        self._cmd_id             = self.get_parameter("can.cmd_id").value
        self._resp_id            = self.get_parameter("can.resp_id").value
        self._cmd_spotlight_on   = self.get_parameter("can.cmd_spotlight_on").value
        self._cmd_spotlight_off  = self.get_parameter("can.cmd_spotlight_off").value
        self._cmd_traffic_light  = self.get_parameter("can.cmd_traffic_light").value
        self._cmd_beautiful_on   = self.get_parameter("can.cmd_beautiful_light_on").value
        self._cmd_beautiful_off  = self.get_parameter("can.cmd_beautiful_light_off").value
        self._cmd_red_on         = self.get_parameter("can.cmd_red_on").value
        self._cmd_red_off        = self.get_parameter("can.cmd_red_off").value
        self._cmd_green_on       = self.get_parameter("can.cmd_green_on").value
        self._cmd_green_off      = self.get_parameter("can.cmd_green_off").value
        self._cmd_blue_on        = self.get_parameter("can.cmd_blue_on").value
        self._cmd_blue_off       = self.get_parameter("can.cmd_blue_off").value
        self._cmd_buzzer_on      = self.get_parameter("can.cmd_buzzer_on").value
        self._cmd_buzzer_off     = self.get_parameter("can.cmd_buzzer_off").value
        self._cmd_spotlight_left_on   = self.get_parameter("can.cmd_spotlight_left_on").value
        self._cmd_spotlight_left_off  = self.get_parameter("can.cmd_spotlight_left_off").value
        self._cmd_spotlight_right_on  = self.get_parameter("can.cmd_spotlight_right_on").value
        self._cmd_spotlight_right_off = self.get_parameter("can.cmd_spotlight_right_off").value
        self._cmd_beautiful_1_on  = self.get_parameter("can.cmd_beautiful_1_on").value
        self._cmd_beautiful_1_off = self.get_parameter("can.cmd_beautiful_1_off").value
        self._cmd_beautiful_2_on  = self.get_parameter("can.cmd_beautiful_2_on").value
        self._cmd_beautiful_2_off = self.get_parameter("can.cmd_beautiful_2_off").value
        self._cmd_beautiful_3_on  = self.get_parameter("can.cmd_beautiful_3_on").value
        self._cmd_beautiful_3_off = self.get_parameter("can.cmd_beautiful_3_off").value
        self._cmd_beautiful_4_on  = self.get_parameter("can.cmd_beautiful_4_on").value
        self._cmd_beautiful_4_off = self.get_parameter("can.cmd_beautiful_4_off").value
        self._cmd_tower_on       = self.get_parameter("can.cmd_tower_on").value
        self._cmd_tower_off      = self.get_parameter("can.cmd_tower_off").value
        self._status_ok          = self.get_parameter("can.status_ok").value
        self._status_error       = self.get_parameter("can.status_error").value
        self._ack_timeout        = self.get_parameter("timeouts.ack_s").value
        self._state_pub_rate     = float(self.get_parameter("state_pub_rate").value)

    def _build_light_specs(self) -> dict:
        """Every SetBool/toggle-served light, keyed by its `lights/<name>` name.

        `spotlight` and `tower` are the only entries whose `fields` names more
        than one LightsState field - they drive several pins from one CAN
        command, same as the firmware does.
        """
        return {
            "spotlight": LightSpec(
                "SPOTLIGHT", self._cmd_spotlight_on, self._cmd_spotlight_off,
                ("spotlight_left", "spotlight_right"),
                lambda s: not (s.spotlight_left and s.spotlight_right)),
            "spotlight_left": LightSpec(
                "SPOTLIGHT_LEFT", self._cmd_spotlight_left_on, self._cmd_spotlight_left_off,
                ("spotlight_left",), lambda s: not s.spotlight_left),
            "spotlight_right": LightSpec(
                "SPOTLIGHT_RIGHT", self._cmd_spotlight_right_on, self._cmd_spotlight_right_off,
                ("spotlight_right",), lambda s: not s.spotlight_right),
            "beautiful": LightSpec(
                "BEAUTIFUL_LIGHT", self._cmd_beautiful_on, self._cmd_beautiful_off,
                ("beautiful",), lambda s: not s.beautiful),
            "beautiful_1": LightSpec(
                "BEAUTIFUL_1", self._cmd_beautiful_1_on, self._cmd_beautiful_1_off,
                ("beautiful_1",), lambda s: not s.beautiful_1),
            "beautiful_2": LightSpec(
                "BEAUTIFUL_2", self._cmd_beautiful_2_on, self._cmd_beautiful_2_off,
                ("beautiful_2",), lambda s: not s.beautiful_2),
            "beautiful_3": LightSpec(
                "BEAUTIFUL_3", self._cmd_beautiful_3_on, self._cmd_beautiful_3_off,
                ("beautiful_3",), lambda s: not s.beautiful_3),
            "beautiful_4": LightSpec(
                "BEAUTIFUL_4", self._cmd_beautiful_4_on, self._cmd_beautiful_4_off,
                ("beautiful_4",), lambda s: not s.beautiful_4),
            "traffic_red": LightSpec(
                "TRAFFIC_RED", self._cmd_red_on, self._cmd_red_off,
                ("traffic_red",), lambda s: not s.traffic_red),
            "traffic_green": LightSpec(
                "TRAFFIC_GREEN", self._cmd_green_on, self._cmd_green_off,
                ("traffic_green",), lambda s: not s.traffic_green),
            "traffic_blue": LightSpec(
                "TRAFFIC_BLUE", self._cmd_blue_on, self._cmd_blue_off,
                ("traffic_blue",), lambda s: not s.traffic_blue),
            "buzzer": LightSpec(
                "BUZZER", self._cmd_buzzer_on, self._cmd_buzzer_off,
                ("buzzer",), lambda s: not s.buzzer),
            "tower": LightSpec(
                "TOWER", self._cmd_tower_on, self._cmd_tower_off,
                ("traffic_red", "traffic_green", "traffic_blue"),
                lambda s: not (s.traffic_red and s.traffic_green and s.traffic_blue)),
        }

    # =======================================================================
    # State topic
    # =======================================================================

    def _publish_state(self):
        """Publish the current state. Also the heartbeat, on a timer.

        Republishing unchanged state is not redundancy: it is the only thing
        that tells a console the value on screen is still live. Without it a
        dead lights_can_node leaves the last latched state up forever. It also
        makes the topic usable from a VOLATILE subscriber - the ground station
        UI reaches it through rosbridge, which need not match TRANSIENT_LOCAL.
        """
        state = self._state
        self._state_dirty = False
        self._last_state_publish = self._now_seconds()

        msg = LightsStateMsg()
        msg.spotlight_left  = state.spotlight_left
        msg.spotlight_right = state.spotlight_right
        msg.beautiful       = state.beautiful
        msg.beautiful_1     = state.beautiful_1
        msg.beautiful_2     = state.beautiful_2
        msg.beautiful_3     = state.beautiful_3
        msg.beautiful_4     = state.beautiful_4
        msg.traffic_red     = state.traffic_red
        msg.traffic_green   = state.traffic_green
        msg.traffic_blue    = state.traffic_blue
        msg.buzzer          = state.buzzer
        self._state_pub.publish(msg)

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _publish_change(self):
        """Push a real change out now instead of waiting for the heartbeat.

        Waiting costs up to a full period, which an operator sees as the
        console lagging the switch they just flipped.
        """
        if self._now_seconds() - self._last_state_publish < self._state_min_period:
            return
        self._publish_state()

    def _commit(self, state: LightsState):
        """Adopt a state the ESP32 has confirmed, and announce it."""
        if state == self._state:
            return
        self._state = state
        self._state_dirty = True
        self._publish_change()

    # =======================================================================
    # Services
    # =======================================================================

    def _on_light_set(self, name: str, request, response):
        return self._apply_light(
            self._lights[name], lambda _state: request.data, response)

    def _on_light_toggle(self, name: str, request, response):
        spec = self._lights[name]
        return self._apply_light(spec, spec.toggle_from, response)

    def _apply_light(self, spec: LightSpec, desired_from, response):
        """Drive one light, or a group of pins one CAN command fans out to.

        `desired_from` reads the target off the current state, which is what
        makes a toggle atomic: the read and the write both happen inside
        _can_lock, so two toggles racing cannot both see the same 'before'.
        """
        with self._can_lock:
            desired = bool(desired_from(self._state))
            state_str = "ON" if desired else "OFF"
            self.get_logger().info(f"Service call: {spec.label} {state_str}")

            cmd = spec.on_cmd if desired else spec.off_cmd
            ok, status = self._send_and_wait(cmd, [cmd])

            if ok and status == self._status_ok:
                changes = {field: desired for field in spec.fields}
                self._commit(replace(self._state, **changes))
                response.success = True
                response.message = f"{spec.label} {state_str} OK"
            else:
                response.success = False
                response.message = (
                    f"{spec.label} {state_str} FAIL"
                    + (" (timeout)" if not ok else " (ESP32 error)")
                )

        return response

    def _on_traffic_request(self, request, response):
        with self._can_lock:
            try:
                desired = resolve_traffic(
                    self._state,
                    red=request.red,
                    green=request.green,
                    blue=request.blue,
                )
            except ValueError as exc:
                response.success = False
                response.message = f"TRAFFIC_LIGHT bad request: {exc}"
                self.get_logger().warn(response.message)
                return response

            mask = traffic_mask(desired)
            self.get_logger().info(
                f"Service call: TRAFFIC_LIGHT mask=0x{mask:02X} "
                f"({describe_traffic(desired)})"
            )

            ok, status = self._send_and_wait(
                self._cmd_traffic_light, [self._cmd_traffic_light, mask])

            if ok and status == self._status_ok:
                self._commit(desired)
                response.success = True
                response.message = (
                    f"TRAFFIC_LIGHT OK mask=0x{mask:02X} ({describe_traffic(desired)})")
            else:
                response.success = False
                response.message = "TRAFFIC_LIGHT FAIL" + (
                    " (timeout)" if not ok else " (ESP32 error)")

        return response

    def turn_off_traffic_light(self):
        """Best-effort: send TRAFFIC_LIGHT off on the way out.

        Called from main()'s finally block, after the executor has already
        stopped spinning - so this does not wait for an ACK the way
        _on_traffic_request does, since nothing is left to deliver one on
        _on_can_msg. A killed lights_can_node must not leave the tower lit
        with whatever colour was last commanded; nothing else on the rover
        clears it, and gs_link_lamp_node / drive_source_lamp_node only ever
        change the lamp, they never turn it off on their own exit either.
        """
        with self._can_lock:
            self._send_cmd(self._cmd_id, [self._cmd_traffic_light, 0])
            self._commit(replace(
                self._state,
                traffic_red=False, traffic_green=False, traffic_blue=False))

    # =======================================================================
    # CAN helpers
    # =======================================================================

    def _send_and_wait(self, expected_cmd: int, data: list) -> tuple[bool, int | None]:
        """Send a CAN command and wait for ACK. Returns (got_reply, status).

        Callers hold _can_lock, so only one transaction is ever outstanding —
        _resp_pending_cmd belongs to this call and nobody else's.
        """
        with self._resp_lock:
            self._resp_pending_cmd = expected_cmd
            self._resp_status = None
            self._resp_event.clear()

        self._send_cmd(self._cmd_id, data)

        got_reply = self._resp_event.wait(timeout=self._ack_timeout)

        with self._resp_lock:
            status = self._resp_status
            self._resp_pending_cmd = None

        if not got_reply:
            self.get_logger().warn(f"No ACK from ESP32 (cmd=0x{expected_cmd:02X})")

        return got_reply, status

    def _send_cmd(self, can_id: int, data: list):
        msg = Frame()
        msg.id          = can_id
        msg.dlc         = len(data)
        msg.data        = data + [0] * (8 - len(data))
        msg.is_extended = False
        msg.is_rtr      = False
        msg.is_error    = False
        self._pub.publish(msg)
        self.get_logger().info(
            f"TX CAN 0x{can_id:03X}  data={[f'0x{b:02X}' for b in data]}"
        )

    def _on_can_msg(self, msg: Frame):
        if msg.id != self._resp_id or msg.dlc < 2:
            return

        cmd    = msg.data[0]
        status = msg.data[1]

        if cmd not in self._own_cmds:
            return  # another node's command echoed back, not ours to read

        self.get_logger().info(
            f"RX CAN 0x{self._resp_id:03X}  cmd=0x{cmd:02X}  status=0x{status:02X}"
        )

        with self._resp_lock:
            if self._resp_pending_cmd is None or cmd != self._resp_pending_cmd:
                expected_str = (
                    f"0x{self._resp_pending_cmd:02X}"
                    if self._resp_pending_cmd is not None
                    else "None"
                )
                self.get_logger().warn(
                    f"Unexpected response cmd=0x{cmd:02X}, "
                    f"expected={expected_str}, ignoring"
                )
                return
            self._resp_status = status
            self._resp_event.set()


# ===========================================================================
# main
# ===========================================================================

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    executor = MultiThreadedExecutor(num_threads=3)
    node = LightsCanNode()
    executor.add_node(node)

    def _stop(signum, frame):
        executor.shutdown()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.turn_off_traffic_light()
            time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
