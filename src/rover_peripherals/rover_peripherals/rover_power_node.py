#!/usr/bin/env python3
"""
ROS2 node - power (voltage/current) monitoring via CAN bus (ros2_socketcan).

Listens on /from_can_bus for frames coming from the two power sensors and
republishes decoded voltage/current as sensor_msgs/msg/BatteryState.

The firmware boots with both sensors OFF and polls each one only while it is
enabled, so this node also owns the enable side of the protocol: it enables
its sensors on start (see `enable_on_start`), exposes a SetBool per sensor so
an operator can silence one that is faulty or unpopulated, and disables them
again on the way out so a dead node does not leave the ESP32 pushing frames
nobody reads.

CAN TX (PC -> ESP32)  ID can.cmd_id:
  byte 0 = cmd_telemetry_all_enable | cmd_telemetry_all_disable  (both sensors)
           or that sensor's sensor_cmd_enable | sensor_cmd_disable

CAN RX (ESP32 -> PC):
  ID can.resp_id     byte 0 = echoed cmd, byte 1 = 0x00 OK | 0x01 ERROR
  ID sensor_can_id   byte 0-3 = current (float32, little-endian, amps)
                      byte 4-7 = voltage (float32, little-endian, volts)
  Values are already physical units (ESP converts via INA228 LSB), no
  scaling needed on this side.

Publishes:
  <topic_prefix>/<sensor_name>   sensor_msgs/msg/BatteryState
    .voltage  -> volts
    .current  -> amps
    .location -> sensor_name (identifies which physical sensor this is)

Services:
  <topic_prefix>/<sensor_name>/enable   std_srvs/srv/SetBool  - one sensor
  <topic_prefix>/enable                 std_srvs/srv/SetBool  - all sensors
"""

import math
import struct
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from can_msgs.msg import Frame
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool

from rover_peripherals.power_sensors import SensorConfig, build_sensor_configs


class PowerMonitorCanNode(Node):
    def __init__(self):
        super().__init__("power_monitor_can_node")

        self._declare_parameters()
        self._load_parameters()

        # The CAN subscription gets its own group so it stays free to deliver
        # the very ACK a blocked service is waiting for - sharing a group with
        # the services would deadlock every call until the timeout.
        self._sub_cbg     = MutuallyExclusiveCallbackGroup()
        self._service_cbg = MutuallyExclusiveCallbackGroup()

        # Serialises a whole send -> wait -> commit transaction, so
        # _resp_pending_cmd always belongs to exactly one caller.
        self._can_lock   = threading.Lock()
        self._resp_lock  = threading.Lock()
        self._resp_event = threading.Event()
        self._resp_pending_cmd: int | None = None
        self._resp_status: int | None = None

        self._can_pub = self.create_publisher(Frame, "to_can_bus", 10)
        self._can_sub = self.create_subscription(
            Frame, "from_can_bus", self._on_can_msg, 10,
            callback_group=self._sub_cbg,
        )

        # --- one publisher per sensor, indexed by CAN id for O(1) lookup ---
        self._sensors_by_id = {}
        for cfg in self._sensor_configs:
            cfg.publisher = self.create_publisher(
                BatteryState,
                f"{self._topic_prefix}/{cfg.name}",
                QoSPresetProfiles.SENSOR_DATA.value,
            )
            self._sensors_by_id[cfg.can_id] = cfg

        # --- Services ---
        self._sensor_services = {
            cfg.name: self.create_service(
                SetBool,
                f"{self._topic_prefix}/{cfg.name}/enable",
                lambda req, resp, name=cfg.name: self._on_sensor_enable(name, req, resp),
                callback_group=self._service_cbg,
            )
            for cfg in self._sensor_configs
        }
        self._all_service = self.create_service(
            SetBool, f"{self._topic_prefix}/enable", self._on_all_enable,
            callback_group=self._service_cbg,
        )

        # Enabling has to wait until the executor is spinning, otherwise
        # nothing is left to deliver the ACK we would block on. The timer
        # repeats rather than firing once because the ESP32 and the Jetson
        # boot together: the first attempt can easily land before the firmware
        # is answering, and a single shot would leave telemetry off for good.
        self._startup_timer = None
        self._startup_attempts = 0
        if self._enable_on_start:
            self._startup_timer = self.create_timer(
                self._enable_on_start_delay_s,
                self._on_startup_enable,
                callback_group=self._service_cbg,
            )

        sensors_desc = "\n".join(
            f"    0x{cfg.can_id:03X} -> {self._topic_prefix}/{cfg.name}"
            f"  (on=0x{cfg.cmd_enable:02X} off=0x{cfg.cmd_disable:02X})"
            for cfg in self._sensor_configs
        )
        self.get_logger().info(
            f"PowerMonitorCanNode ready, listening on /from_can_bus\n"
            f"  CAN TX id=0x{self._cmd_id:03X}\n"
            f"  CAN RX id=0x{self._resp_id:03X}\n"
            f"  sensors:\n{sensors_desc}\n"
            f"  /{self._topic_prefix}/enable (Service) - SetBool, all sensors"
        )


    def _declare_parameters(self):
        self.declare_parameter("topic_prefix", "power_monitor")
        self.declare_parameter("sensor_names", ["sensor_rover", "sensor_arm"])
        self.declare_parameter("sensor_can_ids", [0x302, 0x303])
        self.declare_parameter("sensor_cmd_enable", [0x12, 0x14])
        self.declare_parameter("sensor_cmd_disable", [0x13, 0x15])
        self.declare_parameter("can.cmd_id", 0x300)
        self.declare_parameter("can.resp_id", 0x301)
        self.declare_parameter("can.cmd_telemetry_all_enable", 0x10)
        self.declare_parameter("can.cmd_telemetry_all_disable", 0x11)
        self.declare_parameter("can.status_ok", 0x00)
        self.declare_parameter("can.status_error", 0x01)
        self.declare_parameter("enable_on_start", True)
        self.declare_parameter("enable_on_start_delay_s", 1.0)
        self.declare_parameter("enable_on_start_retries", 10)
        self.declare_parameter("disable_on_shutdown", True)
        self.declare_parameter("timeouts.ack_s", 2.0)

    def _load_parameters(self):
        self._topic_prefix = self.get_parameter("topic_prefix").value
        names = self.get_parameter("sensor_names").value
        ids = self.get_parameter("sensor_can_ids").value
        on_cmds = self.get_parameter("sensor_cmd_enable").value
        off_cmds = self.get_parameter("sensor_cmd_disable").value

        self._sensor_configs = build_sensor_configs(names, ids, on_cmds, off_cmds)
        self._sensors_by_name = {cfg.name: cfg for cfg in self._sensor_configs}

        self._cmd_id       = self.get_parameter("can.cmd_id").value
        self._resp_id      = self.get_parameter("can.resp_id").value
        self._cmd_all_on   = self.get_parameter("can.cmd_telemetry_all_enable").value
        self._cmd_all_off  = self.get_parameter("can.cmd_telemetry_all_disable").value
        self._status_ok    = self.get_parameter("can.status_ok").value
        self._status_error = self.get_parameter("can.status_error").value
        self._enable_on_start = bool(self.get_parameter("enable_on_start").value)
        self._enable_on_start_delay_s = float(
            self.get_parameter("enable_on_start_delay_s").value)
        self._enable_on_start_retries = max(
            1, int(self.get_parameter("enable_on_start_retries").value))
        self._disable_on_shutdown = bool(
            self.get_parameter("disable_on_shutdown").value)
        self._ack_timeout = float(self.get_parameter("timeouts.ack_s").value)


    # =======================================================================
    # Telemetry enable/disable
    # =======================================================================

    def _stop_startup_timer(self):
        self._startup_timer.cancel()
        self.destroy_timer(self._startup_timer)
        self._startup_timer = None

    def _on_startup_enable(self):
        """Turn the sensors on once the executor is up, retrying until the
        firmware answers or `enable_on_start_retries` attempts are spent."""
        self._startup_attempts += 1

        ok, message = self._set_all(True)
        if ok:
            self._stop_startup_timer()
            self.get_logger().info(
                f"Power telemetry enabled on start: {message} "
                f"(attempt {self._startup_attempts})"
            )
            return

        if self._startup_attempts >= self._enable_on_start_retries:
            self._stop_startup_timer()
            self.get_logger().error(
                f"Could not enable power telemetry after "
                f"{self._startup_attempts} attempts: {message}. "
                f"Call /{self._topic_prefix}/enable to retry."
            )
            return

        self.get_logger().warn(
            f"Enabling power telemetry failed ({message}), "
            f"retrying (attempt {self._startup_attempts} of "
            f"{self._enable_on_start_retries})"
        )

    def _on_sensor_enable(self, name: str, request, response):
        response.success, response.message = self._set_sensor(
            self._sensors_by_name[name], bool(request.data))
        return response

    def _on_all_enable(self, request, response):
        response.success, response.message = self._set_all(bool(request.data))
        return response

    def _set_sensor(self, cfg: SensorConfig, enabled: bool) -> tuple[bool, str]:
        """Enable or disable one sensor. Returns (success, message)."""
        state_str = "ON" if enabled else "OFF"
        cmd = cfg.cmd_enable if enabled else cfg.cmd_disable

        with self._can_lock:
            self.get_logger().info(f"Service call: {cfg.name} telemetry {state_str}")
            ok, status = self._send_and_wait(cmd, [cmd])

            if ok and status == self._status_ok:
                cfg.enabled = enabled
                return True, f"{cfg.name} telemetry {state_str} OK"

        return False, (
            f"{cfg.name} telemetry {state_str} FAIL"
            + (" (timeout)" if not ok else " (ESP32 error)")
        )

    def _set_all(self, enabled: bool) -> tuple[bool, str]:
        """Enable or disable every sensor with the single broadcast command.

        The firmware's 0x10/0x11 act on all of its sensors at once, so this is
        one frame rather than one per sensor - but it also reaches sensors this
        node was not configured for, which is the intended meaning of "all".
        """
        state_str = "ON" if enabled else "OFF"
        cmd = self._cmd_all_on if enabled else self._cmd_all_off

        with self._can_lock:
            self.get_logger().info(f"Service call: all telemetry {state_str}")
            ok, status = self._send_and_wait(cmd, [cmd])

            if ok and status == self._status_ok:
                for cfg in self._sensor_configs:
                    cfg.enabled = enabled
                return True, f"all telemetry {state_str} OK"

        return False, (
            f"all telemetry {state_str} FAIL"
            + (" (timeout)" if not ok else " (ESP32 error)")
        )

    def disable_telemetry_on_exit(self):
        """Best-effort: silence the sensors on the way out.

        Called from main()'s finally block, after the executor has stopped
        spinning - so unlike the services it does not wait for an ACK, since
        nothing is left to deliver one on _on_can_msg. Without this the ESP32
        keeps pushing a frame per sensor every 200 ms at nobody.
        """
        if not self._disable_on_shutdown:
            return

        with self._can_lock:
            self._send_cmd(self._cmd_id, [self._cmd_all_off])
            for cfg in self._sensor_configs:
                cfg.enabled = False


    # =======================================================================
    # CAN helpers
    # =======================================================================

    def _send_and_wait(self, expected_cmd: int, data: list) -> tuple[bool, int | None]:
        """Send a CAN command and wait for its ACK. Returns (got_reply, status).

        Callers hold _can_lock, so only one transaction is ever outstanding -
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
        self._can_pub.publish(msg)
        self.get_logger().info(
            f"TX CAN 0x{can_id:03X}  data={[f'0x{b:02X}' for b in data]}"
        )

    def _on_can_msg(self, msg: Frame):
        if msg.id == self._resp_id:
            self._on_response(msg)
            return

        cfg = self._sensors_by_id.get(msg.id)
        if cfg is None:
            return  # not a frame we care about

        self._on_measurement(cfg, msg)

    def _on_response(self, msg: Frame):
        """Land an ACK, if it is the one _send_and_wait is blocked on.

        lights_can_node commands the same ESP32 over the same id pair, so most
        traffic here answers somebody else - matching on the echoed command
        byte is what keeps the two nodes from stealing each other's ACKs.
        """
        if msg.dlc < 2:
            return

        cmd, status = msg.data[0], msg.data[1]

        with self._resp_lock:
            if self._resp_pending_cmd is None or cmd != self._resp_pending_cmd:
                return  # somebody else's ACK
            self._resp_status = status
            self._resp_event.set()

        self.get_logger().info(
            f"RX CAN 0x{self._resp_id:03X}  cmd=0x{cmd:02X}  status=0x{status:02X}"
        )

    def _on_measurement(self, cfg: SensorConfig, msg: Frame):
        if msg.dlc < 8:
            self.get_logger().warn(
                f"RX CAN 0x{msg.id:03X} ({cfg.name}): dlc={msg.dlc} < 8, ignoring"
            )
            return

        current, voltage = struct.unpack_from("<ff", bytes(msg.data), 0)

        if not (math.isfinite(current) and math.isfinite(voltage)):
            self.get_logger().warn(
                f"RX CAN 0x{msg.id:03X} ({cfg.name}): non-finite value, ignoring"
            )
            return

        self.get_logger().debug(
            f"RX CAN 0x{msg.id:03X} ({cfg.name})  "
            f"voltage={voltage:.3f} V  current={current:.3f} A"
        )

        state = BatteryState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.voltage = float(voltage)
        state.current = float(current)
        state.location = cfg.name
        state.present = True
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN

        cfg.publisher.publish(state)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=3)
    node = PowerMonitorCanNode()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.disable_telemetry_on_exit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
