#!/usr/bin/env python3
"""The power sensor roster: four parallel parameter lists turned into one
validated list of sensors.

Each INA228 on the lights ESP32 is described by four numbers that live in four
separate ROS parameters — its telemetry CAN id, its enable command, its
disable command, and the topic name it publishes under. Nothing in the
parameter file ties one row to the next, so a sensor added to three lists and
forgotten in the fourth silently shifts every sensor after it onto another
sensor's commands. That failure is invisible on the bus: the frames are
well-formed and the wrong sensor answers. Catching it at load time is the
whole point of this module.

No ROS import anywhere in here — power_sensors is deliberately standalone.
"""

from dataclasses import dataclass, field


@dataclass
class SensorConfig:
    """One INA228: where its telemetry lands and how it is switched."""

    name: str
    can_id: int
    cmd_enable: int
    cmd_disable: int
    #: Set once the ESP32 has ACKed a command. The firmware boots with every
    #: sensor off, so False is the truth until we hear otherwise.
    enabled: bool = False
    #: Filled in by the node once it has a publisher to hand.
    publisher: object = field(default=None, repr=False, compare=False)


def build_sensor_configs(names, can_ids, cmd_enable, cmd_disable):
    """Zip the four parameter lists into SensorConfigs, or raise ValueError.

    Raises rather than dropping the odd row: a rover that comes up monitoring
    the wrong sensor is worse than one that refuses to come up at all.
    """
    lengths = {
        "sensor_names": len(names),
        "sensor_can_ids": len(can_ids),
        "sensor_cmd_enable": len(cmd_enable),
        "sensor_cmd_disable": len(cmd_disable),
    }
    if len(set(lengths.values())) != 1:
        raise ValueError(
            "sensor_names, sensor_can_ids, sensor_cmd_enable and "
            "sensor_cmd_disable must have the same length, got "
            + ", ".join(f"{k} ({v})" for k, v in lengths.items())
        )

    if not names:
        raise ValueError("no power sensors configured")

    _reject_duplicates(names, "sensor_names", str)
    _reject_duplicates(can_ids, "sensor_can_ids", hex)

    # A command byte reused across sensors means one service silently drives
    # two sensors, so the two disagree with the node's idea of their state.
    _reject_duplicates(
        list(cmd_enable) + list(cmd_disable),
        "sensor_cmd_enable/sensor_cmd_disable", hex)

    return [
        SensorConfig(name=n, can_id=i, cmd_enable=on, cmd_disable=off)
        for n, i, on, off in zip(names, can_ids, cmd_enable, cmd_disable)
    ]


def _reject_duplicates(values, label, fmt):
    dupes = {v for v in values if values.count(v) > 1}
    if dupes:
        raise ValueError(
            f"{label} contains duplicates: {sorted(fmt(d) for d in dupes)}")
