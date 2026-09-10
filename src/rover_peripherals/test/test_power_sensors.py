"""The power sensor roster: what a mismatched parameter file is not allowed
to produce.

Each INA228 is described across four parallel ROS parameter lists, and nothing
in the YAML ties one row to the next. A sensor added to three of them and
forgotten in the fourth shifts every later sensor onto another sensor's
commands — well-formed frames, wrong sensor answering, nothing to see on the
bus. These tests pin the load-time refusal that keeps that off the rover.

No ROS import anywhere in here — power_sensors is deliberately standalone.
"""

import pytest

from rover_peripherals.power_sensors import SensorConfig, build_sensor_configs


NAMES = ["sensor_rover", "sensor_arm"]
IDS = [0x302, 0x303]
ON = [0x12, 0x14]
OFF = [0x13, 0x15]


def test_builds_one_config_per_sensor():
    configs = build_sensor_configs(NAMES, IDS, ON, OFF)

    assert [c.name for c in configs] == NAMES
    assert [c.can_id for c in configs] == IDS
    assert [c.cmd_enable for c in configs] == ON
    assert [c.cmd_disable for c in configs] == OFF


def test_sensors_start_disabled():
    # The firmware boots with every sensor off, so the node must not claim
    # otherwise before it has ACKed an enable.
    assert all(not c.enabled for c in build_sensor_configs(NAMES, IDS, ON, OFF))


def test_rejects_a_sensor_missing_from_one_list():
    # The failure this module exists for: three lists grew, one did not.
    with pytest.raises(ValueError, match="same length"):
        build_sensor_configs(
            NAMES + ["sensor_science"], IDS + [0x304], ON + [0x16], OFF)


def test_rejects_empty_roster():
    with pytest.raises(ValueError, match="no power sensors"):
        build_sensor_configs([], [], [], [])


def test_rejects_duplicate_names():
    # Two sensors on one topic: the second publisher silently wins.
    with pytest.raises(ValueError, match="sensor_names"):
        build_sensor_configs(["a", "a"], IDS, ON, OFF)


def test_rejects_duplicate_can_ids():
    # One telemetry id cannot mean two sensors - the node looks sensors up by
    # it, so one of them would never be published at all.
    with pytest.raises(ValueError, match="sensor_can_ids"):
        build_sensor_configs(NAMES, [0x302, 0x302], ON, OFF)


def test_rejects_a_command_reused_across_sensors():
    # Sharing a command byte makes one service drive two sensors, leaving the
    # node's idea of the other sensor's state wrong with no error anywhere.
    with pytest.raises(ValueError, match="sensor_cmd_enable"):
        build_sensor_configs(NAMES, IDS, [0x12, 0x12], OFF)


def test_rejects_enable_colliding_with_a_disable():
    with pytest.raises(ValueError, match="sensor_cmd_enable"):
        build_sensor_configs(NAMES, IDS, [0x12, 0x15], OFF)


def test_duplicate_report_names_the_offending_values():
    with pytest.raises(ValueError) as exc:
        build_sensor_configs(NAMES, [0x302, 0x302], ON, OFF)

    assert "0x302" in str(exc.value)


def test_config_carries_no_publisher_until_the_node_sets_one():
    config, = build_sensor_configs(["solo"], [0x302], [0x12], [0x13])

    assert config.publisher is None
    assert config == SensorConfig("solo", 0x302, 0x12, 0x13)
