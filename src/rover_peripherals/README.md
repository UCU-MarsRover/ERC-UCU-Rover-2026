# Rover Peripherals

## Container (ESP)

### Launch file & prerequisites

> ⚠️ Warning: This launch files assumes that ros2_socketcan is already running!

```bash
ros2 launch rover_bringup container.launch.py
```

### ROS2 Interface details

**Action**:  /container/lid  ([indomitus_interfaces/action/ContainerLid](../indomitus_interfaces/action/ContainerLid.action))

  goal.open = True  -> open lid   (CAN cmd_id, byte 0 = cmd_open)  
  goal.open = False -> close lid  (CAN cmd_id, byte 0 = cmd_close)  
  feedback.status   -> "opening" | "closing" | "done" | "error"  
  result.success    -> True if ESP32 finished OK within timeout  
  result.message    -> human-readable status  

**Service**: /container/weight  ([indomitus_interfaces/srv/GetWeight](../indomitus_interfaces/srv/GetWeight.srv))
  request:  (empty)
  response: float32 weight, bool success, string message

### CAN interface

Used IDs: [config/container_can.yaml](config/container_can.yaml)
## Lights (ESP)

### Launch file & prerequisites

> ⚠️ Warning: This launch file assumes that ros2_socketcan is already running!

```bash
ros2 launch rover_peripherals lighting.launch.py
```

Included by `rover_bringup/launch/rover.launch.py`.

### ROS2 Interface details

`lights_can_node` **owns** the light state. Two operators command the same
lights and they ask differently — the onboard joystick has momentary buttons,
the ground station has latching switches — so neither keeps a copy of its own,
and neither can drift out of step with the other. Both read the truth back off
`/lights/state`.

**Services** — absolute forms for the ground station's switches, `/toggle`
forms for the joystick's buttons:

| Service | Type | Meaning |
|---|---|---|
| `/lights/spotlight` | `std_srvs/SetBool` | set both spotlight pins together |
| `/lights/spotlight_left` | `std_srvs/SetBool` | set the left spotlight pin only |
| `/lights/spotlight_right` | `std_srvs/SetBool` | set the right spotlight pin only |
| `/lights/beautiful` | `std_srvs/SetBool` | set the decorative animation (all 4 pins) |
| `/lights/beautiful_1` .. `/lights/beautiful_4` | `std_srvs/SetBool` | set one decorative pin, static |
| `/lights/traffic_red` | `std_srvs/SetBool` | set the traffic-head red pin only |
| `/lights/traffic_green` | `std_srvs/SetBool` | set the traffic-head green pin only |
| `/lights/traffic_blue` | `std_srvs/SetBool` | set the traffic-head blue pin only |
| `/lights/buzzer` | `std_srvs/SetBool` | set the buzzer |
| `/lights/tower` | `std_srvs/SetBool` | set all three traffic-head pins together |
| `/lights/traffic_light` | [`indomitus_interfaces/SetTrafficLight`](../indomitus_interfaces/srv/SetTrafficLight.srv) | set any subset of the traffic-head colours |

Every `/lights/<name>` service above also has a matching
`/lights/<name>/toggle` (`std_srvs/Trigger`) that inverts it. `/lights/beautiful_1`
through `_4` fight the `/lights/beautiful` animation if it is running — that is
a firmware quirk, not something this node papers over.

`SetTrafficLight` is tri-state per colour: `KEEP=0`, `OFF=1`, `ON=2`. `KEEP`
is zero so a request only ever changes the colours it names — an empty request
is a no-op, and switching blue does not disturb red. The head is red/green/blue
only; the firmware has no yellow LED.

```bash
ros2 service call /lights/traffic_light indomitus_interfaces/srv/SetTrafficLight "{blue: 2}"
ros2 service call /lights/traffic_light indomitus_interfaces/srv/SetTrafficLight "{red: 1}"
```

The traffic light is meant to signal the rover's own state, so nothing on the
joystick or the ground station panel is bound to it. What drives it from rover
state is not built yet — this is the interface it will use.

**Topic**: `/lights/state` ([indomitus_interfaces/msg/LightsState](../indomitus_interfaces/msg/LightsState.msg))

Latched (`TRANSIENT_LOCAL`, depth 1), published on change and rate limited to
`state_pub_rate` (2 Hz) so it does not load the Wi-Fi link to the ground
station. There is no periodic heartbeat, which is exactly why it is latched: a
UI that connects late gets the current state immediately.

```bash
ros2 topic echo /lights/state
```

### CAN interface

Used IDs: [config/lights.yaml](config/lights.yaml)

Traffic-light bitmask is `R=bit0 Y=bit1 G=bit2 B=bit3`, computed in the node
from its own state — the firmware protocol did not change.

`0x300` / `0x301` are shared with `power_node` (same ESP32, see below). Both
nodes match on the echoed command byte and ignore ACKs for commands they did
not send, so neither steals the other's reply.

## Power monitor (Lights ESP)

The two INA228 current sensors live on the same ESP32 as the lights, so
`power_node` and `lights_can_node` share one CAN command/response id pair
(`0x300` / `0x301`) and tell their traffic apart by the echoed command byte.

### Launch file & prerequisites

> ⚠️ Warning: This launch file assumes that ros2_socketcan is already running!

```bash
ros2 launch rover_peripherals power_monitor_node.launch.py
```

Included by `rover_bringup/launch/rover.launch.py`.

### ROS2 Interface details

The firmware boots with **both sensors off** and polls each one only while it
is enabled, so `power_node` owns the enable side too: it turns its sensors on
`enable_on_start_delay_s` after start, and turns them off again on exit so a
stopped node does not leave the ESP32 pushing frames nobody reads.

**Topics** — one `sensor_msgs/BatteryState` per sensor, at 5 Hz while enabled:

| Topic | Sensor |
|---|---|
| `/power_monitor/sensor_rover` | INA228 #1, I2C `0x45`, CAN `0x302` |
| `/power_monitor/sensor_arm` | INA228 #2, I2C `0x44`, CAN `0x303` |

`.voltage` is volts, `.current` is amps — the ESP32 has already applied the
INA228 LSB, so nothing is scaled on this side. `.location` carries the sensor
name, which is what tells two otherwise identical messages apart if they are
merged downstream.

**Services** (`std_srvs/SetBool`) — for silencing a sensor that is faulty or
unpopulated, and for cutting the telemetry when the bus is busy:

| Service | Meaning |
|---|---|
| `/power_monitor/sensor_rover/enable` | enable/disable sensor 1 only |
| `/power_monitor/sensor_arm/enable` | enable/disable sensor 2 only |
| `/power_monitor/enable` | enable/disable every sensor in one frame |

```bash
ros2 service call /power_monitor/sensor_arm/enable std_srvs/srv/SetBool "{data: false}"
ros2 service call /power_monitor/enable std_srvs/srv/SetBool "{data: true}"
```

A disabled sensor simply stops publishing; the topic stays advertised and the
node stays healthy. The QoS is `SENSOR_DATA` (best-effort, volatile, not
latched), so a consumer that needs to tell "off" from "dead" should watch the
message timestamps rather than the topic list.

### CAN interface

Used IDs: [config/power_node.yaml](config/power_node.yaml)
