# smart_waste_nav

`smart_waste_nav` now supports three navigation workflows on top of `move_base`:

1. Sequential node execution from `config/navigation_nodes.yaml`
2. Alarm-driven multi-node routing with optimal visit ordering
3. Recording traveled poses into a reusable node graph

## Node config

Nodes are defined in `config/navigation_nodes.yaml`.

Required fields:

- `id`
- `x`
- `y`
- `yaw`

Optional fields:

- `name`
- `aliases`
- `lat`
- `lng`
- `mqtt_id`
- `clear_id`
- `active`
- `neighbors`

`neighbors` defines the node graph used by the alarm route planner. If no neighbors are defined, the planner falls back to full Euclidean connectivity.

## Launch files

Sequential navigation with existing nodes:

```bash
roslaunch smart_waste_nav waypoint_executor.launch
```

Alarm-driven demo with the current built-in nodes:

```bash
roslaunch smart_waste_nav alarm_route_demo.launch
```

Alarm-driven execution from MQTT or ROS alarms:

```bash
roslaunch smart_waste_nav alarm_route_executor.launch
```

Record nodes while driving on the map:

```bash
roslaunch smart_waste_nav node_path_recorder.launch
```

## Alarm input

The alarm executor can read alarms from:

- ROS `std_msgs/String` on `/waste_alarm`
- MQTT / Mosquitto topic `bin/status`

MQTT is enabled by default in `alarm_route_executor.launch`.
When the robot reaches an alarm node successfully, it publishes a clear message to `bin/cleared` so the ESP32 side or the mock publisher can drop the latched alarm.

Recommended MQTT topology:

- Robot PC: Mosquitto broker + `alarm_route_executor`
- ESP32: publishes only the real sensor node to `bin/status`
- Mock publisher PC: publishes nodes `2-7` directly to the same `bin/status`

`alarm_route_executor` now filters `bin/status` by explicit `alarm` and `isFull` fields first, so regular status packets with `alarm: false` do not create navigation jobs.

The payload is expected to be either a single JSON object, a JSON array, or:

```json
{
  "alarms": [
    {
      "name": "Test Node",
      "lat": 39.8721,
      "lng": 32.7352,
      "weightKg": 12.5,
      "fillPercent": 70,
      "isFull": false
    }
  ]
}
```

Matching order:

1. Exact `name` or `aliases` match against a configured node
2. Nearest configured node by `lat/lng`

For real GPS-based matching, fill `lat` and `lng` in `navigation_nodes.yaml`. Until then, direct testing is easiest through `alarm_route_demo.launch` or by sending alarm names that match existing nodes.

If your publishers send numeric node ids, set `mqtt_id` and `clear_id` in `navigation_nodes.yaml`. The default trash-bin config already maps bins `1-7` this way.

## Mock publisher

The mock-data PC should run the standalone file:

`/Users/nisa/Desktop/moborobo/mock_bin_status_publisher.py`

Run:

```bash
python3 /Users/nisa/Desktop/moborobo/mock_bin_status_publisher.py \
  --host 192.168.1.100
```

The file is intentionally outside the ROS package. Edit the `MOCK_BINS` list inside it when you want to change nodes `2-7`, then restart the script.

## MQTT notes

Preferred runtime:

- `python3-paho-mqtt`

Fallback runtime:

- `mosquitto_sub`
- `mosquitto_pub`

If `paho-mqtt` is not installed, the executor falls back to `mosquitto_sub` for alarm subscription and `mosquitto_pub` for clear publishing when they exist on the system.
