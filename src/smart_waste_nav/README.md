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
- MQTT / Mosquitto topic `waste/alarm`

MQTT is enabled by default in `alarm_route_executor.launch`.

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

## MQTT notes

Preferred runtime:

- `python3-paho-mqtt`

Fallback runtime:

- `mosquitto_sub`

If `paho-mqtt` is not installed, the executor automatically falls back to `mosquitto_sub` when it exists on the system.
