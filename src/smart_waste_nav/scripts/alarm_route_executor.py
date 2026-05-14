#!/usr/bin/env python3

import heapq
import json
import math
import os
import re
import shutil
import subprocess
import threading
from collections import OrderedDict

import actionlib
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from static_map_planner import StaticOccupancyMap

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


DEFAULT_NODES_FILE = os.path.realpath(
    os.path.join(os.path.dirname(__file__), "..", "config", "navigation_nodes.yaml")
)
DEFAULT_MAP_YAML_FILE = os.path.realpath(
    os.path.join(os.path.dirname(__file__), "..", "maps", "sunum_map.yaml")
)


def normalize_name(value):
    text = str(value or "").strip().lower()
    return "".join(character for character in text if character.isalnum())


def coerce_float(value):
    if value in (None, ""):
        return None

    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def coerce_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y", "on"}:
            return True
        if normalized in {"false", "0", "no", "n", "off"}:
            return False
    return default


def load_nodes(path):
    with open(path, "r") as file_handle:
        data = yaml.safe_load(file_handle) or {}

    raw_nodes = data.get("nodes") or data.get("waypoints") or []
    if not raw_nodes:
        raise ValueError("Navigation config does not contain any nodes")

    nodes = OrderedDict()
    for raw_node in raw_nodes:
        node_id = str(raw_node["id"])
        if node_id in nodes:
            raise ValueError("Duplicate node id detected: %s" % node_id)

        nodes[node_id] = {
            "id": node_id,
            "name": raw_node.get("name", node_id),
            "aliases": [str(alias) for alias in raw_node.get("aliases", [])],
            "x": float(raw_node["x"]),
            "y": float(raw_node["y"]),
            "yaw": float(raw_node.get("yaw", 0.0)),
            "lat": coerce_float(raw_node.get("lat")),
            "lng": coerce_float(raw_node.get("lng")),
            "mqtt_id": raw_node.get("mqtt_id"),
            "clear_id": raw_node.get("clear_id"),
            "active": bool(raw_node.get("active", True)),
            "neighbors": [str(neighbor) for neighbor in raw_node.get("neighbors", [])],
        }

    return nodes


def create_goal(node, frame_id="map"):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = node["x"]
    goal.target_pose.pose.position.y = node["y"]
    goal.target_pose.pose.position.z = 0.0

    quaternion = quaternion_from_euler(0.0, 0.0, node["yaw"])
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    return goal


def create_pose_stamped(x, y, yaw, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    quaternion = quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    return pose


def path_length_from_poses(poses):
    if len(poses) < 2:
        return 0.0

    total_distance = 0.0
    previous_pose = poses[0].pose.position
    for pose_stamped in poses[1:]:
        current_pose = pose_stamped.pose.position
        total_distance += math.hypot(
            current_pose.x - previous_pose.x,
            current_pose.y - previous_pose.y,
        )
        previous_pose = current_pose
    return total_distance


def path_points_from_poses(poses):
    return [
        (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
        for pose_stamped in poses
    ]


def haversine_distance_meters(lat_1, lng_1, lat_2, lng_2):
    earth_radius_meters = 6371000.0
    lat_1_rad = math.radians(lat_1)
    lat_2_rad = math.radians(lat_2)
    delta_lat = math.radians(lat_2 - lat_1)
    delta_lng = math.radians(lng_2 - lng_1)

    value = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat_1_rad)
        * math.cos(lat_2_rad)
        * math.sin(delta_lng / 2.0) ** 2
    )
    return 2.0 * earth_radius_meters * math.asin(math.sqrt(value))


class AlarmRouteExecutor:
    def __init__(self):
        self.nodes_file = rospy.get_param("~nodes_file", DEFAULT_NODES_FILE)
        self.alarm_topic = rospy.get_param("~alarm_topic", "/waste_alarm")
        self.enable_ros_alarm_topic = coerce_bool(
            rospy.get_param("~enable_ros_alarm_topic", True), default=True
        )
        self.enable_mqtt = coerce_bool(rospy.get_param("~enable_mqtt", False))
        self.mqtt_host = rospy.get_param("~mqtt_host", "localhost")
        self.mqtt_port = int(rospy.get_param("~mqtt_port", 1883))
        self.mqtt_topic = rospy.get_param("~mqtt_topic", "bin/status")
        self.mqtt_clear_topic = rospy.get_param("~mqtt_clear_topic", "bin/cleared")
        self.mqtt_client_id = rospy.get_param(
            "~mqtt_client_id", "smart_waste_nav_executor"
        )
        self.mqtt_username = rospy.get_param("~mqtt_username", "")
        self.mqtt_password = rospy.get_param("~mqtt_password", "")
        self.mqtt_qos = int(rospy.get_param("~mqtt_qos", 0))
        self.mqtt_keepalive = int(rospy.get_param("~mqtt_keepalive", 60))
        self.mqtt_clear_qos = int(rospy.get_param("~mqtt_clear_qos", 0))
        self.mqtt_clear_retain = coerce_bool(
            rospy.get_param("~mqtt_clear_retain", False)
        )
        self.publish_cleared_on_reach = coerce_bool(
            rospy.get_param("~publish_cleared_on_reach", True), default=True
        )
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.75))
        self.wait_after_goal = float(rospy.get_param("~wait_after_goal", 1.0))
        self.max_exact_route_nodes = int(rospy.get_param("~max_exact_route_nodes", 9))
        self.alarm_filter_enabled = coerce_bool(
            rospy.get_param("~alarm_filter_enabled", False)
        )
        self.fill_percent_threshold = float(
            rospy.get_param("~fill_percent_threshold", 80.0)
        )
        self.use_move_base_plan_costs = coerce_bool(
            rospy.get_param("~use_move_base_plan_costs", True), default=True
        )
        self.make_plan_service_name = rospy.get_param(
            "~make_plan_service_name", "/move_base/make_plan"
        )
        self.make_plan_tolerance = float(
            rospy.get_param("~make_plan_tolerance", 0.10)
        )
        self.use_static_map_costs = coerce_bool(
            rospy.get_param("~use_static_map_costs", True), default=True
        )
        self.map_yaml_path = rospy.get_param("~map_yaml_path", DEFAULT_MAP_YAML_FILE)
        self.static_map_clearance_m = float(
            rospy.get_param("~static_map_clearance_m", 0.20)
        )
        self.static_map_allow_unknown = coerce_bool(
            rospy.get_param("~static_map_allow_unknown", False)
        )

        self.nodes = load_nodes(self.nodes_file)
        self.active_node_ids = [
            node_id for node_id, node in self.nodes.items() if node.get("active", True)
        ]
        if not self.active_node_ids:
            raise ValueError("No active nodes found in %s" % self.nodes_file)

        self.static_map = None
        if self.use_static_map_costs:
            try:
                self.static_map = StaticOccupancyMap.from_map_yaml(
                    self.map_yaml_path,
                    clearance_m=self.static_map_clearance_m,
                    allow_unknown=self.static_map_allow_unknown,
                )
                rospy.loginfo(
                    "Loaded static map costs from %s for obstacle-aware route planning.",
                    self.map_yaml_path,
                )
            except Exception as error:
                rospy.logwarn(
                    "Could not load static map costs from %s: %s. Falling back to Euclidean edge costs.",
                    self.map_yaml_path,
                    error,
                )

        self.name_index = self.build_name_index()
        self.edge_path_cache = {}
        self.edge_cost_cache = {}
        self.shortest_path_cache = {}
        self.distance_matrix = {}
        self.graph = self.build_graph()
        self.pending_alarm_nodes = OrderedDict()
        self.pending_alarm_lock = threading.Lock()
        self.replan_requested = False

        self.demo_alarm_node_ids = rospy.get_param("~demo_alarm_node_ids", [])
        self.demo_alarm_json = rospy.get_param("~demo_alarm_json", "")
        self.mqtt_client = None
        self.mosquitto_process = None
        self.mosquitto_thread = None
        self.make_plan_service = None

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.tf_listener = tf.TransformListener()
        self.route_publisher = rospy.Publisher(
            "~planned_route", Path, queue_size=1, latch=True
        )
        self.alarm_subscriber = None
        if self.enable_ros_alarm_topic:
            self.alarm_subscriber = rospy.Subscriber(
                self.alarm_topic, String, self.alarm_callback, queue_size=20
            )

        rospy.on_shutdown(self.shutdown)
        self.setup_alarm_inputs()

    def parse_bin_numeric_id(self, value):
        if value is None:
            return None

        if isinstance(value, bool):
            return None

        if isinstance(value, int):
            return value

        if isinstance(value, float):
            return int(value)

        text = str(value).strip()
        if not text:
            return None

        if text.isdigit():
            return int(text)

        match = re.search(r"(\d+)", text)
        if match:
            return int(match.group(1))

        return None

    def build_name_index(self):
        index = {}
        for node_id, node in self.nodes.items():
            candidates = [
                node_id,
                node["name"],
                node.get("mqtt_id"),
                node.get("clear_id"),
            ] + node.get("aliases", [])
            for candidate in candidates:
                normalized = normalize_name(candidate)
                if normalized and normalized not in index:
                    index[normalized] = node_id
        return index

    def refresh_graph(self):
        self.edge_path_cache = {}
        self.edge_cost_cache = {}
        self.shortest_path_cache = {}
        self.distance_matrix = {}
        self.graph = self.build_graph()
        self.build_distance_matrix()

    def build_graph(self):
        graph = {node_id: {} for node_id in self.active_node_ids}
        has_explicit_edges = False

        for node_id in self.active_node_ids:
            node = self.nodes[node_id]
            for neighbor_id in node.get("neighbors", []):
                if neighbor_id not in self.nodes:
                    rospy.logwarn(
                        "Node %s references unknown neighbor %s", node_id, neighbor_id
                    )
                    continue
                if not self.nodes[neighbor_id].get("active", True):
                    continue

                has_explicit_edges = True
                cost = self.compute_edge_cost(node_id, neighbor_id)
                graph[node_id][neighbor_id] = cost
                graph[neighbor_id][node_id] = cost

        if has_explicit_edges:
            return graph

        rospy.logwarn(
            "No explicit node graph found in %s, using all-to-all Euclidean fallback.",
            self.nodes_file,
        )
        for source_index, source_id in enumerate(self.active_node_ids):
            for target_id in self.active_node_ids[source_index + 1 :]:
                cost = self.compute_edge_cost(source_id, target_id)
                graph[source_id][target_id] = cost
                graph[target_id][source_id] = cost

        return graph

    def euclidean_node_distance(self, source_id, target_id):
        source = self.nodes[source_id]
        target = self.nodes[target_id]
        return math.hypot(source["x"] - target["x"], source["y"] - target["y"])

    def compute_edge_cost(self, source_id, target_id):
        edge_key = tuple(sorted((source_id, target_id)))
        if edge_key in self.edge_cost_cache:
            return self.edge_cost_cache[edge_key]

        source = self.nodes[source_id]
        target = self.nodes[target_id]
        direct_path = [
            (source["x"], source["y"]),
            (target["x"], target["y"]),
        ]
        euclidean_cost = self.euclidean_node_distance(source_id, target_id)
        self.edge_path_cache[(source_id, target_id)] = direct_path
        self.edge_path_cache[(target_id, source_id)] = list(reversed(direct_path))

        if self.make_plan_service is not None:
            edge_cost, world_path = self.plan_cost_between_nodes(source_id, target_id)
            if not math.isinf(edge_cost) and world_path:
                self.edge_path_cache[(source_id, target_id)] = world_path
                self.edge_path_cache[(target_id, source_id)] = list(reversed(world_path))
                self.edge_cost_cache[edge_key] = edge_cost
                return edge_cost

        if self.static_map is None:
            self.edge_cost_cache[edge_key] = euclidean_cost
            return euclidean_cost

        edge_cost, world_path = self.static_map.plan_path(
            source["x"],
            source["y"],
            target["x"],
            target["y"],
        )
        if math.isinf(edge_cost) or not world_path:
            rospy.logwarn(
                "Static map path not found between %s and %s. Using Euclidean fallback.",
                source_id,
                target_id,
            )
            self.edge_cost_cache[edge_key] = euclidean_cost
            return euclidean_cost

        self.edge_path_cache[(source_id, target_id)] = world_path
        self.edge_path_cache[(target_id, source_id)] = list(reversed(world_path))
        self.edge_cost_cache[edge_key] = edge_cost
        return edge_cost

    def connect_make_plan_service(self):
        if not self.use_move_base_plan_costs:
            return

        try:
            rospy.wait_for_service(self.make_plan_service_name, timeout=5.0)
            self.make_plan_service = rospy.ServiceProxy(
                self.make_plan_service_name, GetPlan
            )
            rospy.loginfo(
                "Connected to planner service %s for route cost matrix.",
                self.make_plan_service_name,
            )
        except (rospy.ROSException, rospy.ROSInterruptException) as error:
            rospy.logwarn(
                "Planner service %s not available: %s. Falling back to static map or Euclidean costs.",
                self.make_plan_service_name,
                error,
            )
            self.make_plan_service = None

    def plan_cost_from_pose(self, pose, node_id):
        if self.make_plan_service is None:
            return float("inf"), []

        node = self.nodes[node_id]
        start_pose = create_pose_stamped(
            pose["x"], pose["y"], pose.get("yaw", 0.0), self.map_frame
        )
        goal_pose = create_pose_stamped(node["x"], node["y"], node["yaw"], self.map_frame)

        try:
            response = self.make_plan_service(
                start=start_pose,
                goal=goal_pose,
                tolerance=self.make_plan_tolerance,
            )
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(
                5.0,
                "make_plan failed from current pose to %s: %s",
                node_id,
                error,
            )
            return float("inf"), []

        if not response.plan.poses:
            return float("inf"), []

        return path_length_from_poses(response.plan.poses), path_points_from_poses(
            response.plan.poses
        )

    def plan_cost_between_nodes(self, source_id, target_id):
        if self.make_plan_service is None:
            return float("inf"), []

        source = self.nodes[source_id]
        target = self.nodes[target_id]
        start_pose = create_pose_stamped(
            source["x"], source["y"], source["yaw"], self.map_frame
        )
        goal_pose = create_pose_stamped(
            target["x"], target["y"], target["yaw"], self.map_frame
        )

        try:
            response = self.make_plan_service(
                start=start_pose,
                goal=goal_pose,
                tolerance=self.make_plan_tolerance,
            )
        except rospy.ServiceException as error:
            rospy.logwarn(
                "make_plan failed between %s and %s: %s",
                source_id,
                target_id,
                error,
            )
            return float("inf"), []

        if not response.plan.poses:
            return float("inf"), []

        return path_length_from_poses(response.plan.poses), path_points_from_poses(
            response.plan.poses
        )

    def parse_alarm_payloads(self, payload):
        data = payload
        if isinstance(payload, str):
            data = json.loads(payload)

        if isinstance(data, list):
            return data
        if isinstance(data, dict) and isinstance(data.get("alarms"), list):
            return data["alarms"]
        if isinstance(data, dict):
            return [data]

        raise ValueError("Alarm payload must be an object, array or {alarms: [...]}")

    def alarm_callback(self, message):
        self.handle_alarm_payload(message.data, source_label="ros_topic")

    def handle_alarm_payload(self, payload_text, source_label):
        try:
            alarm_payloads = self.parse_alarm_payloads(payload_text)
        except (ValueError, json.JSONDecodeError) as error:
            rospy.logwarn("Invalid alarm payload received: %s", error)
            return

        self.register_alarm_payloads(alarm_payloads, source_label=source_label)

    def register_alarm_payloads(self, alarm_payloads, source_label):
        queued_count = 0
        with self.pending_alarm_lock:
            for payload in alarm_payloads:
                if not isinstance(payload, dict):
                    rospy.logwarn(
                        "Ignoring alarm entry because it is not a JSON object: %s", payload
                    )
                    continue

                if not self.is_alarm_candidate(payload):
                    continue

                matched_node_id = self.match_alarm_to_node(payload)
                if matched_node_id is None:
                    rospy.logwarn("Alarm could not be mapped to any node: %s", payload)
                    continue

                self.pending_alarm_nodes[matched_node_id] = dict(payload)
                queued_count += 1
                rospy.loginfo(
                    "Queued %s alarm for node %s from %s",
                    payload.get("name", matched_node_id),
                    matched_node_id,
                    source_label,
                )

            if queued_count:
                self.replan_requested = True

    def is_alarm_candidate(self, payload):
        if not self.alarm_filter_enabled:
            return True

        if payload.get("alarm") is not None:
            return coerce_bool(payload.get("alarm"))

        if payload.get("isFull") is not None:
            return coerce_bool(payload.get("isFull"))

        fill_percent = coerce_float(payload.get("fillPercent"))
        return fill_percent is not None and fill_percent >= self.fill_percent_threshold

    def match_alarm_to_node(self, payload):
        candidate_fields = [
            payload.get("nodeId"),
            payload.get("node_id"),
            payload.get("pointId"),
            payload.get("point_id"),
            payload.get("binId"),
            payload.get("bin_id"),
            payload.get("id"),
            payload.get("name"),
            payload.get("point"),
        ]

        for candidate in candidate_fields:
            name_key = normalize_name(candidate)
            if name_key in self.name_index:
                return self.name_index[name_key]

        latitude = coerce_float(payload.get("lat"))
        longitude = coerce_float(payload.get("lng"))
        if latitude is None or longitude is None:
            return None

        best_node_id = None
        best_distance = None
        for node_id in self.active_node_ids:
            node = self.nodes[node_id]
            if node["lat"] is None or node["lng"] is None:
                continue

            distance = haversine_distance_meters(
                latitude,
                longitude,
                node["lat"],
                node["lng"],
            )
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_node_id = node_id

        return best_node_id

    def queue_demo_alarms(self):
        if self.demo_alarm_node_ids:
            demo_payloads = []
            for node_id in self.demo_alarm_node_ids:
                if node_id not in self.nodes:
                    rospy.logwarn("Demo alarm node does not exist: %s", node_id)
                    continue
                demo_payloads.append(
                    {
                        "name": self.nodes[node_id]["name"],
                        "isFull": True,
                        "fillPercent": 100,
                    }
                )

            if demo_payloads:
                self.register_alarm_payloads(
                    demo_payloads, source_label="demo_node_ids"
                )

        if self.demo_alarm_json:
            try:
                demo_payloads = self.parse_alarm_payloads(self.demo_alarm_json)
            except (ValueError, json.JSONDecodeError) as error:
                rospy.logwarn("Invalid demo alarm JSON: %s", error)
                return

            self.register_alarm_payloads(demo_payloads, source_label="demo_alarm_json")

    def setup_alarm_inputs(self):
        if self.enable_ros_alarm_topic:
            rospy.loginfo("ROS alarm input enabled on %s", self.alarm_topic)

        if not self.enable_mqtt:
            return

        if mqtt is not None:
            self.setup_paho_mqtt()
            return

        if shutil.which("mosquitto_sub"):
            self.setup_mosquitto_subprocess()
            return

        rospy.logerr(
            "MQTT alarm input requested but neither paho-mqtt nor mosquitto_sub is available."
        )

    def setup_paho_mqtt(self):
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        if self.mqtt_username:
            self.mqtt_client.username_pw_set(
                self.mqtt_username,
                self.mqtt_password,
            )

        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, self.mqtt_keepalive)
        self.mqtt_client.loop_start()
        rospy.loginfo(
            "MQTT alarm input enabled via paho-mqtt on %s:%d topic %s",
            self.mqtt_host,
            self.mqtt_port,
            self.mqtt_topic,
        )

    def setup_mosquitto_subprocess(self):
        command = [
            "mosquitto_sub",
            "-h",
            self.mqtt_host,
            "-p",
            str(self.mqtt_port),
            "-t",
            self.mqtt_topic,
            "-q",
            str(self.mqtt_qos),
        ]
        if self.mqtt_username:
            command.extend(["-u", self.mqtt_username])
        if self.mqtt_password:
            command.extend(["-P", self.mqtt_password])

        self.mosquitto_process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self.mosquitto_thread = threading.Thread(
            target=self.read_mosquitto_output,
            name="mosquitto_sub_reader",
            daemon=True,
        )
        self.mosquitto_thread.start()
        rospy.loginfo(
            "MQTT alarm input enabled via mosquitto_sub on %s:%d topic %s",
            self.mqtt_host,
            self.mqtt_port,
            self.mqtt_topic,
        )

    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(self.mqtt_topic, qos=self.mqtt_qos)
            rospy.loginfo("Connected to MQTT broker and subscribed to %s", self.mqtt_topic)
            return

        rospy.logwarn("MQTT connection failed with rc=%s", rc)

    def on_mqtt_message(self, client, userdata, message):
        payload_text = message.payload.decode("utf-8", errors="replace")
        self.handle_alarm_payload(payload_text, source_label="mqtt")

    def on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        if rc != 0 and not rospy.is_shutdown():
            rospy.logwarn("MQTT connection closed unexpectedly with rc=%s", rc)

    def read_mosquitto_output(self):
        if self.mosquitto_process is None or self.mosquitto_process.stdout is None:
            return

        for line in self.mosquitto_process.stdout:
            payload_text = line.strip()
            if not payload_text:
                continue
            self.handle_alarm_payload(payload_text, source_label="mqtt")

    def shutdown(self):
        if self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_client = None

        if self.mosquitto_process is not None:
            self.mosquitto_process.terminate()
            self.mosquitto_process = None

    def lookup_current_pose(self):
        try:
            self.tf_listener.waitForTransform(
                self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            translation, rotation = self.tf_listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0)
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        yaw = euler_from_quaternion(rotation)[2]
        return {"x": translation[0], "y": translation[1], "yaw": yaw}

    def pose_distance_to_node(self, pose, node_id):
        node = self.nodes[node_id]
        return math.hypot(pose["x"] - node["x"], pose["y"] - node["y"])

    def nearest_node_id(self, pose):
        best_node_id = None
        best_distance = None
        for node_id in self.active_node_ids:
            distance = self.pose_travel_cost_to_node(pose, node_id)
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_node_id = node_id
        return best_node_id

    def pose_travel_cost_to_node(self, pose, node_id):
        if self.make_plan_service is not None:
            travel_cost, _ = self.plan_cost_from_pose(pose, node_id)
            if not math.isinf(travel_cost):
                return travel_cost

        if self.static_map is None:
            return self.pose_distance_to_node(pose, node_id)

        node = self.nodes[node_id]
        travel_cost, _ = self.static_map.plan_path(
            pose["x"],
            pose["y"],
            node["x"],
            node["y"],
        )
        if math.isinf(travel_cost):
            return self.pose_distance_to_node(pose, node_id)
        return travel_cost

    def build_distance_matrix(self):
        for source_id in self.active_node_ids:
            self.distance_matrix[source_id] = {}
            for target_id in self.active_node_ids:
                if source_id == target_id:
                    self.distance_matrix[source_id][target_id] = 0.0
                    continue

                distance, _ = self.shortest_path(source_id, target_id)
                self.distance_matrix[source_id][target_id] = distance

        rospy.loginfo("Built %dx%d node distance matrix.", len(self.active_node_ids), len(self.active_node_ids))

    def consume_reached_alarm_nodes(self, pose):
        with self.pending_alarm_lock:
            reached_node_ids = [
                node_id
                for node_id in list(self.pending_alarm_nodes.keys())
                if self.pose_distance_to_node(pose, node_id) <= self.goal_tolerance
            ]
            for node_id in reached_node_ids:
                self.pending_alarm_nodes.pop(node_id, None)
                rospy.loginfo("Alarm node %s marked as already reached.", node_id)

    def get_pending_alarm_node_ids(self):
        with self.pending_alarm_lock:
            return list(self.pending_alarm_nodes.keys())

    def get_pending_alarm_payload(self, node_id):
        with self.pending_alarm_lock:
            payload = self.pending_alarm_nodes.get(node_id)
            return dict(payload) if payload is not None else None

    def has_pending_alarm_nodes(self):
        with self.pending_alarm_lock:
            return bool(self.pending_alarm_nodes)

    def clear_alarm_node(self, node_id):
        with self.pending_alarm_lock:
            self.pending_alarm_nodes.pop(node_id, None)

    def set_replan_requested(self, value):
        with self.pending_alarm_lock:
            self.replan_requested = value

    def is_replan_requested(self):
        with self.pending_alarm_lock:
            return self.replan_requested

    def resolve_clear_bin_id(self, node_id):
        payload = self.get_pending_alarm_payload(node_id)
        candidate_values = []

        if payload is not None:
            candidate_values.extend(
                [
                    payload.get("binId"),
                    payload.get("bin_id"),
                    payload.get("nodeId"),
                    payload.get("node_id"),
                    payload.get("pointId"),
                    payload.get("point_id"),
                    payload.get("id"),
                    payload.get("name"),
                ]
            )

        node = self.nodes.get(node_id, {})
        candidate_values.extend(
            [
                node.get("clear_id"),
                node.get("mqtt_id"),
                node.get("id"),
                node.get("name"),
            ]
        )

        for candidate_value in candidate_values:
            numeric_id = self.parse_bin_numeric_id(candidate_value)
            if numeric_id is not None:
                return numeric_id

        return None

    def publish_clear_message(self, node_id):
        if not self.publish_cleared_on_reach:
            return True

        clear_bin_id = self.resolve_clear_bin_id(node_id)
        if clear_bin_id is None:
            rospy.logwarn("Could not resolve clear bin id for node %s", node_id)
            return False

        payload_text = json.dumps({"id": clear_bin_id, "emptied": True})

        if self.mqtt_client is not None:
            result = self.mqtt_client.publish(
                self.mqtt_clear_topic,
                payload_text,
                qos=self.mqtt_clear_qos,
                retain=self.mqtt_clear_retain,
            )
            result_code = getattr(result, "rc", 0)
            if result_code == 0:
                rospy.loginfo(
                    "Published clear message for node %s to %s: %s",
                    node_id,
                    self.mqtt_clear_topic,
                    payload_text,
                )
                return True

            rospy.logwarn(
                "MQTT clear publish failed for node %s with rc=%s",
                node_id,
                result_code,
            )
            return False

        if shutil.which("mosquitto_pub"):
            command = [
                "mosquitto_pub",
                "-h",
                self.mqtt_host,
                "-p",
                str(self.mqtt_port),
                "-t",
                self.mqtt_clear_topic,
                "-q",
                str(self.mqtt_clear_qos),
                "-m",
                payload_text,
            ]
            if self.mqtt_clear_retain:
                command.append("-r")
            if self.mqtt_username:
                command.extend(["-u", self.mqtt_username])
            if self.mqtt_password:
                command.extend(["-P", self.mqtt_password])

            try:
                subprocess.run(command, check=True)
                rospy.loginfo(
                    "Published clear message for node %s with mosquitto_pub: %s",
                    node_id,
                    payload_text,
                )
                return True
            except (subprocess.CalledProcessError, OSError) as error:
                rospy.logwarn(
                    "mosquitto_pub clear publish failed for node %s: %s",
                    node_id,
                    error,
                )
                return False

        rospy.logwarn(
            "No MQTT publisher available to send clear message for node %s.", node_id
        )
        return False

    def shortest_path(self, start_id, target_id):
        cache_key = (start_id, target_id)
        if cache_key in self.shortest_path_cache:
            return self.shortest_path_cache[cache_key]

        queue = [(0.0, start_id, [start_id])]
        best_costs = {start_id: 0.0}

        while queue:
            current_cost, current_id, current_path = heapq.heappop(queue)
            if current_id == target_id:
                result = (current_cost, current_path)
                self.shortest_path_cache[(start_id, target_id)] = result
                self.shortest_path_cache[(target_id, start_id)] = (
                    current_cost,
                    list(reversed(current_path)),
                )
                return result

            if current_cost > best_costs.get(current_id, float("inf")):
                continue

            for neighbor_id, edge_cost in self.graph[current_id].items():
                new_cost = current_cost + edge_cost
                if new_cost >= best_costs.get(neighbor_id, float("inf")):
                    continue

                best_costs[neighbor_id] = new_cost
                heapq.heappush(
                    queue, (new_cost, neighbor_id, current_path + [neighbor_id])
                )

        result = (float("inf"), [])
        self.shortest_path_cache[(start_id, target_id)] = result
        return result

    def solve_target_order(self, start_node_id, target_node_ids):
        target_node_ids = list(OrderedDict.fromkeys(target_node_ids))
        if len(target_node_ids) <= 1:
            return target_node_ids

        if len(target_node_ids) <= self.max_exact_route_nodes:
            exact_order = self.solve_exact_route(start_node_id, target_node_ids)
            if exact_order:
                return exact_order

        return self.solve_greedy_route(start_node_id, target_node_ids)

    def solve_exact_route(self, start_node_id, target_node_ids):
        target_count = len(target_node_ids)
        pair_distances = {}

        for node_id in target_node_ids:
            distance = self.distance_matrix[start_node_id][node_id]
            if math.isinf(distance):
                return []
            pair_distances[(start_node_id, node_id)] = distance

        for source_id in target_node_ids:
            for target_id in target_node_ids:
                if source_id == target_id:
                    continue
                distance = self.distance_matrix[source_id][target_id]
                if math.isinf(distance):
                    return []
                pair_distances[(source_id, target_id)] = distance

        dp = {}
        parents = {}
        for index, node_id in enumerate(target_node_ids):
            mask = 1 << index
            dp[(mask, index)] = pair_distances[(start_node_id, node_id)]
            parents[(mask, index)] = None

        full_mask = (1 << target_count) - 1
        for mask in range(1, full_mask + 1):
            for last_index in range(target_count):
                state_key = (mask, last_index)
                current_cost = dp.get(state_key)
                if current_cost is None:
                    continue
                if not mask & (1 << last_index):
                    continue

                last_node_id = target_node_ids[last_index]
                for next_index, next_node_id in enumerate(target_node_ids):
                    next_bit = 1 << next_index
                    if mask & next_bit:
                        continue

                    next_mask = mask | next_bit
                    next_cost = current_cost + pair_distances[(last_node_id, next_node_id)]
                    next_key = (next_mask, next_index)
                    if next_cost < dp.get(next_key, float("inf")):
                        dp[next_key] = next_cost
                        parents[next_key] = last_index

        best_last_index = None
        best_cost = float("inf")
        for candidate_index in range(target_count):
            candidate_cost = dp.get((full_mask, candidate_index), float("inf"))
            if candidate_cost < best_cost:
                best_cost = candidate_cost
                best_last_index = candidate_index

        if best_last_index is None:
            return []

        order_indices = []
        current_index = best_last_index
        current_mask = full_mask
        while current_index is not None:
            order_indices.append(current_index)
            parent_index = parents[(current_mask, current_index)]
            current_mask ^= 1 << current_index
            current_index = parent_index

        order_indices.reverse()
        return [target_node_ids[index] for index in order_indices]

    def solve_greedy_route(self, start_node_id, target_node_ids):
        remaining_node_ids = list(target_node_ids)
        current_node_id = start_node_id
        ordered_node_ids = []

        while remaining_node_ids:
            best_node_id = None
            best_distance = float("inf")
            for candidate_node_id in remaining_node_ids:
                distance = self.distance_matrix[current_node_id][candidate_node_id]
                if distance < best_distance:
                    best_distance = distance
                    best_node_id = candidate_node_id

            if best_node_id is None or math.isinf(best_distance):
                return ordered_node_ids

            ordered_node_ids.append(best_node_id)
            remaining_node_ids.remove(best_node_id)
            current_node_id = best_node_id

        return ordered_node_ids

    def expand_route(self, start_node_id, ordered_target_ids):
        expanded_node_ids = []
        current_node_id = start_node_id

        for target_node_id in ordered_target_ids:
            _, segment = self.shortest_path(current_node_id, target_node_id)
            if not segment:
                return []

            if expanded_node_ids:
                segment = segment[1:]
            expanded_node_ids.extend(segment)
            current_node_id = target_node_id

        return expanded_node_ids

    def publish_route(self, route_node_ids):
        path_message = Path()
        path_message.header.frame_id = self.map_frame
        path_message.header.stamp = rospy.Time.now()

        route_points = []
        for index, node_id in enumerate(route_node_ids):
            if index == 0:
                node = self.nodes[node_id]
                route_points.append((node["x"], node["y"]))
                continue

            previous_node_id = route_node_ids[index - 1]
            edge_path = self.edge_path_cache.get((previous_node_id, node_id))
            if edge_path:
                for edge_index, point in enumerate(edge_path):
                    if route_points and edge_index == 0:
                        continue
                    route_points.append(point)
                continue

            node = self.nodes[node_id]
            route_points.append((node["x"], node["y"]))

        for x, y in route_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_message.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_message.poses.append(pose_stamped)

        self.route_publisher.publish(path_message)

    def plan_route(self):
        current_pose = self.lookup_current_pose()
        if current_pose is None:
            rospy.logwarn_throttle(5.0, "Current pose in map frame is not available yet.")
            return None

        self.consume_reached_alarm_nodes(current_pose)
        pending_alarm_node_ids = self.get_pending_alarm_node_ids()
        if not pending_alarm_node_ids:
            return None

        start_node_id = self.nearest_node_id(current_pose)
        ordered_target_ids = self.solve_target_order(
            start_node_id, pending_alarm_node_ids
        )
        if not ordered_target_ids:
            rospy.logwarn("Could not find a valid route for pending alarms.")
            return None

        expanded_node_ids = self.expand_route(start_node_id, ordered_target_ids)
        if not expanded_node_ids:
            rospy.logwarn("Could not expand the route between planned alarm nodes.")
            return None

        self.publish_route(expanded_node_ids)
        return {
            "start_node_id": start_node_id,
            "ordered_target_ids": ordered_target_ids,
            "expanded_node_ids": expanded_node_ids,
        }

    def send_goal_to_node(self, node_id):
        node = self.nodes[node_id]
        rospy.loginfo("Sending move_base goal to node %s (%s)", node_id, node["name"])
        self.client.send_goal(create_goal(node, frame_id=self.map_frame))

        while not rospy.is_shutdown():
            if self.client.wait_for_result(rospy.Duration(0.5)):
                break

            if self.is_replan_requested():
                rospy.loginfo(
                    "Cancelling goal for node %s because a new alarm requires replanning.",
                    node_id,
                )
                self.client.cancel_goal()
                self.client.wait_for_result(rospy.Duration(2.0))
                return None

        result_state = self.client.get_state()
        if result_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached node %s", node_id)
            return True

        rospy.logwarn("move_base failed for node %s with state %s", node_id, result_state)
        return False

    def spin(self):
        rospy.loginfo("Loading nodes from %s", self.nodes_file)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")
        self.connect_make_plan_service()
        self.refresh_graph()

        self.queue_demo_alarms()
        idle_rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if not self.has_pending_alarm_nodes():
                self.publish_route([])
                idle_rate.sleep()
                continue

            plan = self.plan_route()
            if plan is None:
                idle_rate.sleep()
                continue

            self.set_replan_requested(False)
            rospy.loginfo(
                "Alarm visit order: %s", " -> ".join(plan["ordered_target_ids"])
            )
            rospy.loginfo(
                "Expanded node route: %s", " -> ".join(plan["expanded_node_ids"])
            )

            for node_id in plan["expanded_node_ids"]:
                if rospy.is_shutdown():
                    break

                if self.is_replan_requested():
                    rospy.loginfo("New alarm arrived, replanning route.")
                    break

                current_pose = self.lookup_current_pose()
                if (
                    current_pose
                    and self.pose_distance_to_node(current_pose, node_id)
                    <= self.goal_tolerance
                ):
                    rospy.loginfo("Skipping node %s because robot is already close.", node_id)
                    goal_succeeded = True
                else:
                    goal_succeeded = self.send_goal_to_node(node_id)

                if goal_succeeded and node_id in self.get_pending_alarm_node_ids():
                    self.publish_clear_message(node_id)
                    self.clear_alarm_node(node_id)
                    rospy.loginfo("Alarm serviced at node %s", node_id)

                if goal_succeeded is None:
                    break

                if goal_succeeded is False:
                    self.set_replan_requested(True)
                    break

                rospy.sleep(self.wait_after_goal)


def main():
    rospy.init_node("alarm_route_executor")
    executor = AlarmRouteExecutor()
    executor.spin()


if __name__ == "__main__":
    main()
