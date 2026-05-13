#!/usr/bin/env python3

import math
import os

import rospy
import tf
import yaml
from tf.transformations import euler_from_quaternion


DEFAULT_OUTPUT_FILE = os.path.realpath(
    os.path.join(os.path.dirname(__file__), "..", "config", "recorded_nodes.yaml")
)


class NodePathRecorder:
    def __init__(self):
        self.output_file = rospy.get_param("~output_file", DEFAULT_OUTPUT_FILE)
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.distance_threshold = float(rospy.get_param("~distance_threshold", 1.0))
        self.record_rate = float(rospy.get_param("~record_rate", 2.0))
        self.id_prefix = rospy.get_param("~id_prefix", "route_node")

        self.tf_listener = tf.TransformListener()
        self.recorded_nodes = []

        rospy.on_shutdown(self.write_nodes)

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

    def should_record_pose(self, pose):
        if not self.recorded_nodes:
            return True

        last_node = self.recorded_nodes[-1]
        distance = math.hypot(pose["x"] - last_node["x"], pose["y"] - last_node["y"])
        return distance >= self.distance_threshold

    def append_node(self, pose):
        node_index = len(self.recorded_nodes) + 1
        node_id = "%s_%03d" % (self.id_prefix, node_index)
        node = {
            "id": node_id,
            "name": "Recorded Node %03d" % node_index,
            "x": round(pose["x"], 3),
            "y": round(pose["y"], 3),
            "yaw": round(pose["yaw"], 3),
            "active": True,
            "neighbors": [],
        }

        if self.recorded_nodes:
            previous_node = self.recorded_nodes[-1]
            previous_neighbors = previous_node.setdefault("neighbors", [])
            if node_id not in previous_neighbors:
                previous_neighbors.append(node_id)
            node["neighbors"].append(previous_node["id"])

        self.recorded_nodes.append(node)
        rospy.loginfo(
            "Recorded node %s at x=%.3f y=%.3f yaw=%.3f",
            node_id,
            node["x"],
            node["y"],
            node["yaw"],
        )

    def write_nodes(self):
        output_directory = os.path.dirname(self.output_file)
        if output_directory:
            os.makedirs(output_directory, exist_ok=True)

        with open(self.output_file, "w") as file_handle:
            yaml.safe_dump(
                {"nodes": self.recorded_nodes},
                file_handle,
                sort_keys=False,
                allow_unicode=False,
            )

        rospy.loginfo(
            "Saved %d recorded nodes to %s",
            len(self.recorded_nodes),
            self.output_file,
        )

    def spin(self):
        rate = rospy.Rate(self.record_rate)
        while not rospy.is_shutdown():
            pose = self.lookup_current_pose()
            if pose is not None and self.should_record_pose(pose):
                self.append_node(pose)
            rate.sleep()


def main():
    rospy.init_node("node_path_recorder")
    NodePathRecorder().spin()


if __name__ == "__main__":
    main()
