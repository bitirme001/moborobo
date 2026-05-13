#!/usr/bin/env python3

import actionlib
import os

import rospy
import yaml

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


DEFAULT_NODES_FILE = os.path.realpath(
    os.path.join(os.path.dirname(__file__), "..", "config", "navigation_nodes.yaml")
)


def load_waypoints(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    if "nodes" in data:
        return data["nodes"]
    if "waypoints" in data:
        return data["waypoints"]

    raise KeyError("Expected either 'nodes' or 'waypoints' in navigation config")


def create_goal(wp):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = wp["x"]
    goal.target_pose.pose.position.y = wp["y"]
    goal.target_pose.pose.position.z = 0.0

    q = quaternion_from_euler(0, 0, wp["yaw"])

    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    return goal


def main():
    rospy.init_node("waypoint_executor")

    waypoints_file = rospy.get_param(
        "~waypoints_file",
        DEFAULT_NODES_FILE
    )

    waypoints = load_waypoints(waypoints_file)

    active_waypoints = [wp for wp in waypoints if wp.get("active", True)]

    rospy.loginfo("Loaded %d active waypoints", len(active_waypoints))

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base.")

    for wp in active_waypoints:
        rospy.loginfo("Going to waypoint: %s", wp["id"])

        goal = create_goal(wp)
        client.send_goal(goal)

        client.wait_for_result()

        result_state = client.get_state()

        if result_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached waypoint: %s", wp["id"])
        else:
            rospy.logwarn("Failed to reach waypoint: %s, state: %s", wp["id"], result_state)
            rospy.logwarn("Continuing to next waypoint...")

        rospy.sleep(2.0)

    rospy.loginfo("All waypoints completed.")


if __name__ == "__main__":
    main()
