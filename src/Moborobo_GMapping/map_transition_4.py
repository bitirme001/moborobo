#!/usr/bin/env python3
# https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

import rospy
import tf
import datetime
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D, PoseStamped
from slam_toolbox_msgs.srv import DeserializePoseGraph


COMMANDS_FILE = '/home/mali7319/catkin_ws/commands.txt'
NUMBER_OF_COMMANDS = 0

BOLD_RED_LOG = '\033[1;31m'
BOLD_GREEN_LOG = '\033[1;32m'
BOLD_CYAN_LOG = '\033[1;36m'
BOLD_PURPLE_LOG = '\033[1;35m'
RESET_COLOR = '\033[0;0m'



def movebase_client(client, waypoint_list, checker):

   # Creates a new goal with the MoveBaseGoal constructor
    destination_msg = load_map_and_give_dest(waypoint_list, checker)
    goal = MoveBaseGoal(destination_msg)

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()  



def load_map_and_give_dest(waypoint_list, checker):

    filename = str(waypoint_list[checker].split("\n")[0].split(" ")[4])[1:-1]
    match_type = int(waypoint_list[checker].split("\n")[1].split(" ")[1])
    x = float(waypoint_list[checker].split("\n")[3].split(" ")[3])
    y = float(waypoint_list[checker].split("\n")[4].split(" ")[3])
    theta = float(waypoint_list[checker].split("\n")[5].split(" ")[3][:-1])

    frame_id = str(waypoint_list[checker+1].split(" ")[22].split("\n")[0])[1:-1]

    pose_x = float(waypoint_list[checker+1].split(" ")[29])
    pose_y = float(waypoint_list[checker+1].split(" ")[34])
    pose_z = float(waypoint_list[checker+1].split(" ")[39])

    angle_x = float(waypoint_list[checker+1].split(" ")[46])
    angle_y = float(waypoint_list[checker+1].split(" ")[51])
    angle_z = float(waypoint_list[checker+1].split(" ")[56])
    angle_w = float(waypoint_list[checker+1].split(" ")[61][:-1])

    call_deserialize_map_service(filename, match_type, x, y, theta)
    
    destination_message = PoseStamped(Header(frame_id=frame_id), 
                                      Pose(
                                          Point(pose_x, pose_y, pose_z), 
                                          Quaternion(angle_x, angle_y, angle_z, angle_w)))
    
    hour = datetime.datetime.now().hour
    minute = datetime.datetime.now().minute
    second = datetime.datetime.now().second
    rospy.loginfo(BOLD_PURPLE_LOG + f"Destination point is received!" + BOLD_RED_LOG + f"  At time: {hour}:{minute}:{second}" + RESET_COLOR)
    
    return destination_message



def call_deserialize_map_service(filename, match_type, x, y, theta):

    try:
        deserialize_map = rospy.ServiceProxy("/slam_toolbox/deserialize_map", DeserializePoseGraph)
        deserialize_map(filename, match_type, Pose2D(x, y, theta))
        hour = datetime.datetime.now().hour
        minute = datetime.datetime.now().minute
        second = datetime.datetime.now().second
        rospy.loginfo(BOLD_CYAN_LOG + "New map is loaded successfully!" + BOLD_RED_LOG + f"  At time: {hour}:{minute}:{second}" + RESET_COLOR)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    rospy.sleep(3)


if __name__ == '__main__':

    rospy.init_node("moborobot_tf_listener")
    destination = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

    checker = 0
    waypoint_list = []
    with open(COMMANDS_FILE, "r") as waypoints_file:
        waypoint = waypoints_file.read().split("\n\n")
        waypoint_list.extend(waypoint)

    NUMBER_OF_COMMANDS = len(waypoint_list)

    rospy.wait_for_service("/slam_toolbox/deserialize_map")
        
       # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    listener = tf.TransformListener()
    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        try:
            (location, angle) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        while checker < NUMBER_OF_COMMANDS:
            print("before")
            result = movebase_client(client, waypoint_list, checker)
            print("after")
            if result:
                hour = datetime.datetime.now().hour
                minute = datetime.datetime.now().minute
                second = datetime.datetime.now().second
                rospy.loginfo(BOLD_GREEN_LOG + "Arrived!" + BOLD_RED_LOG + f"  At time: {hour}:{minute}:{second}" + RESET_COLOR)
                client.stop_tracking_goal()
                client.cancel_goal()
                checker += 2

        rospy.loginfo(BOLD_RED_LOG + "My job is done :)" + RESET_COLOR)
        break

