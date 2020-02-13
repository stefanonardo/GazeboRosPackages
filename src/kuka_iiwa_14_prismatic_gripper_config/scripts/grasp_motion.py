#!/usr/bin/env python

import sys
import copy
import rospy
import json
import yaml
import moveit_commander
import math
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import genpy
from moveit_msgs.msg._RobotTrajectory import RobotTrajectory
from threading import Lock
import std_msgs
import std_srvs
from cle_ros_msgs.srv import SetDuration, SetDurationRequest, SetDurationResponse
from time import sleep

targetPoseLock = Lock()
moveLock = Lock()

targetPose = Pose()
robotPose = Pose(position=Point(x=1.0, y=-0.75, z=0.8))

return_speed = 1.0

speed = 6.5
trajectoryIndex = 13
execTime = genpy.Duration()


def createPoseFromJSONObj(posejsonobj):
    """Create Pose object from JSON"""
    return Pose(position=Point(x=posejsonobj["position"]["x"], y=posejsonobj["position"]["y"], z=posejsonobj["position"]["z"]),
                orientation=Quaternion(x=posejsonobj["orientation"]["x"], y=posejsonobj["orientation"]["y"], z=posejsonobj["orientation"]["z"], w=posejsonobj["orientation"]["w"]))


def execTrajectory(trajIndex, speed=1.0):
    """
    Execute trajectory with the given index. Assumes the arm is at uppose, then moves it to a position above the
    conveyor. There, the gripper is closed, and the arm subsequently moved back up to uppose. Lastly, the gripper is
    opened again, releasing any object
    """
    with targetPoseLock:
        exec_traj = iiwa_group.plan(targetPose)
    exec_traj = adjustSpeed(exec_traj, speed)
    iiwa_group.execute(exec_traj)
    grasp_group.go(close_gripper_target)
    return_traj = adjustSpeed(iiwa_group.plan(upJointState), return_speed)
    grasp_group.execute(return_traj)
    grasp_group.go(open_gripper_target)

    return exec_traj


def adjustSpeed(old_traj, speed):
    """Recalculate a trajectory with new speed"""
    new_traj = RobotTrajectory()
    new_traj.joint_trajectory = copy.deepcopy(old_traj.joint_trajectory)
    n_joints = len(old_traj.joint_trajectory.joint_names)
    n_points = len(old_traj.joint_trajectory.points)

    for i in range(n_points):
        new_traj.joint_trajectory.points[i].time_from_start = old_traj.joint_trajectory.points[i].time_from_start / speed
        new_traj.joint_trajectory.points[i].velocities = [old_traj.joint_trajectory.points[i].velocities[j] * speed for j in range(n_joints)]
        new_traj.joint_trajectory.points[i].accelerations = [old_traj.joint_trajectory.points[i].accelerations[j] * speed for j in range(n_joints)]
        new_traj.joint_trajectory.points[i].positions = [old_traj.joint_trajectory.points[i].positions[j] for j in range(n_joints)]

    return new_traj


def callback(data):
    """
    Execute motion to center of conveyor belt if trigger is received.
    While executing a motion, ignore any additional triggers
    """
    if data.data != 0:
        if moveLock.acquire(False):
            global speed
            try:
                execTrajectory(trajectoryIndex, speed)
                sleep(0.01)
            except:
                moveLock.release()
                raise

            moveLock.release()


def setTargetPose(data):
    """Set the target position. Keep orientation the same (gripper facing down toward conveyor)"""
    global targetPose
    with targetPoseLock:
        # Extract last frame's position
        targetPose.position.x = data.data[-3]
        targetPose.position.y = data.data[-2]
        # NOTE: Keep the end effector z position the same. Just above the conveyor belt
        # targetPose.position.z = data.data[-1] + 0.1

        # Get relative position to robot
        targetPose.position = targetPose.position - robotPose.position


def service_callback(data):
    """Set trajectory execution time"""
    global execTime, speed
    execTime = copy.deepcopy(data.duration.data)
    speed = downTrajectories[trajectoryIndex].joint_trajectory.points[-1].time_from_start / execTime

    return SetDurationResponse(success=True)


def add_conveyor_collision(scene, robot):
    """Create and add a collision box to the planning scene that prevents collision with the conveyor belt"""
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.0
    p.pose.position.y = -0.55
    p.pose.position.z = -0.19

    scene.add_box("conveyor", p, (2.5, 0.8, 0.6))

    sleep(2)


if __name__ == '__main__':
    jsonFileName = sys.argv[1]

    # Start Moveit and initialize this node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('grasp_motion', anonymous=True)

    # Identify robot
    robot = moveit_commander.RobotCommander(ns="/iiwa", robot_description="/iiwa/robot_description")

    # Initiate planning scene
    scene = moveit_commander.PlanningSceneInterface(ns="/iiwa", synchronous=True)
    add_conveyor_collision(scene, robot)

    # Get IIWA arm planning group
    iiwa_group_name = "iiwa_plan_group"
    iiwa_group = moveit_commander.MoveGroupCommander(iiwa_group_name, ns="/iiwa",
                                                     robot_description="/iiwa/robot_description")

    # Get Gripper planning group
    grasp_group_name = "grasp_plan_group"
    grasp_group = moveit_commander.MoveGroupCommander(grasp_group_name, ns="/iiwa",
                                                      robot_description="/iiwa/robot_description")

    # Load saved trajectories
    with open(jsonFileName, "r") as jsonFile:
        trajectories = json.load(jsonFile)

    # Set Positions
    uppose = createPoseFromJSONObj(trajectories["up_pose"])
    upJointState = trajectories["up_joint_state"]
    downPoses = [createPoseFromJSONObj(x) for x in trajectories["down_poses"]]

    # Load trajectories from start to position over conveyor belt
    downTrajectories = []
    for trajObj in trajectories["down_trajectories"]:
        trajectory = RobotTrajectory()
        genpy.message.fill_message_args(trajectory, trajObj)
        downTrajectories.append(trajectory)

    # Load trajectories from positions over the conveyor back to the start pose
    upTrajectories = []
    for trajObj in trajectories["up_trajectories"]:
        trajectory = RobotTrajectory()
        genpy.message.fill_message_args(trajectory, trajObj)
        upTrajectories.append(trajectory)

    # Load Open and closed gripper targets
    close_gripper_target = grasp_group.get_named_target_values("gripper_closed")
    open_gripper_target = grasp_group.get_named_target_values("gripper_open")

    # Setup speed and execution time
    execTime = genpy.Duration(0.5)
    speed = downTrajectories[trajectoryIndex].joint_trajectory.points[-1].time_from_start / execTime

    # Move iiwa arm to start pose before accepting calls
    trajectory = iiwa_group.plan(upJointState)
    iiwa_group.execute(trajectory)

    # Set first target pose to default down position
    targetPose = downPoses[trajectoryIndex]

    # Create a subscriber that executes grasp motions
    adaptive_sub = rospy.Subscriber("/adaptive_trigger", std_msgs.msg.Bool, callback, queue_size=1)
    reactive_sub = rospy.Subscriber("/reactive_trigger", std_msgs.msg.Bool, callback, queue_size=1)

    pose_sub = rospy.Subscriber("/pred_pos", std_msgs.msg.Float32MultiArray, setTargetPose, queue_size=10)

    time_pub = rospy.Publisher("/traj_execution_time", std_msgs.msg.Duration, queue_size=10)

    time_service = rospy.Service("/set_traj_execution_time", SetDuration, service_callback)

    while not rospy.is_shutdown():
        # Time Pub
        duration = std_msgs.msg.Duration(execTime)
        time_pub.publish(duration)

        # Make sure arm isn't stuck
        if moveLock.acquire(False):
            try:
                iiwa_group.go(upJointState)
                grasp_group.go(open_gripper_target)
            except:
                moveLock.release()
                raise

            moveLock.release()

        sleep(1)
