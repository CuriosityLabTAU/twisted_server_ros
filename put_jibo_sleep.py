#!/usr/bin/python
import rospy
from jibo_msgs.msg import JiboAction
from std_msgs.msg import Header  # standard ROS msg header
import json
import time

rospy.init_node('jibo_sleep_node')
robot_commander = rospy.Publisher('/jibo', JiboAction, queue_size=10)

def send_robot_anim_transition_cmd(tran):
    msg = JiboAction()
    # add header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.do_anim_transition = True
    msg.anim_transition = tran
    robot_commander.publish(msg)
    rospy.loginfo(msg)

def send_robot_motion_cmd(command):
    """
    send a Motion Command to Jibo
    """

    msg = JiboAction()
    # add header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.do_motion = True
    msg.do_tts = False
    msg.do_lookat = False

    msg.motion = command

    robot_commander.publish(msg)
    rospy.loginfo(msg)



time.sleep(1)
send_robot_motion_cmd("Poses/Directional/Body_Look_Center_Down_01_01.keys")
send_robot_anim_transition_cmd(JiboAction.ANIMTRANS_KEEP_LASTFRAME)
send_robot_motion_cmd("Eye-Globals/open-to-close_01.keys")