#!/usr/bin/python
import json
import rospy
from std_msgs.msg import String
import sys,os
from time import sleep, time
from std_msgs.msg import Header  # standard ROS msg header
from jibo_msgs.msg import JiboAction
import threading
import collections

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
json_file = "participant_info.json"

rospy.init_node('GameCommand', anonymous=False)
publisher = rospy.Publisher('/jibo', JiboAction, queue_size=1)

def send_robot_tts_cmd(text):
        """
        send a Motion Command to Jibo
        """

        msg = JiboAction()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()


        msg.do_motion = False
        msg.do_tts = True
        msg.do_lookat = False

        msg.tts_text = text

        publisher.publish(msg)
        #rospy.loginfo(msg)
        #print("\nSent TTS message: ", text)

def main(info):

    #print info
    sleep(0.5)

    if len(info) > 0:
        send_robot_tts_cmd(info[0])
        sleep(2)
        exit()


    f = open(json_file)
    info_dict = json.loads(f.read())

    info_dict = collections.OrderedDict(sorted(info_dict.items()))

    for pid in info_dict:
        print pid, info_dict[pid]['name']
        send_robot_tts_cmd(info_dict[pid]['name'])

        sleep(3)

    


if __name__ == "__main__":
   main(sys.argv[1:])
