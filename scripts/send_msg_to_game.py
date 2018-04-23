#!/usr/bin/python
import json
import rospy
from std_msgs.msg import String
import sys,os
from time import sleep, time
import rospy
from jibo_msgs.msg import JiboState
import threading

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
json_file = "participant_info.json"

counter = 0
motion_stuck_counter = 0
prev_in_motion = ''
jibo_state_freq = 0
FREQ_THRESHOLD = 9
STUCK_THRESHOLD = 30 * 5

def on_jibo_state_msg(data):
    global counter, motion_stuck_counter, prev_in_motion
    counter += 1

    if data.doing_motion and data.in_motion == prev_in_motion:
        motion_stuck_counter += 1
    else:
        motion_stuck_counter = 0

    prev_in_motion = data.in_motion

    if motion_stuck_counter > STUCK_THRESHOLD:
        motion_stuck_counter = 0
        print '\033[92m\n[Troubleshoot J-3] Jibo is stuck while playing a previous motion.\n' \
              '1) Restart jibo-ros ($ jibo run -n) (Mac)\n ' \
              '2) skip (k) or continue (c) as necessary\033[0m'


def topic_freq():
    global counter, jibo_state_freq
    threading.Timer(5.0, topic_freq).start()
    jibo_state_freq = counter/5.0
    counter = 0

    if jibo_state_freq < FREQ_THRESHOLD:
        print '\033[92m\n[Troubleshoot J-2] JiboState Message is not publishing.\n' \
              '1) CTRL-c and restart rosbridge on 3rd tab (Linux)\n' \
              '  or\n1) Close TwistedServer (black window) and restart twisted_server on 4th tab (Linux)\n' \
              '  or\n1) Restart jibo-ros ($ jibo run -n) (Mac)\n' \
              '  then\n2) skip (k) or continue (c) as necessary\033[0m'



def main(info):

    #print info
    rospy.init_node('GameCommand', anonymous=False)
    publisher = rospy.Publisher('/to_twisted', String, queue_size=1)
    rospy.Subscriber('/jibo_state', JiboState, on_jibo_state_msg)

    sleep(0.5)

    f = open(json_file)
    info_dict = json.loads(f.read())

    #msg = "pid:"+info[0]+",condition:"+info_dict[info[0]]['condition']+",world:w"+info[1]+","+info[2]
    #print msg

    #publisher.publish(msg)

    #print "Sent "+info[2]+" command for "+info[0]+" Session " + info[1]
    #print

    topic_freq()

    try:
        while True:
            print "Enter 's (start)', 'k (skip)', or 'c (continue)'"
            command = raw_input(">> ")

            if command == "s":
                command = "start"
            elif command == "c":
                command = "continue"
            elif command == "k":
                command = "skip"

            if command == 'start' or command == 'continue' or command == 'skip':
                msg = {"pid": info[0],
                       "pname": info_dict[info[0]]['name'],
                       "robot": info_dict[info[0]]['robot'],
                       "condition": info_dict[info[0]]['condition'],
                       "world": 'w' + info[1],
                       "entry": command
                       }
                publisher.publish(json.dumps(msg))

                print "Sent " + command + " command for " + info[0] + " Session " + info[1]
                print
            else:
                print "wrong command!!!"
                print

    except KeyboardInterrupt:
        pass



if __name__ == "__main__":
   main(sys.argv[1:])
