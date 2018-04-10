#!/usr/bin/python
import json
import rospy
from std_msgs.msg import String
import sys,os
from time import sleep

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
json_file = "participant_info.json"


def main(info):

    #print info
    rospy.init_node('GameCommand', anonymous=False)
    publisher = rospy.Publisher('/to_twisted', String, queue_size=1)
    sleep(0.5)

    f = open(json_file)
    info_dict = json.loads(f.read())

    #msg = "pid:"+info[0]+",condition:"+info_dict[info[0]]['condition']+",world:w"+info[1]+","+info[2]
    #print msg

    #publisher.publish(msg)

    #print "Sent "+info[2]+" command for "+info[0]+" Session " + info[1]
    #print

    try:
        while True:
            print "Enter 's' or 'c'"
            command = raw_input(">> ")

            if command == "s":
                command = "start"
            elif command == "c":
                command = "continue"

            if command == 'start' or command == 'continue':
                msg = "pid:" + info[0] + ",condition:" + info_dict[info[0]]['condition'] + ",world:w" + info[1] + "," + command
                publisher.publish(msg)

                print "Sent " + command + " command for " + info[0] + " Session " + info[1]
                print
            else:
                print "wrong command!!!"
                print

    except KeyboardInterrupt:
        pass



if __name__ == "__main__":
   main(sys.argv[1:])
