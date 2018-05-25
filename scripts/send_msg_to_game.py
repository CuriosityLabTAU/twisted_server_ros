#!/usr/bin/python
import json
import rospy
from std_msgs.msg import String
import sys,os
from time import sleep, time, strftime, gmtime
from std_msgs.msg import Header 
from jibo_msgs.msg import JiboState, JiboAction
import threading

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
json_file = "participant_info.json"

counter = 0
motion_stuck_counter = 0
prev_in_motion = ''
jibo_state_freq = 0
FREQ_THRESHOLD = 5
STUCK_THRESHOLD = 5

subscriber = None
jibo_publisher = None

s8_script=[
        "<es name='Yawn_02' />  Oh, <participant-name>! Glad to see you. <es cat='happy'/> I'm <es name='Emoji_Beach'> going on a Summer vacation next week, <break size='0.3'/> so, sadly, this will be our last day</es> together. <es name='Sad_03' /> I definitely had so much fun getting to know you and playing and learning with you. <break size='0.5'/> and we got so many stickers together! Can you show me our sticker sheet?",
        "Yay! Please keep it with you and remember me. <break size='1.0'/> Well, This calls for a celebration! <es name='electronic_house_music_dance_01'/> We made such a great team. <es name='affection_03'/> <es name='dont_understand_02'> Okay, I have to say good bye now, but I have a feeling</es> we will meet again soon. <es cat='happy'/> <break size='2.0'/> bye, <participant-name>. Thanks for being my friend. <break size='0.5'/> please come visit me at my place when you want to! I'll bake pizza <es name='Emoji_Pizza'/> and we will play more games together. <break size='1.0'/> Bye! <es name='goodbye_01'/> <es name='nap-to-sleep-01'/>"
        ]

def on_jibo_state_msg(data):
    global counter, motion_stuck_counter, prev_in_motion
    counter += 1

    if data.doing_motion and data.in_motion == prev_in_motion:
        motion_stuck_counter += 1
    else:
        motion_stuck_counter = 0

    prev_in_motion = data.in_motion

    stuck_limit = STUCK_THRESHOLD * jibo_state_freq

    if motion_stuck_counter > stuck_limit:
        motion_stuck_counter = 0
        print '\033[1;36m\n'+strftime("%M:%S", gmtime())+',[Troubleshoot J-3] Jibo is stuck while playing a previous motion.\n' \
              '1) Restart jibo-ros ($ jibo run -n) (Mac)\n ' \
              '2) skip (k) or continue (c) as necessary\033[0m'


def topic_freq():
    global counter, jibo_state_freq, subscriber
    threading.Timer(3.0, topic_freq).start()
    jibo_state_freq = counter/3.0
    counter = 0

    if jibo_state_freq < FREQ_THRESHOLD:
        print '\033[92m\n'+strftime("%M:%S", gmtime())+',[Troubleshoot J-2] JiboState Message is not publishing.\n\n' \
              '1) CTRL-c and restart rosbridge on 3rd tab (Linux)\n' \
              '  or\n1) Close TwistedServer (black window) and restart twisted_server on 4th tab (Linux)\n' \
              '  or\n1) Restart jibo-ros ($ jibo run -n) (Mac)\n' \
              '  then\n2) skip (k) or continue (c) as necessary\033[0m'
        if subscriber is not None:
            subscriber.unregister()
        subscriber = rospy.Subscriber('/jibo_state', JiboState, on_jibo_state_msg)

def send_robot_tts_cmd(text):
    msg = JiboAction()
    # add header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.do_motion = False
    msg.do_tts = True
    msg.do_lookat = False
    
    msg.tts_text = text

    jibo_publisher.publish(msg)
        #rospy.loginfo(msg)
        #print("\nSent TTS message: ", text)

def send_robot_anim_transition_cmd(tran):

    msg = JiboAction()
    
    # add header   
    msg.header = Header()
    
    msg.header.stamp = rospy.Time.now()
    msg.do_anim_transition = True

    msg.anim_transition = tran
    jibo_publisher.publish(msg)

def send_robot_motion_cmd(command):
    msg = JiboAction()
    # add header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.do_motion = True
    msg.do_tts = False
    msg.do_lookat = False

    msg.motion = command

    jibo_publisher.publish(msg)

def jibo_sleep():
    send_robot_motion_cmd("Poses/Directional/Body_Look_Center_Down_01_01.keys")
    send_robot_anim_transition_cmd(JiboAction.ANIMTRANS_KEEP_LASTFRAME)
    send_robot_motion_cmd("Eye-Globals/open-to-close_01.keys")


def main(info):
    global subscriber, jibo_publisher
    #print info
    rospy.init_node('GameCommand', anonymous=False)
    publisher = rospy.Publisher('/to_twisted', String, queue_size=1)
    jibo_publisher = rospy.Publisher('/jibo', JiboAction, queue_size=1)
    subscriber = rospy.Subscriber('/jibo_state', JiboState, on_jibo_state_msg)

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
                #post-session
                if info[1] == '8':
                    send_robot_anim_transition_cmd(JiboAction.ANIMTRANS_RESET)
                    pname = info_dict[info[0]]['name']
                    send_robot_tts_cmd(s8_script[0].replace("<participant-name>",pname))
                    sleep(35)
                    send_robot_tts_cmd(s8_script[1].replace("<participant-name>",pname))
                    continue

                    
                    
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
