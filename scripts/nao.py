import rospy
from std_msgs.msg import String
import json
import time


class Nao:

    publisher = None
    server = None
    animations = []
    current_animation = None

    def __init__(self, server=None):
        self.server = server
        self.publisher = rospy.Publisher('nao_commands', String, queue_size=10)
        rospy.Subscriber('nao_state', String, self.on_nao_state_msg)
        self.current_animation = None
        self.doable_animations = None
        self.current_animation_seq = None
        print('Finished initializing Nao')


    def publish(self, message):
        # print('nao: ', message)
        # message = eval(message)
        self.current_animation_seq = message[0]
        self.current_animation = message[1]
        print('nao: ', message)
        self.doable_animations = []
        for robot_action in self.current_animation[1:]:
            if robot_action.upper() == robot_action:
                # pose
                pass
            else:
                self.doable_animations.append(robot_action)

        if len(self.doable_animations) == 0:
            self.send_finish_animation_sequence()
        else:
            for robot_action in self.doable_animations:
                # filename
                print('play filename: ', robot_action)
                self.publisher.publish(robot_action)

    def on_nao_state_msg(self, data):
        print('done: ', data.data, self.doable_animations)
        if data.data == self.doable_animations[-1]:
            self.send_finish_animation_sequence()


    def send_finish_animation_sequence(self):
        try:
            msg = {'nao': ["express", self.current_animation_seq]}
            print('trying to send a message:', msg)
            print('server', self.server)
            print('protocol', self.server.protocol)
            self.server.protocol.sendMessage(str(json.dumps(msg)))
            print('sent back: ', msg)
        except:
            print('failed to send the message...')