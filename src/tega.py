import rospy
from r1d1_msgs.msg import TegaAction # ROS msgs
from r1d1_msgs.msg import TegaState # ROS msgs
import json
import time


class Tega:

    publisher = None
    server = None
    animations = []
    current_animation = None

    def __init__(self, server=None):
        self.server = server
        self.publisher = rospy.Publisher('tega', TegaAction, queue_size=10)
        rospy.Subscriber('tega_state', TegaState, self.on_tega_state_msg)
        self.sound = {'started': False, 'playing': False, 'stopped': False, 'file': None}
        self.motion = {'started': False, 'playing': False, 'stopped': False, 'name': None}
        print('Finished initializing Tega')

    def publish(self, message):
        print('tega: ', message)
        self.animations.append({'expression': message[0], 'sequence': message[1][1:]})
        print('animation:', self.animations)
        self.update_animations()

    def send_motion_message(self, motion):
        """ Publish TegaAction do motion message """
        print 'sending motion message: %s' % motion
        msg = TegaAction()
        msg.motion = motion
        self.publisher.publish(msg)
        rospy.loginfo(msg)

    def send_lookat_message(self, lookat):
        """ Publish TegaAction lookat message """
        print 'sending lookat message: %s' % lookat
        msg = TegaAction()
        msg.do_look_at = True
        msg.look_at = lookat
        self.publisher.publish(msg)
        rospy.loginfo(msg)

    def send_speech_message(self, speech):
        """ Publish TegaAction playback audio message """
        print '\nsending speech message: %s' % speech
        msg = TegaAction()
        msg.wav_filename = 'mindset2/' + speech + '.wav'
        self.publisher.publish(msg)
        rospy.loginfo(msg)
        self.sound['file'] = speech

    def on_tega_state_msg(self, data):
        self.sound['started'] = data.is_playing_sound and not self.sound['playing']
        self.sound['stopped'] = self.sound['playing'] and not data.is_playing_sound
        self.sound['playing'] = data.is_playing_sound

        self.motion['started'] = data.doing_motion and not self.motion['playing']
        self.motion['stopped'] = (self.motion['playing'] and not data.doing_motion)
        self.motion['playing'] = data.doing_motion

        # print('on_tega_state_msg', self.motion, data.in_motion)

        if not self.sound['playing'] and not self.motion['playing']:
            if self.motion['stopped'] or (not self.motion['playing'] and self.sound['stopped']):
                print(self.sound, self.motion)
                self.update_animations()

    def update_animations(self):
        print('remaining animations:', self.animations, len(self.animations))
        if len(self.animations[0]['sequence']) > 0:
            self.current_animation = self.animations[0]['sequence'][0]
            self.animations[0]['sequence'] = self.animations[0]['sequence'][1:]

            if 'lookat_' in self.current_animation:
                self.send_lookat_message(self.current_animation.replace('lookat_', ''))
            elif self.current_animation.isupper():
                self.send_motion_message(self.current_animation)
                if 'POSE' in self.current_animation or 'IDLESTILL' in self.current_animation:
                    self.update_animations()
            elif self.current_animation.islower():
                self.send_speech_message(self.current_animation)
        else:
            try:
                msg = {'tega': ["express", self.animations[0]['expression']]}
                print('trying to send a message:', msg)
                print('server', self.server)
                print('protocol', self.server.protocol)
                self.server.protocol.sendMessage(str(json.dumps(msg)))
                print('sent back: ', msg)
            except:
                print('failed to send the message...')
            self.animations = self.animations[1:]
            if len(self.animations)  > 0:
                self.update_animations()
            else:
                print('NO MORE ANIMATIONS')
        self.sound['file'] = None
