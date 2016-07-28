import rospy
from r1d1_msgs.msg import TegaAction # ROS msgs
from r1d1_msgs.msg import TegaState # ROS msgs
import json


class Tega:

    publisher = None
    server = None
    animations = []
    expression = None

    def __init__(self, server=None):
        self.server = server
        self.publisher = rospy.Publisher('tega', TegaAction, queue_size=10)
        rospy.Subscriber('tega_state', TegaState, self.on_tega_state_msg)
        self.sound = {'started': False, 'playing': False, 'stopped': False, 'file': None}
        self.motion = {'started': False, 'playing': False, 'stopped': False}

    def publish(self, message):
        print('tega: ', message)
        self.animations = message[1][1:]
        self.expression = message[0]
        print('expression:', self.expression)
        print('animation:', self.animations)
        self.update_animations()

    def send_motion_message(self, motion):
        """ Publish TegaAction do motion message """
        print 'sending motion message: %s' % motion
        msg = TegaAction()
        msg.do_motion = True
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
        msg.do_sound_playback = True
        msg.wav_filename = speech
        self.publisher.publish(msg)
        rospy.loginfo(msg)
        self.sound['file'] = speech

    def on_tega_state_msg(self, data):
        self.sound['started'] = data.is_playing_sound and not self.sound['playing']
        self.sound['stopped'] = self.sound['playing'] and not data.is_playing_sound
        self.sound['playing'] = data.is_playing_sound

        self.motion['started'] = data.doing_motion and not self.motion['playing']
        self.motion['stopped'] = self.motion['playing'] and not data.doing_motion
        self.motion['playing'] = data.doing_motion

        if self.motion['stopped'] or (not self.motion['playing'] and self.sound['stopped']):
            print(self.sound, self.motion)
            self.update_animations()

    def update_animations(self):
        print('remaining animations:', self.animations, len(self.animations))
        if len(self.animations) > 0:
            current_animation = self.animations[0]
            self.animations = self.animations[1:]

            if 'lookat_' in current_animation:
                self.send_lookat_message(current_animation.replace('lookat_', ''))
            elif current_animation.isupper():
                self.send_motion_message(current_animation)
            elif current_animation.islower():
                self.send_speech_message(current_animation)
        else:
            try:
                msg = {'tega': ["express", self.expression]}
                print('trying to send a message:', msg)
                print('server', self.server)
                print('protocol', self.server.protocol)
                self.server.protocol.sendMessage(str(json.dumps(msg)))
                print('sent back: ', msg)
            except:
                print('failed to send the message...')
            self.expression = None
            self.sound['file'] = None