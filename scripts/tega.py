import rospy
from r1d1_msgs.msg import TegaAction # ROS msgs
from r1d1_msgs.msg import TegaState # ROS msgs



class Tega:

    publisher = None
    server = None

    def __init__(self, server=None):
        self.server = server
        self.publisher = rospy.Publisher('tega', TegaAction, queue_size=10)
        rospy.Subscriber('tega_state', TegaState, self.on_tega_state_msg)
        self.sound = {'started': False, 'playing': False, 'stopped': False, 'file': None}
        self.motion = {'started': False, 'playing': False, 'stopped': False}

    def publish(self, message):
        print('tega: ', message)
        if message[0] == 'motion':
            self.send_motion_message(message[1])
        elif message[0] == 'lookat':
            self.send_lookat_message(message[1])
        else:
            self.send_speech_message(message[1])

    def send_motion_message(self, motion):
        """ Publish TegaAction do motion message """
        print 'sending motion message: %s' % motion
        msg = TegaAction()
        msg.do_motion = True
        msg.motion = motion
        self.tega_pub.publish(msg)
        rospy.loginfo(msg)

    def send_lookat_message(self, lookat):
        """ Publish TegaAction lookat message """
        print 'sending lookat message: %s' % lookat
        msg = TegaAction()
        msg.do_look_at = True
        msg.look_at = lookat
        self.tega_pub.publish(msg)
        rospy.loginfo(msg)

    def send_speech_message(self, speech):
        """ Publish TegaAction playback audio message """
        print '\nsending speech message: %s' % speech
        msg = TegaAction()
        msg.do_sound_playback = True
        msg.wav_filename = speech
        self.tega_pub.publish(msg)
        rospy.loginfo(msg)
        self.sound['file'] = speech

    def on_tega_state_msg(self, data):
        self.sound['started'] = data.is_playing_sound and not self.sound['playing']
        self.sound['stopped'] = self.sound['playing'] and not data.is_playing_sound
        self.sound['playing'] = data.is_playing_sound

        self.motion['started'] = data.doing_motion and not self.motion['playing']
        self.motion['stopped'] = self.motion['playing'] and not data.doing_motion
        self.motion['playing'] = data.doing_motion

        if self.sound['stopped']:
            try:
                msg = {'tega': ["express", self.sound['file']]}
                self.server.protocol.sendMessage(msg)
            except:
                pass
            self.sound['file'] = None