import rospy
from jibo_msgs.msg import JiboState, JiboAction, JiboVec3, JiboAsrCommand
from std_msgs.msg import Header  # standard ROS msg header
import json
import time


class Jibo:

    publisher = None
    server = None
    animations = []
    current_animation = None

    def __init__(self, server=None):
        self.server = server
        self.robot_commander = rospy.Publisher('/jibo', JiboAction, queue_size=10)
        # rospy.Subscriber('/jibo_state', JiboState, self.on_jibo_state_msg)
        self.sound = {'started': False, 'playing': False, 'stopped': False, 'file': None}
        self.motion = {'started': False, 'playing': False, 'stopped': False, 'name': None}
        print('Finished initializing Jibo')

    def publish(self, message):
        print('jibo: ', message)
        self.animations.append({'expression': message[0], 'sequence': message[1][0:1]})
        print('animation:', self.animations)
        self.update_animations()

    def send_robot_motion_cmd(self, command):
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

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)

    def send_robot_tts_cmd(self, text, *args):
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

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)
        print("\nSent TTS message: ", text)

    def send_robot_lookat_cmd(self, x, y, z):
        """
        send a Motion Command to Jibo
        """

        msg = JiboAction()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()


        msg.do_motion = False
        msg.do_tts = False
        msg.do_lookat = True

        position = JiboVec3(x, y, z)

        msg.look_at = position

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)

    def send_robot_audio_cmd(self, audio_command):
        """
        send a Motion Command to Jibo
        """

        msg = JiboAction()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()


        msg.do_motion = False
        msg.do_tts = False
        msg.do_lookat = False
        msg.do_sound_playback = True

        msg.audio_filename = audio_command

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)

    def send_robot_audio_motion_cmd(self, a, m):
        """
        send LED ring color Command to Jibo
        """

        msg = JiboAction()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.do_sound_playback = True
        msg.do_motion = True

        msg.audio_filename = a
        msg.motion = m

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)


    def send_robot_led_cmd(self, r, g, b):
        """
        send LED ring color Command to Jibo
        """

        msg = JiboAction()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.do_led = True

        color = JiboVec3(r, g, b)
        msg.led_color = color

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)

    def send_robot_volume_cmd(self, v):
        """
        send LED ring color Command to Jibo
        """

        msg = JiboAction()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.do_volume = True

        msg.volume = v

        self.robot_commander.publish(msg)
        rospy.loginfo(msg)

    def send_robot_asr_cmd(self, cmd, heyjibo=False, continuous=False, rule=""):

        """
        send ASR Command to Jibo
        """

        msg = JiboAsrCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.heyjibo = heyjibo
        msg.continuous = continuous

        msg.rule = rule
        msg.command = cmd

        self.robot_asr_commander.publish(msg)
        rospy.loginfo(msg)

    def send_robot_anim_transition_cmd(self, tran):

        msg = JiboAction()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()

        msg.do_anim_transition = True
        msg.anim_transition = tran
        self.robot_commander.publish(msg)
        rospy.loginfo(msg)

    # def on_jibo_state_msg(self, data):
    #     self.sound['started'] = data.is_playing_sound and not self.sound['playing']
    #     self.sound['stopped'] = self.sound['playing'] and not data.is_playing_sound
    #     self.sound['playing'] = data.is_playing_sound
    #
    #     self.motion['started'] = data.doing_motion and not self.motion['playing']
    #     self.motion['stopped'] = (self.motion['playing'] and not data.doing_motion)
    #     self.motion['playing'] = data.doing_motion
    #
    #     # print('on_jibo_state_msg', self.motion, data.in_motion)
    #
    #     if not self.sound['playing'] and not self.motion['playing']:
    #         if self.motion['stopped'] or (not self.motion['playing'] and self.sound['stopped']):
    #             print(self.sound, self.motion)
    #             self.update_animations()

    def update_animations(self):
        print('\nremaining animations:', self.animations, len(self.animations))
        if len(self.animations[0]['sequence']) > 0:
            self.current_animation = self.animations[0]['sequence'][0]
            self.animations[0]['sequence'] = self.animations[0]['sequence'][1:]

            print self.current_animation
            # send TTS command
            if ' ' in self.current_animation:
                self.send_robot_tts_cmd(self.current_animation)
            self.update_animations()
        #    elif 'lookat_' in self.current_animation:
        #        self.send_lookat_message(self.current_animation.replace('lookat_', ''))
        #    elif self.current_animation.isupper():
        #        self.send_motion_message(self.current_animation)
        #         if 'POSE' in self.current_animation or 'IDLESTILL' in self.current_animation:
        #             self.update_animations()
        #     elif self.current_animation.islower():
        #         self.send_speech_message(self.current_animation)
        else:
            try:
                msg = {'jibo': ["express", self.animations[0]['expression']]}
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
        # self.sound['file'] = None


