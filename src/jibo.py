import rospy
from jibo_msgs.msg import JiboState, JiboAction, JiboVec3, JiboAsrCommand
from std_msgs.msg import Header  # standard ROS msg header
import json
import time

IDLE_MAX = 5

class Jibo:

    publisher = None
    server = None
    animations = []
    current_animation = None

    def __init__(self, server=None):
        self.server = server
        self.robot_commander = rospy.Publisher('/jibo', JiboAction, queue_size=10)
        rospy.Subscriber('/jibo_state', JiboState, self.on_jibo_state_msg)
        self.sound = {'started': False, 'playing': False, 'stopped': False, 'file': None}
        self.motion = {'started': False, 'playing': False, 'stopped': False, 'name': None}
        self.idle_counter = 0
        self.play_anim = False
        self.anim_tran_fixed = False
        self.playing_tts = ''
        self.send_robot_anim_transition_cmd(JiboAction.ANIMTRANS_RESET)
        print('Finished initializing Jibo')

    def publish(self, message):
        print('jibo: ', message)
        self.animations.append({'expression': message[0], 'sequence': message[1][0:1]})
        print('animation:', self.animations)
        if self.anim_tran_fixed:
            self.send_robot_anim_transition_cmd(JiboAction.ANIMTRANS_RESET)
            self.anim_tran_fixed = False
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

    def jibo_sleep(self):
        self.send_robot_motion_cmd("Poses/Directional/Body_Look_Center_Down_01_01.keys")
        self.send_robot_anim_transition_cmd(JiboAction.ANIMTRANS_KEEP_LASTFRAME)
        self.send_robot_motion_cmd("Eye-Globals/open-to-close_01.keys")
        self.anim_tran_fixed = True

    def on_jibo_state_msg(self, data):

        self.playing_tts = data.tts_msg

        if self.play_anim:
            if not data.is_playing_sound and not data.doing_motion:
                self.idle_counter += 1

            if self.idle_counter > IDLE_MAX:
                self.idle_counter = 0
                self.play_anim = False
                if self.current_expression in ['explain_move','ask_question_robot_play','my_turn','comment_selection','comment_move']:
                    self.send_robot_motion_cmd("Poses/Directional/Body_Look_Center_Down_01_01.keys")
                elif self.current_expression == 'end_party':
                    self.jibo_sleep()
                time.sleep(0.5)
                self.update_animations()

    def update_animations(self):
        print('\nremaining animations:', self.animations, len(self.animations))
        if len(self.animations) > 0 and len(self.animations[0]['sequence']) > 0:
            if self.animations[0]['expression'] == "tega_init":
                self.jibo_sleep()
                print "put jibo sleep"
                return
            
            self.current_animation = self.animations[0]['sequence'][0]
            print "current animation: "+self.current_animation
            self.current_animation = self.current_animation.encode('ascii', 'ignore').replace("idle","").split(':')[-1]
            self.send_robot_tts_cmd(self.current_animation)

            self.current_expression = self.animations[0]['expression']
            self.animations[0]['sequence'] = self.animations[0]['sequence'][1:]


            counter = 0
            while (self.playing_tts != self.current_animation):
                print "not identical: ", self.current_animation, self.playing_tts
                if counter == 25:    # wait for 5 seconds, resend command once more
                    self.send_robot_tts_cmd(self.current_animation)
                elif counter == 40:     # wait for 8 seconds, and move on
                    self.update_animations()
                    break
                counter += 1
                time.sleep(0.2)

            self.play_anim = True

            #self.update_animations()

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


