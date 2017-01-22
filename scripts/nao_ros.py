import rospy
from std_msgs.msg import String
from naoqi import ALProxy
import sys
import almath


class NaoNode():
    def __init__(self):
        # self.robotIP = '192.168.0.100'
        self.robotIP = '192.168.0.100'
        self.port = 9559

        try:
            self.motionProxy = ALProxy("ALMotion", self.robotIP, self.port)
            self.audioProxy = ALProxy("ALAudioPlayer", self.robotIP, self.port)
            tracker = ALProxy("ALTracker", self.robotIP, self.port)
            self.tts = ALProxy("ALTextToSpeech", self.robotIP, self.port)
        except Exception,e:
            print "Could not create proxy to ALMotion"
            print "Error was: ",e
            sys.exit(1)

        # Get the Robot Configuration
        self.robotConfig = self.motionProxy.getRobotConfig()
        self.motionProxy.rest()
        self.motionProxy.setStiffnesses("Body", 1.0)

        self.publisher = rospy.Publisher('nao_state', String, queue_size=10)


    def start(self):
        #init a listener to kinect and
        rospy.init_node('nao_listener')
        rospy.Subscriber('nao_commands', String, self.callback_nih)
        rospy.spin()


    def callback_nih(self, data):
        print("callback_nih", data.data)
        # self.tts.say(data.data)
        # self.tts.isRunning()
        action = data.data
        if action.lower() == action:
            self.audioProxy.playFile('/home/nao/naoqi/wav/nih_howie/howie_wav/' + data.data + '.wav',1.0,0.0)
        else:
            self.do_animation(action)
        print('finished ', data.data)
        self.publisher.publish(data.data)

    def do_animation(self, action):
        if action == 'LOOKAT_TABLET':
            self.change_pose('HeadPitch;29.0;0.1')
        elif action == 'LOOKAT_CHILD':
            self.change_pose('HeadPitch;0.0;0.1')
        elif action == 'POSE_FORWARD':
            self.change_pose('HeadPitch;0.0;0.1')
        elif action == 'EXCITED':
            self.change_pose('HeadPitch,RShoulderPitch;-10.0,-50.0;0.5')
            self.change_pose('HeadPitch,RShoulderPitch;0.0,70.0;0.5')
        elif action == 'LEFTRIGHTLOOKING':
            self.change_pose('HeadYaw;-50.0;0.2')
            self.change_pose('HeadYaw;50.0;0.2')
            self.change_pose('HeadYaw;0.0;0.1')
        elif action == 'HAPPY_UP':
            self.change_pose('HeadPitch,HeadYaw,RShoulderPitch,LShoulderPitch;-10.0,-10.0,-50.0,-50.0;0.5')
            self.change_pose('HeadPitch,HeadYaw,RShoulderPitch,LShoulderPitch;0.0,0.0,70.0,70.0;0.5')
        elif action == 'PROUD':
            self.change_pose('RShoulderPitch,RElbowYaw;-50.0,-50.0;0.2')
            self.change_pose('RShoulderPitch,RElbowYaw;50.0,-10.0;0.2')
        elif action == 'SAD':
            self.change_pose('HeadPitch,HeadYaw;10.0,10.0;0.05')
            self.change_pose('HeadPitch,HeadYaw;0.0,0.0;0.1')

    def change_pose(self, data_str):
        # data_str = 'name1, name2;target1, target2;pMaxSpeedFraction'

        info = data_str.split(';')

        pNames = info[0].split(',')

        pTargetAngles = [float(x) for x in info[1].split(',')]
        pTargetAngles = [ x * almath.TO_RAD for x in pTargetAngles]             # Convert to radians

        pMaxSpeedFraction = float(info[2])

        print(pNames, pTargetAngles, pMaxSpeedFraction)
        self.motionProxy.post.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)

nao = NaoNode()
nao.start()