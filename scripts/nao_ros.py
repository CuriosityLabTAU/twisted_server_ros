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
        self.motionProxy.setStiffnesses("Body", 1.0)
        # self.motionProxy.rest()

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
        self.audioProxy.playFile('/home/nao/naoqi/wav/nih_howie/howie_wav/' + data.data + '.wav',1.0,0.0)
        print('finished ', data.data)
        self.publisher.publish(data.data)


    def callback_pysical(self, data):
        # data = 'name1, name2;target1, target2;pMaxSpeedFraction'
        data_str = data.data
        info = data_str.split(';')

        pNames = info[0].split(',')

        pTargetAngles = [float(x) for x in info[1].split()]
        pTargetAngles = [ x * almath.TO_RAD for x in pTargetAngles]             # Convert to radians

        pMaxSpeedFraction = float(info[2])

        print(pNames, pTargetAngles, pMaxSpeedFraction)
        self.motionProxy.angleInterpolationWithSpeed(pNames, pTargetAngles, pMaxSpeedFraction)

nao = NaoNode()
nao.start()