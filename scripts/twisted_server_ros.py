#!/usr/bin/env python
# install_twisted_rector must be called before importing  and using the reactor
from kivy.support import install_twisted_reactor
install_twisted_reactor()


from twisted.internet import reactor
from twisted.internet import protocol

import rospy
from std_msgs.msg import String

import json


class EchoProtocol(protocol.Protocol):
    def dataReceived(self, data):
        response = self.factory.app.handle_message(data, self)

    def sendMessage(self, msg):
        self.transport.write(msg)


class EchoFactory(protocol.Factory):
    def __init__(self, app):
        self.app = app
        self.protocol = EchoProtocol


from kivy.app import App
from kivy.uix.label import Label


class TwistedServerApp(App):
    publishers = {}
    factory = None
    label = None
    protocol = None

    def build(self):
        self.label = Label(text="server started\n")
        self.factory = EchoFactory(self)
        reactor.listenTCP(8000, self.factory)
        rospy.init_node('twisted_node')
        rospy.Subscriber("to_twisted", String, self.transmit_msg)
        return self.label

    def handle_message(self, msg, protocol_in):
        self.protocol = protocol_in
        self.label.text = "received:  %s\n" % msg
        try:
            msgs = []
            print(msg)
            spl = msg.split('}{')
            print(spl)
            for k in range(0, len(spl)):
                the_msg = spl[k]
                if k > 0:
                    the_msg = '{' + the_msg
                if k < (len(spl)-1):
                    the_msg = the_msg + '}'
                print(the_msg)
                msgs.append(json.loads(the_msg))
            for m in msgs:
                for topic, message in m.items():
                    topic = str(topic)
                    message = str(message)
                    if topic not in self.publishers:
                        self.publishers[topic] = rospy.Publisher(topic, String, queue_size=10)
                    self.publishers[topic].publish(message)
        except:
            if 'from_twisted' not in self.publishers:
                self.publishers['from_twisted'] = rospy.Publisher('from_twisted', String, queue_size=10)
            self.publishers['from_twisted'].publish(msg)
        self.label.text += "published: %s\n" % msg
        self.protocol.sendMessage(msg)
        return msg

    def transmit_msg(self, data):
        print('transmitting ', data.data)
        self.label.text = 'transmitting ' + str(data.data)
        if self.protocol:
            self.protocol.sendMessage(data.data)




if __name__ == '__main__':
    TwistedServerApp().run()