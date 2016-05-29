#!/usr/bin/env python
# install_twisted_rector must be called before importing  and using the reactor
from kivy.support import install_twisted_reactor
install_twisted_reactor()


from twisted.internet import reactor
from twisted.internet import protocol

import rospy
from std_msgs.msg import String

import json
import ast


class EchoProtocol(protocol.Protocol):
    def dataReceived(self, data):
        response = self.factory.app.handle_message(data)
        if response:
            self.transport.write(response)


class EchoFactory(protocol.Factory):
    protocol = EchoProtocol

    def __init__(self, app):
        self.app = app


from kivy.app import App
from kivy.uix.label import Label


class TwistedServerApp(App):
    publishers = {}

    def build(self):
        self.label = Label(text="server started\n")
        reactor.listenTCP(8000, EchoFactory(self))
        rospy.init_node('twisted_node')
        return self.label

    def handle_message(self, msg):
        self.label.text = "received:  %s\n" % msg
        try:
            msgs = []
            spl = msg.split('}{')
            for s in spl:
                the_msg = s
                if s[0] is not '{':
                    the_msg = '{' + the_msg
                if s[-1] is not '}':
                    the_msg = the_msg + '}'
                msgs.append(json.loads(the_msg))
            for m in msgs:
                for topic, message in m.items():
                    if topic not in self.publishers:
                        self.publishers[topic] = rospy.Publisher(topic, String, queue_size=10)
                    self.publishers[topic].publish(message)
        except:
            if 'from_twisted' not in self.publishers:
                self.publishers['from_twisted'] = rospy.Publisher('from_twisted', String, queue_size=10)
            self.publishers['from_twisted'].publish(msg)
        self.label.text += "published: %s\n" % msg
        return msg


if __name__ == '__main__':
    TwistedServerApp().run()