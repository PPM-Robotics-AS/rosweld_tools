"""
ROSWELD
Version 0.0.1, March 2019
http://rosin-project.eu/ftp/rosweld

Copyright (c) 2019 PPM Robotics AS

This library is part of ROSWELD project,
the Focused Technical Project ROSWELD - ROS based framework
for planning, monitoring and control of multi-pass robot welding
is co-financed by the EU project ROSIN (www.rosin-project.eu)
and ROS-Industrial initiative (www.rosindustrial.eu).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import datetime
import rospy

from rosweld_drivers.msg import Status
from rosweld_drivers.srv import SendStatus

from src.rosweld.singleton import Singleton

class STATE(object):
    ERROR = 2
    INFO = 1
    DEBUG = 0
    WARNING = 3

class RosProxy(object):
    """ROS Proxy sigleton class, handles the registration to services and topics

    Arguments:
        object {object} -- base class
    """

    __metaclass__ = Singleton

    def get_param(self, name, value=None):
        try:
            private = "~%s" % name
            if rospy.has_param(private):
                return rospy.get_param(private)
            elif rospy.has_param(name):
                return rospy.get_param(name)
            return value

        except BaseException:
            return value

    def call_service(self, service, service_type, data=None, handler=None):
        """Call a ros service with given params

        Arguments:
            service {string} -- name of the service
            service_type {type} -- type of the service
            data {object} -- service parameter
        """

        try:
            srv = rospy.ServiceProxy(service, service_type)
            if data is not None:
                result = srv(data)
            else:
                result = srv()

            if handler is not None:
                handler(result)
        except:
            self.notify("%s service is not available"%(service), STATE.ERROR)

    def subscribe_topic(self, topic, topic_type, callback):
        """Subscribe to ROS topic

        Arguments:
            topic {string} -- topic to subscribe
            topic_type {type} -- message type of the topic
            callback {function} -- topic update handler function
        """

        return rospy.Subscriber(topic, topic_type, callback=callback)

    def advertise_service(self, service, service_type, callback):
        """Advertise a new ROS service

        Arguments:
            service {string} -- name of the service
            service_type {type} -- type of the service
            callback {function} -- service call handler
        """

        rospy.Service(service, service_type, callback)

    def publish_topic(self, topic, topic_type, data, latch=False):
        """Publish a topic to ROS

        Arguments:
            topic {string} -- name of the topic
            topic_type {type} -- message type of the topic
            data {object} -- new value of the topic
        Keyword Arguments:
            latch {bool} -- send the last known value after connection (default: {False})
            publisher {Publisher} -- ROS topic publisher to use (default: {None})

        Returns:
            Publisher -- ROS topic publisher
        """

        if topic not in self.publishers:
            publisher = rospy.Publisher(topic, topic_type, queue_size=10, latch=latch)
            self.publishers[topic] = {'publisher': publisher}

        self.publishers[topic]['publisher'].publish(data)
        self.publishers[topic]['data'] = data

    def publish_last(self):
        """Republish all known topics
        """

        for topic in self.publishers:
            data = self.publishers[topic]['data']
            self.publishers[topic]['publisher'].publish(data)

    def notify(self, msg, prio=STATE.INFO):
        """Send notification to nodes

        Arguments:
            msg {string} -- message
            prio {int} -- priority
        """

        msg = Status(
            message="[%s] %s - %s" % (self.name, datetime.datetime.now(), msg),
            node=self.name,
            priority=prio)

        try:
            if prio == 0:
                rospy.logdebug(msg.message)
            elif prio == 1:
                rospy.loginfo(msg.message)
            else:
                rospy.logerr(msg.message)

            srv = rospy.ServiceProxy("/notify", SendStatus)
            srv.call(msg)
        except:
            return

    def __init__(self):
        """Class initializer to register a new node in ROS
        """

        self.name = self.get_param("name", "rosweld")
        rospy.init_node(self.name)

        self.publishers = {}
