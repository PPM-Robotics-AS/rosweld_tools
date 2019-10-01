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

from threading import Thread, currentThread
import datetime
import time
import rospy
from pydispatch import dispatcher
from std_srvs.srv import Empty
from rosweld_drivers.msg import RobotState
from rosweld_drivers.srv import (MoveAlong, MoveBetween, MoveBetweenRequest,
                                 SetSpeed, SetSpeedRequest)

from src.rosweld.rosproxy import RosProxy
from src.rosweld.signals import Signals

class RobotController(object):
    """Robot controller class to handle communication between
    a robot driver and rosweld
    """

    def __init__(self, name):
        """Initalizing a new robot controller
           Subscribing to pose and step topics
        """

        self.name = name
        self.moves = []
        self.current_state = None
        self.last_update = None
        self.subscriber = None

        # Starts the robot pose publisher on a new thread
        self.update_thread = Thread(target=self.check_alive)
        self.update_thread.do_run = True
        self.update_thread.start()

    def __del__(self):
        """Delete RobotController object
        """

        self.update_thread.do_run = False
        self.update_thread.join()


    def check_alive(self):
        """Check if the thread is still alive
        """

        t = currentThread()
        while not rospy.is_shutdown() and getattr(t, "do_run", True):

            if self.last_update is None or \
                (datetime.datetime.now() - self.last_update).seconds > 2:
                self.subscriber = RosProxy().subscribe_topic(
                    "/%s/robot_state" %
                    (self.name),
                    RobotState,
                    self.set_current_state)

            time.sleep(2)

    def set_current_state(self, data):
        """Current pose update handler

        Arguments:
            data {RobotState} -- Robot state
        """

        self.current_state = data
        self.last_update = datetime.datetime.now()

        dispatcher.send(
            signal=Signals['STATE_CHANGED'], sender=self, name=self.name
        )

    def store_poses(self, moves):
        """Send the poses to the robot

        Arguments:
            moves {rosweld_tools/Move[]} -- path poses and their data
        """

        RosProxy().call_service(
            "/%s/store_poses" %
            (self.name), MoveAlong, moves)

    def move_along(self, moves):
        """Move the robot on a given path

        Arguments:
            moves {rosweld_tools/Move[]} -- path poses and their data
        """

        RosProxy().call_service(
            "/%s/move_along" %
            (self.name), MoveAlong, moves)

    def move_pose(self, moves):
        """Move along the robot on a given path

        Arguments:
            moves {rosweld_tools/Move[]} -- path poses and their data
        """

        RosProxy().call_service(
            "/%s/move_pose" %
            (self.name), MoveAlong, moves)

    def move_between(self, start, end, handler=None):
        """Move on the path

        Arguments:
            start: start index on the path
            end: end index on the path
            handler: result handler
        """

        msg = MoveBetweenRequest()
        msg.start = start
        msg.end = end

        RosProxy().call_service("/%s/move_between" %
                                (self.name), MoveBetween, msg, handler)

    def abort(self):
        """Abort current movement
        """

        RosProxy().call_service("/%s/abort" % (self.name), Empty, None)

    def set_speed(self, speed):
        """Set the speed on the robot
        """

        msg = SetSpeedRequest()
        msg.value = int(speed)
        RosProxy().call_service("/%s/set_speed" % (self.name), SetSpeed, msg)
