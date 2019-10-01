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

import numpy as np
import rospy
import tf
from pydispatch import dispatcher
from std_srvs.srv import Empty
from rosweld_drivers.msg import RobotState
from rosweld_tools.srv import SetService, SetServiceResponse

from src.rosweld.rosproxy import RosProxy
from src.rosweld.signals import Signals
from src.rosweld.singleton import Singleton
from src.rosweld.robotcontroller import RobotController

class RobotControllerHandler(object):
    """RobotControllerHandler handles multiple robots at the same time
    Makes it possible to switch between different environments
    Centralizes and forwards the communication and messages
    between ROSweld and the robots.

    By default a MoveIt! simulation and a NACHI controller is used.
    """

    __metaclass__ = Singleton

    simulation = True
    current_state = None
    current_step = 0

    def __init__(self):
        """Init the RobotControllerHandler
        """

        # Add the simulation and the robot controller
        self._controllers = {
            'simulation': {
                'controller': RobotController(
                    RosProxy().get_param(
                        "simulation_controller",
                        "move_it_robot")),
                'transformation': None,
                'lastState': None},
            'robot': {
                'controller': RobotController(
                    RosProxy().get_param(
                        "robot_controller",
                        "nachi_robot")),
                'transformation': None,
                'lastState': None}}

        # Subscribe to the robot state changed event
        dispatcher.connect(
            self._handle_robot_state_changed,
            signal=Signals['STATE_CHANGED'],
            sender=dispatcher.Any
        )

        # Advertise the simulation set service to change between simulation and
        # robot control
        RosProxy().advertise_service("set_simulation", SetService, self._set_simulation)

    def update(self):
        """Request update from the connected controllers
        """

        for ctrl in self._controllers:
            controller = self._controllers[ctrl]
            RosProxy().call_service("/%s/update"%(controller['controller'].name), Empty, None)

    def _set_simulation(self, data):
        """Set simulation mode on or off

        Arguments:
            data {SetService} -- basic service type for the communication,
                                 contains the command true / false to enable or disable simulation
        """

        # enable / disable simulation
        if data.command == "true":
            RosProxy().notify("Enabling simulation")
            RobotControllerHandler.simulation = True
        else:
            RosProxy().notify("Disabling simulation")
            RobotControllerHandler.simulation = False

        # publish the new state
        RobotControllerHandler.current_state = self.current_controller.current_state
        self.proxy_robot_state(RobotControllerHandler.current_state)

        return SetServiceResponse()

    def _handle_robot_state_changed(self, name):
        """Forward the currently active robot controller's state

        Arguments:
            name {string} -- Name of the publisher
        """

        # Handle change only from the current controller, drop the rest
        if name == self.current_controller.name:
            RobotControllerHandler.current_state = self.current_controller.current_state

            # Set the isSumulation flag
            RobotControllerHandler.current_state.isSimulation = RobotControllerHandler.simulation

            # Forward the robot state as a general rosweld robot_state
            self.proxy_robot_state(RobotControllerHandler.current_state)

            # Update the current step
            RobotControllerHandler.current_step = RobotControllerHandler.current_state.step

            dispatcher.send(
                signal=Signals['STEP_CHANGED'],
                sender=self,
                step=RobotControllerHandler.current_step)

    def proxy_robot_state(self, state):
        """Forward the robot states to ROS

        Arguments:
            state {RobotState} -- the current robot state to forward
        """

        if state is None:
            return

        # Forward the robot state as a general rosweld robot_state
        state.isSimulation = RobotControllerHandler.simulation
        RosProxy().publish_topic("robot_state", RobotState, state, latch=True)

    @property
    def current_transformation(self):
        """Getter for the current controller's transformation
        """

        # Select the right controller
        if RobotControllerHandler.simulation:
            # Get the ideal transformation

            listener = tf.TransformListener()
            listener.waitForTransform(
                "base_link", "plate", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform(
                "base_link", "plate", rospy.Time(0))

            trans_mat = tf.transformations.translation_matrix(trans)
            rot_mat = tf.transformations.quaternion_matrix(rot)
            trafo = np.dot(trans_mat, rot_mat)

            # Return the ideal tansformation
            return trafo

        return self._controllers['robot']['transformation']

    @property
    def current_controller(self):
        """Getter for the current controller

        Returns:
            RobotController -- the current controller
        """

        # Select the right controller
        if RobotControllerHandler.simulation:
            return self._controllers['simulation']['controller']

        return self._controllers['robot']['controller']

    # Set the current transformation
    def set_transformation(self, transformation):
        """Set the transformation

        Arguments:
            transformation {nparray} -- the current transformation matrix (4x4)
        """

        self._controllers['robot']['transformation'] = transformation
