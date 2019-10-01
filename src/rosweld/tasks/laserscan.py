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
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest

from src.rosweld.tasks.move import MoveTask

from src.rosweld.robotcontrollerhandler import RobotControllerHandler
from src.rosweld.point import Point
from src.rosweld.rosproxy import RosProxy


class LaserScanTask(MoveTask):
    """Laser scanning task

    Arguments:
        MoveTask {MoveTask} -- parent object
    """

    def __setstate__(self, d):
        self.__dict__ = d
        RosProxy().subscribe_topic(
            "/%s/is_scanning" %
            (self.laser_name),
            Bool,
            self.handle_laser_update)

    def __init__(
            self,
            task_type,
            progress,
            desc,
            title,
            step,
            current_bead,
            path):
        """Init laser scanning task

        Arguments:
            ITask {super class} -- super class
            task_type {string} -- type as string
            progress {float} -- progress
            desc {string} -- description
            title {string} -- title
            step {int} -- current step
            current_bead {Bead} -- current bead with modifications
            path {float[6][]} -- Pose array: x, y, z, r, p, y
        """

        super(LaserScanTask, self).__init__(
            task_type, progress, desc, title, step, current_bead, path)

        self.laser_name = RosProxy().get_param("laser_name", "laser_scanner")

        offset = RosProxy().get_param("laser_offset", [0.105, 0., 0.03])
        self.offset = Point(offset[0], offset[1], offset[2])

        rot = RosProxy().get_param("laser_rotation", [0, 0, 0])
        self.rotation = np.array(rot)
        self.scanning = False

        RosProxy().subscribe_topic(
            "/%s/is_scanning" %
            (self.laser_name),
            Bool,
            self.handle_laser_update)

    def get_robot_path(self, offset=Point(0, 0, 0), rotation=np.zeros(3)):
        """Override get robot path function to be able to add
        the offset from the torch

        Arguments:
            offset {Point} -- offset vector from the current path (default: {Point(0,0,0)})
            rotation {np.array} -- additional rotation

        Returns:
            [type] -- [description]
        """

        ret = super(LaserScanTask, self).get_robot_path(
            self.offset, self.rotation)

        return ret

    def play(self, data=None):
        """Play task according to the defined data property
        sleect the move type (based on data['type'])

        Arguments:
            data {dict} -- move parameter dict
        """

        if data is not None and "type" in data and data["type"] == "ScanTask":
            ctrl = RobotControllerHandler()
            ctrl.current_controller.set_speed(
                1 if self.speed is None else self.speed)
            RosProxy().notify("Enabling scanning")
            RosProxy().call_service("/%s/start" % (self.laser_name), Empty, EmptyRequest())
            moves = self.get_robot_path()
            ctrl.current_controller.move_along(moves)

            return

        super(LaserScanTask, self).play(data)

    def handle_laser_update(self, data):
        """Handle the updates coming from the laser scanner

        Arguments:
            data {Bool} -- The scanner is on or off
        """

        self.scanning = data

    def handle_robot_step_changed(self, step):
        """Handle the robot step change

        The weld task is subscribed to the current step coming from the robot.
        If there is welding parameter changes for the current step,
        the task will request the WPS to change accordingly
        """

        if self.path is None:
            return

        if self.scanning and len(
                self.path.points) == step + 1 and step != self.step:
            RosProxy().notify("Disabling scanning")
            RosProxy().call_service("/%s/stop" % (self.laser_name), Empty, EmptyRequest())

        super(LaserScanTask, self).handle_robot_step_changed(step)

    def pause(self):
        """Stop scanning if pause called
        """

        RosProxy().call_service(
            "/%s/stop" %
            (self.laser_name),
            Empty,
            EmptyRequest())
        super(LaserScanTask, self).pause()
        RosProxy().notify("Scanning stopped")

    def input(self, data):
        """Change the offset/rotation of the current laserscan task

        Arguments:
            data {dict} -- dictionary containing the param to change and its value
        """

        super(LaserScanTask, self).input(data)

        p = data['param']
        v = data['value']

        if p == "offset":
            self.offset = Point(float(v['x']), float(v['y']), float(v['z']))
        if p == "rotation":
            self.rotation = np.array(
                [float(v['r']), float(v['p']), float(v['y'])])
