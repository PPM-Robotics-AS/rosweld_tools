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

import copy
import json
import time

import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rosweld_drivers.msg import Move
from rosweld_tools.msg import Calibration as CalibrationMsg
from rosweld_tools.srv import SetService

from src.rosweld.calibrationpoint import CalibrationPoint
from src.rosweld.robotcontrollerhandler import RobotControllerHandler
from src.rosweld.point import Point
from src.rosweld.rosproxy import RosProxy, STATE
from src.rosweld.savableobject import SavableObject
from src.rosweld.transformation import Transformation

SAFETY_DISTANCE = 0.02

class Calibration(SavableObject):
    """Calibration class.
    Stores the calibration points on the model and pair them with a
    robot coordinate.

    Publishes the calibration points as MarkerArray to visualize is.

    Recalculates the transformation in case of change in the config.
    """


    marker_workpiece_frame = "plate"
    marker_size = 0.015

    def __init__(self, poses=None, selected_point=None):
        """Initialize a new Calibration

        Arguments:
            poses {CalibrationPoint[]} -- calibration points

        Keyword Arguments:
            selected_point {int} -- selected point index (default: {0})
        """
        super(Calibration, self).__init__("calibration")

        self.marker_array = MarkerArray()
        if selected_point is None:
            selected_point = 0

        if poses is not None:
            self.poses = poses
        else:
            self.load()

        self.selected_point = None if len(self.poses) == 0 else selected_point
        self.calibration_changed()
        RosProxy().advertise_service("set_calibration", SetService, self.set_calibration)

    @property
    def transformation_matrix(self):
        """Transformation matrix property

        Returns:
            numpy.nparray -- transformation matrix
        """

        return Transformation.get_transformation_matrix(self.poses)

    def calibration_changed(self):
        """Calibration changed event, publish the markers and the calibration data to ROS
        """

        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE

        for idx, pose in enumerate(self.poses):
            marker = Marker()
            marker.id = int(str(int(round(time.time() * 1000)) + idx)[6:])
            marker.header.frame_id = Calibration.marker_workpiece_frame
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = Calibration.marker_size
            marker.scale.y = Calibration.marker_size
            marker.scale.z = Calibration.marker_size
            marker.color.a = 1.0
            marker.color = ColorRGBA(a=1, r=1, g=0, b=0) if\
                idx == self.selected_point else ColorRGBA(a=1, r=1, g=1, b=0)
            marker.pose.orientation.w = 1.0
            marker.pose.position = pose.model
            self.marker_array.markers.append(marker)

        RosProxy().publish_topic(
            "calibration_markers", MarkerArray, self.marker_array, latch=True)

        # Keep only the added markers
        self.marker_array.markers = [
            marker for marker in self.marker_array.markers if marker.action == Marker.ADD]

        calibration_msg = CalibrationMsg(
            points=self.poses,
            selected_point=-
            1 if self.selected_point is None else self.selected_point,
            transformation=np.append([], self.transformation_matrix))

        RosProxy().publish_topic(
            "calibration_points", CalibrationMsg, calibration_msg, latch=True)

        RobotControllerHandler().set_transformation(self.transformation_matrix)

    def add(self, p):
        """Add new model point to the calibration points

        Arguments:
            p {Point} -- new calibration point
        """

        self.poses.append(CalibrationPoint(p))
        self.selected_point = len(self.poses) - 1
        self.calibration_changed()

    def edit(self, p):
        """Edit the seleted model point

        Arguments:
            p {Point} -- The new positions
        """
        self.poses[self.selected_point].model = p
        self.calibration_changed()

    def record(self):
        """Record the real world position of the selected model point
        """

        if self.selected_point is None:
            RosProxy().notify("No calibration point selected", STATE.ERROR)
            return

        if len(self.poses) == 0:
            RosProxy().notify("No calibration point added", STATE.ERROR)
            return

        if RobotControllerHandler.current_state is None:
            RosProxy().notify("The current robot state is not available", STATE.ERROR)
            return

        robot_pose = RobotControllerHandler.current_state.pose

        # pos = robot_pose.position
        # orientation = robot_pose.orientation
        # axes = 'sxyz'
        # (rx, ry, rz) = tf.transformations.euler_from_quaternion(np.array(
        #     [orientation.x, orientation.y, orientation.z, orientation.w]), axes)

        self.poses[self.selected_point].measured = robot_pose
        self.calibration_changed()

        # logdebug(
        #     "Recording pose (x,y,z,r,p,y): %d,%d,%d,%d,%d,%d" %
        #     (pos.x *
        #      1000,
        #      pos.y *
        #      1000,
        #      pos.z *
        #      1000,
        #      math.degrees(rx),
        #      math.degrees(ry),
        #      math.degrees(rz)))

    def remove(self):
        """Remove selected calibration point
        """

        if self.selected_point is None:
            RosProxy().notify("No calibration point selected", STATE.ERROR)
            return

        if len(self.poses) == 0:
            RosProxy().notify("No calibration point added", STATE.ERROR)
            return

        self.poses.remove(self.poses[self.selected_point])

        if len(self.poses) == 0:
            self.selected_point = None
        else:
            self.selected_point = min(len(self.poses) - 1, self.selected_point)

        self.calibration_changed()

    def load_object(self, obj):
        """Load a given object

        Arguments:
            obj {object} -- object to load
        """

        self.poses = obj.poses
        self.selected_point = obj.selected_point
        self.calibration_changed()

    def load_default(self):
        """Load a default calibration as an example
        """

        poses = []

        cp = CalibrationPoint(Pose(), Pose())
        cp.measured.position = Point(0.0, 0.0, 0.12)
        cp.measured.orientation.x = -0.025
        cp.measured.orientation.y = 1.
        cp.measured.orientation.z = 0.011
        cp.measured.orientation.w = 0.002
        cp.model = Point(0., 0., 0.)
        poses.append(copy.deepcopy(cp))

        cp.measured.position = Point(0.25, 0., 0.12)
        cp.model = Point(0.25, 0., 0.)
        cp.measured.orientation.x = -0.025
        cp.measured.orientation.y = 1.
        cp.measured.orientation.z = 0.011
        cp.measured.orientation.w = 0.002
        poses.append(copy.deepcopy(cp))

        cp.measured.position = Point(0.25, 0.14, 0.12)
        cp.model = Point(0.25, 0.14, 0.)
        cp.measured.orientation.x = -0.025
        cp.measured.orientation.y = 1.
        cp.measured.orientation.z = 0.011
        cp.measured.orientation.w = 0.002
        poses.append(copy.deepcopy(cp))

        cp.measured.position = Point(0., 0.12, 0.12)
        cp.model = Point(0., 0.12, 0.)
        cp.measured.orientation.x = -0.025
        cp.measured.orientation.y = 1.
        cp.measured.orientation.z = 0.011
        cp.measured.orientation.w = 0.002
        poses.append(copy.deepcopy(cp))

        self.poses = poses
        self.set_selected_point(0)

    def goto(self, speed=1):
        """Goto to the selected position

        Keyword Arguments:
            speed {int} -- move speed (default: {1})
        """

        self.safe_goto(speed, 0)

    def safe_goto(self, speed=1.0, distance=SAFETY_DISTANCE):
        """Safe goto to the selected position (the Z is increased with 2cm)

        Keyword Arguments:
            speed {float} -- move speed (default: {1.0})
            distance {float} -- safety distance (default: {SAFETY_DISTANCE})
        """

        moves = []
        point = copy.deepcopy(
            self.poses[self.selected_point].measured.position)
        orientation = self.poses[self.selected_point].measured.orientation
        point.z += distance
        move = Move()
        move.pose.position = point
        move.pose.orientation = orientation
        move.speed = speed
        moves.append(move)

        RobotControllerHandler().current_controller.set_speed(speed)
        RobotControllerHandler().current_controller.move_pose(moves)

    def set_selected_point(self, i):
        """Set the selected point

        Arguments:
            i {int} -- index of the selected point
        """

        if i < len(self.poses):
            self.selected_point = min(len(self.poses), max(0, i))
            self.calibration_changed()

    def set_calibration(self, req):
        """Calibration setter service handler

        Arguments:
            req {rosweld_tools.srv.SetService} -- ROS Service object

        Returns:
            bool -- state of the service result
        """

        cmd = req.command
        data = json.loads(req.json)
        if cmd == "add":
            if "x" in data and "y" in data and "z" in data:
                x, y, z = float(data["x"]), float(data["y"]), float(data["z"])
                self.add(Point(x, y, z))
            else:
                return False
        elif cmd == "edit":
            if "x" in data and "y" in data and "z" in data:
                x, y, z = float(data["x"]), float(data["y"]), float(data["z"])
                self.edit(Point(x, y, z))
            else:
                return False
        elif cmd == "goto":
            self.goto(speed=float(data["speed"]) if "speed" in data else 1.0)
        elif cmd == "save":
            self.save()
        elif cmd == "cancel":
            self.load()
        elif cmd == "safe_goto":
            self.safe_goto(
                speed=float(
                    data["speed"]) if "speed" in data else 1.0, distance=float(
                        data["distance"]) if "distance" in data else SAFETY_DISTANCE)
        elif cmd == "record":
            self.record()
        elif cmd == "remove":
            self.remove()
        elif cmd == "set_selected_point" and "index" in data:
            self.set_selected_point(int(data["index"]))
        else:
            RosProxy().notify("Invalid calibration input", STATE.ERROR)
            return False

        return True
