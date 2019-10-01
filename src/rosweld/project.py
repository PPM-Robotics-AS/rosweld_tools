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

import json
import time

import numpy as np
import rospy
import tf
import ujson
from pydispatch import dispatcher
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from rosweld_tools.msg import Project as ProjectMsg
from rosweld_tools.srv import SetService

from src.rosweld.rosproxy import RosProxy, STATE
from src.rosweld.calibration import Calibration
from src.rosweld.savableobject import SavableObject
from src.rosweld.settings.groove import GrooveSettings
from src.rosweld.settings.project import ProjectSettings
from src.rosweld.settings.torch import TorchSettings
from src.rosweld.signals import Signals
from src.rosweld.singleton import Singleton
from src.rosweld.tasks import MoveTask, WeldTask, ContainerTask, LaserScanTask
from src.rosweld.weldingstate import WeldingState
from src.rosweld.robotcontrollerhandler import RobotControllerHandler


class Project(SavableObject):
    __metaclass__ = Singleton

    def __init__(
            self,
            state=None,
            model_torch=None,
            model_workpiece=None,
            cycle_state=None,
            cad_path=None,
            plan=None,
            available_paths=None,
            settings=None,
            beads=None,
            calibration=None):
        """Initalize a project with the given data

        Keyword Arguments:
            state {string} -- program state (default: {None})
            model_torch {settings.torch} -- Torch config (default: {None})
            model_workpiece {string} -- workpiece location (default: {None})
            cycle_state {string} -- cycle state (default: {None})
            cad_path {string} -- cad path (default: {None})
            plan {ContainerTask} -- root task of the plan (default: {None})
            available_paths {Path[]} -- available path list (default: {None})
            settings {settings.project} -- project default settings (default: {None})
            beads {Bead[]} -- beads (default: {None})
            calibration {Calibraiton} -- calibration for the model-world transf. (default: {None})
        """

        super(Project, self).__init__("project")

        self.__state = state
        self.model_torch = model_torch
        self.model_workpiece = model_workpiece
        self.cycle_state = cycle_state
        self.cad_path = cad_path
        self.plan = plan
        self.available_paths = available_paths
        self.path_markers = None

        if settings is None:
            settings = ProjectSettings(
                WeldingState(),
                TorchSettings.default(),
                GrooveSettings.default())

        self.settings = settings
        self.beads = beads
        self.calibration = calibration

        if self.calibration is None:
            self.calibration = Calibration()

        RosProxy().advertise_service("project_control", SetService, self.__project_control)

        self.publish_plan()

        # subscribe to step change
        dispatcher.connect(
            self.handle_robot_step_changed,
            signal=Signals['STEP_CHANGED'],
            sender=dispatcher.Any
        )

    def handle_robot_step_changed(self, step):
        """Sets the step on the current task in the project

        Arguments:
            step {int} -- current step
        """

        if self.plan is None or self.plan.current_task.step == step:
            return

        self.plan.current_task.input({'param': 'robot_update', 'value': step})
        self.publish_plan()

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, s):
        dispatcher.send(
            signal=Signals['PROJECT_STATE_CHANGED'], sender=self, state=s
        )
        self.__state = s

    def publish_plan(self):
        """Publish the plan to ROS in every sec
        """

        if self.path_markers is None:
            self.path_markers = MarkerArray()

        for marker in self.path_markers.markers:
            marker.action = Marker.DELETE

        if self.plan is not None and hasattr(
                self.plan.current_task, "model_path"):
            rospy.Duration(2)
            mp = self.plan.current_task.model_path
            for idx, p in enumerate(mp):
                marker = Marker()
                start = p.position
                qori = np.array([
                    p.orientation.x,
                    p.orientation.y,
                    p.orientation.z,
                    p.orientation.w])

                cp = np.array([p.position.x, p.position.y, p.position.z, 0.])
                z = np.array([0., 0., 0.03, 0.])
                ep = np.add(np.dot(tf.transformations.quaternion_matrix(qori), z), cp)
                end = Point()
                end.x = ep[0]
                end.y = ep[1]
                end.z = ep[2]

                marker.points.append(start)
                marker.points.append(end)
                marker.id = int(str(int(round(time.time() * 1000)) + idx)[6:])
                marker.header.frame_id = Calibration.marker_workpiece_frame
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.scale.x = 0.005
                marker.scale.y = 0.01
                marker.color.a = 1.0
                marker.color = ColorRGBA(
                    a=1, r=1, g=0, b=0) if idx == 0 else ColorRGBA(
                        a=1, r=0, g=1, b=0)
                self.path_markers.markers.append(marker)

        # Publish path markers
        RosProxy().publish_topic(
            "current_path", MarkerArray, self.path_markers, latch=True)

        # Keep only the added markers
        self.path_markers.markers = [
            marker for marker in self.path_markers.markers if marker.action == Marker.ADD]

        # Create project message
        proj_msg = ProjectMsg(
            project_json=String(
                data=ujson.dumps(
                    self.plan)), default_settings=String(
                        data=ujson.dumps(
                            self.settings)))

        # Publish project message
        RosProxy().publish_topic(
            "project", ProjectMsg, proj_msg, latch=True)

    def reset_state(self):
        pass

    def set_state(self):
        pass

    def remove_path(self):
        pass

    def load_object(self, obj):
        """Load a project

        Arguments:
            obj {Project} -- the project to laod
        """

        self.plan = obj.plan
        self.settings = obj.settings
        self.publish_plan()

    def load_default(self):
        """Load default plan to the project
        """

        self.plan = Project.default_plan()

        self.publish_plan()

    @staticmethod
    def default_plan():
        """Returns a minimal plan with one weld task

        Returns:
            ContainerTask -- default plan
        """

        weld_task = WeldTask("WeldTask", 0, "Default weld task",
                             "Def. weld", 0, None, 1.0)
        scan_task = LaserScanTask("ScanTask", 0, "Default scan task",
                                  "Def. scan", 0, None, 1.0)
        plan = ContainerTask(
            "ContainerTask", 0, "Default plan container", "Def. container", [
                weld_task, scan_task], 0)

        return plan

    def __project_control(self, req): # pragma: no cover
        """Handle project control service calls

        Arguments:
            req {rosweld_tools.srv.SetService} -- request

        Returns:
            bool -- the request is handled or not
        """

        cmd = req.command

        if cmd == "play":
            data = json.loads(req.json)
            self.plan.play(data)
        elif cmd == "publish":
            self.publish_plan()
            return True
        elif cmd == "pause":
            self.plan.pause()
        elif cmd == "save":
            self.save()
        elif cmd == "cancel":
            self.load()
        elif cmd == "input":
            data = json.loads(req.json)
            self.plan.input(data)
        elif cmd == "update":
            RosProxy().publish_last()
            RobotControllerHandler().update()
        elif cmd == "plan":
            data = json.loads(req.json)
            action = data["action"]

            if action == "select":
                self.plan.set_current_task(int(data["index"]))

            elif action == "add":
                cont_id = data["container_id"]
                container = self.plan.get_task(cont_id)

                if container is not None:
                    task_type = data["type"]
                    if task_type == "move":
                        container.tasks.append(
                            MoveTask(
                                task_type,
                                0,
                                task_type,
                                task_type,
                                0,
                                1.0))
                    if task_type == "weld":
                        container.tasks.append(
                            WeldTask(
                                task_type,
                                0,
                                task_type,
                                task_type,
                                0,
                                1.0,
                                None))
                    if task_type == "scan":
                        container.tasks.append(LaserScanTask(
                            task_type, 0, task_type, task_type, 0, 1.0, None))

            elif action == "delete":
                container = self.plan.get_task(data["container_id"])
                task = container.get_task(data["task_id"])
                container.tasks.remove(task)
        else:
            RosProxy().notify("Invalid input", STATE.ERROR)
            return False

        self.publish_plan()

        return True
