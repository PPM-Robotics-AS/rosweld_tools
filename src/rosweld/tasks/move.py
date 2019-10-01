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
from rosweld_drivers.msg import Move

from src.rosweld.tasks.itask import ITask

from src.rosweld.bead import Bead
from src.rosweld.camparser import CamParser
from src.rosweld.robotcontrollerhandler import RobotControllerHandler
from src.rosweld.modification import Modification
from src.rosweld.path import Path
from src.rosweld.point import Point
from src.rosweld.transformation import Transformation
from src.rosweld.weldingstate import WeldingState
from src.rosweld.rosproxy import RosProxy, STATE

#define the minimum speed
MINSPEED = 0.01


class MoveTask(ITask):
    """Move task: the task moves on a specified path, including
    real time and plannet modifications
    """

    def __init__(
            self,
            task_type,
            progress,
            desc,
            title,
            step,
            current_bead_index,
            speed=1.0,
            path=None):
        """Move task init

        Arguments:
            ITask {super class} -- super class
            task_type {string} -- type as string
            progress {float} -- progress
            desc {string} -- description
            title {string} -- title
            step {int} -- current step
            speed {int} -- default speed (default: {0})
            current_bead_index {int} -- current bead index with modifications
            path {Path} -- path (default: {None})
        """

        super(MoveTask, self).__init__(task_type, progress, desc, title)
        self.step = step

        if path is None:
            path = Path([], 0, [], [])

        self.path = path
        self._current_bead_index = current_bead_index
        self._speed = speed

    @property
    def current_bead_index(self):
        """Getter for the current bead index

        Returns:
            int -- current bead index
        """

        return self._current_bead_index

    @current_bead_index.setter
    def current_bead_index(self, value):
        """Setter for the current bead index

        Arguments:
            value {int} -- current bead index to set
        """

        self._current_bead_index = min(
            max(value, 0), len(self.path.beads) - 1)

        if self.current_bead_index != value:
            RosProxy().notify("The bead index was invalid.", STATE.ERROR)

    @property
    def speed(self):
        """Getter for speed

        Returns:
            int -- current speed
        """

        return max(MINSPEED, self._speed)

    @speed.setter
    def speed(self, value):
        """Setter for speed

        Arguments:
            value {int} -- new speed
        """

        self._speed = max(MINSPEED, value)

    def handle_robot_step_changed(self, step):
        """Handle the robot step change

        Arguments:
            step {int} -- step to set
        """

        self.step = step

    @property
    def current_bead(self):
        """Getter for the current bead
        based on the self.current_bead_index parameter
        
        Returns:
            Bead -- current bead, None if the index is invalid
        """

        if len(self.path.beads) > self.current_bead_index and \
            len(self.path.beads) > 0:
            return self.path.beads[self.current_bead_index]

        return None

    def move_to_step(self, data):
        """Set the current step in the move

        Arguments:
            data {dict} -- dictionary, containing additional params for the step change
        """

        step = int(data['step'])
        RobotControllerHandler().current_controller.move_between(self.step, step)

    def play(self, data=None):
        """Play task according to the defined data property
        sleect the move type (based on data['type'])

        Arguments:
            data {dict} -- move parameter dict
        """

        moves = []
        ctrl = RobotControllerHandler()
        ctrl.current_controller.set_speed(
            1 if self.speed is None else self.speed)

        if data is None or "type" not in data or data["type"] == "verify":
            moves = self.get_robot_path()
        elif data["type"] == "travel":
            moves = self.get_robot_path()
            ctrl.current_controller.store_poses(moves)
            ctrl.current_controller.move_pose(moves[0:1:1])
            return
        elif data["type"] == "store":
            moves = self.get_robot_path()
            ctrl.current_controller.store_poses(moves)
            return
        elif data['type'] == "continue":
            poses_count = 0 if ctrl.current_state is None else ctrl.current_state.storedPoses
            ctrl.current_controller.move_between(self.step, poses_count - 1)
            return
        else:
            RosProxy().notify("Unkown move type", STATE.ERROR)

        ctrl.current_controller.move_along(moves)

    def pause(self):
        """Pause task
        """
        RobotControllerHandler().current_controller.abort()

    def reset(self):
        """Reset the current task and go back to the 1st step
        """

        self.pause()
        self.move_to_step({'step': 0})

    @property
    def model_path(self):
        """Get model path

        Returns:
            Pose[] -- Pose list
        """

        path = Transformation.apply_path(
            self.path.points,
            np.identity(4))

        return self.current_bead.apply_modifications_to_path(path, self.path.angle)

    def get_robot_path(self, offset=Point(0, 0, 0), rotation=np.zeros(3)):
        """Convert the current path to robot path
        including the modifications (plan and runtime)

        Arguments:
            offset {Point} -- offset vector from the current path (default: {Point(0,0,0)})
            rotation {np.array} -- additional rotation

        Returns:
            Move[] -- Robot moves
        """

        path = Transformation.apply_path(
            self.path.points,
            RobotControllerHandler().current_transformation)

        moves = []
        for pose in \
            self.current_bead.apply_modifications_to_path(path, self.path.angle, offset, rotation):
            move = Move()
            move.pose = pose
            move.speed = self.speed
            moves.append(move)

        return moves

    def input(self, data):
        """Receive input and act accordingly

        Arguments:
            data {dict} -- param and value pairs

        Returns:
            bool -- input handled or not
        """

        p = data['param']
        v = data['value']

        if p == 'path':
            self.path = Path(CamParser.convert(
                v), 0, None, [Bead(None, {}, {})])
            self.current_bead_index = 0
        elif p == 'robot_update':
            self.handle_robot_step_changed(v)
        elif p == 'select_bead':
            self.current_bead_index = int(v)

            RobotControllerHandler().current_controller.store_poses(self.get_robot_path())
            self.move_to_step({'step': 0})
        elif p == 'add_bead':
            self.path.beads.append(Bead(None, {}, {}))
            self.current_bead_index = len(self.path.beads) - 1

            RobotControllerHandler().current_controller.store_poses(self.get_robot_path())
            self.move_to_step({'step': 0})
        elif p == 'remove_bead':
            del self.path.beads[self.current_bead_index]
        elif p == 'step':
            self.move_to_step(v)
        elif p == 'modification':
            self.handle_modification_change(v)
        elif p == 'speed':
            self.speed = float(v)
            RobotControllerHandler().current_controller.set_speed(self.speed)
        elif p == 'setup_bead':
            self.current_bead.wps_job_number = None if "job_number" not in v or \
                v['job_number'] is None else int(v['job_number'])
            self.current_bead.offset.z = float(v['z'])
            self.current_bead.offset.y = float(v['y'])
            self.current_bead.angle = float(v['angle'])

            RobotControllerHandler().current_controller.store_poses(self.get_robot_path())
            self.move_to_step({'step': self.step})
        else:
            RosProxy().notify("Invalid input")
            return False

        return True

    def handle_modification_change(self, mod):
        """Handle modification change input

        Arguments:
            mod {dict} -- modification
        """

        current_mod = self.current_bead.get_modification(self.step, True)
        if current_mod is None:
            current_mod = Modification()
            self.current_bead.add_modification(self.step, True, current_mod)

        current_mod.offset.z = float(mod['z'])
        current_mod.offset.y = float(mod['y'])
        current_mod.angle = float(mod['angle'])
        current_mod.delta_r = float(mod['delta_r'])

        if current_mod.welding_parameters is None:
            current_mod.welding_parameters = WeldingState()

        current_mod.welding_parameters.amperage = float(mod['amperage'])
        current_mod.welding_parameters.voltage = float(mod['voltage'])
        current_mod.welding_parameters.filler_speed = float(
            mod['filler_speed'])

        if Modification.is_zero(current_mod):
            self.current_bead.remove_modification(self.step, True)

        RobotControllerHandler().current_controller.store_poses(self.get_robot_path())
        self.move_to_step({'step': self.step})
