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

from std_srvs.srv import Empty, EmptyRequest
from rosweld_drivers.msg import WeldingJobs
from rosweld_drivers.srv import (SetJobNumber, SetJobNumberRequest,
                                 SetWeldingParameters,
                                 SetWeldingParametersRequest)


from src.rosweld.tasks.move import MoveTask

from src.rosweld.rosproxy import RosProxy, STATE
from src.rosweld.weldingstate import WeldingState


class WeldTask(MoveTask):

    def __setstate__(self, d):
        """Restore object from save file

        Arguments:
            d {dict} -- dictionary of the saved object
        """

        self.__dict__ = d
        RosProxy().subscribe_topic("/welding_driver/jobs",
                                   WeldingJobs, self.handle_wps_update)

    def __init__(
            self,
            task_type,
            progress,
            desc,
            title,
            step,
            current_bead_index,
            speed=1.0,
            path=None,
            use_arc_sensor=False):
        """Weld task init

        The difference from a move task is the handling of the weld parameters,
        thus, the task handles a step-by-step welding parameter update on the WPS and starts/stops
        the arc, when it is necessary

        Arguments:
            ITask {super class} -- super class
            task_type {string} -- type as string
            progress {float} -- progress
            desc {string} -- description
            title {string} -- title
            step {int} -- current step
            current_bead_index {int} -- current bead index
            path {float[6][]} -- Pose array: x, y, z, r, p, y
            use_arc_sensor {bool} -- use arc sensor or not (default: {False})
        """

        super(
            WeldTask,
            self).__init__(
                task_type,
                progress,
                desc,
                title,
                step,
                current_bead_index,
                speed,
                path)
        self.use_arc_sensor = use_arc_sensor
        RosProxy().subscribe_topic("/welding_driver/jobs",
                                   WeldingJobs, self.handle_wps_update)

        self.jobs = None

    @property
    def speed(self):
        """Getter for speed

        Returns:
            int -- current speed
        """

        if not hasattr(self, "job") or self.job is None:
            return self._speed

        return self.job.speed

    @speed.setter
    def speed(self, value):
        """Setter for speed

        Arguments:
            value {int} -- new speed
        """

        self._speed = int(value)

        if not hasattr(self, "job") or self.job is None:
            return

        data = SetWeldingParametersRequest()
        data.params.speed = self._speed
        data.params.job_number = self.job.job_number

        RosProxy().call_service("/welding_driver/set_params", SetWeldingParameters, data)

    @property
    def job(self):
        """Getter for the current job
        based on the wps_job_number property
        """

        if self.current_bead is None:
            return None

        if self.jobs is None:
            RosProxy().notify("Can not find jobs.", STATE.ERROR)
            return None

        _job = None
        for job in self.jobs.configurations:
            if job.job_number == self.current_bead.wps_job_number:
                return job

        return None

    def handle_wps_update(self, data):
        """Select current job from WPS

        Arguments:
            data {WeldingJobs} -- job list on the WPS
        """

        self.jobs = data

    def welding_state_add(self, ws1, ws2):
        """Add ws2 to ws1
        if the other object has a none None value, overwrite the
        ws1 value

        Arguments:
            ws1 {WeldingState} -- welding state 1, original
            ws2 {WeldingState} -- welding state 2, increment

        Returns:
            WeldingState -- the modified object
        """

        if ws2 is None:
            return ws1

        if ws2.amperage is not None:
            ws1.amperage += ws2.amperage

        if ws2.voltage is not None:
            ws1.voltage += ws2.voltage

        if ws2.filler_speed is not None:
            ws1.filler_speed += ws2.filler_speed

        if ws2.default_arc_length is not None:
            ws1.default_arc_length += ws2.default_arc_length

        if ws2.mode is not None:
            ws1.mode = ws2.mode

        if ws2.speed is not None:
            ws1.speed += ws2.speed

        return ws1

    @property
    def welding_parameters(self):
        """Welding parameters getter

        Gets the welding parameters for the current step
        """
        ret_params = []

        _p = copy.deepcopy(self.job)

        if _p is None:
            return [WeldingState() for i in range(len(self.path.points))]

        for i in range(len(self.path.points)):
            rm = self.current_bead.get_modification(i, False)
            pm = self.current_bead.get_modification(i, True)

            if pm is not None:
                _p = self.welding_state_add(_p, pm.welding_parameters)

            if rm is not None:
                _p = self.welding_state_add(_p, rm.welding_parameters)

            ret_params.append(copy.deepcopy(_p))

        return ret_params

    def handle_robot_step_changed(self, step):
        """Handle the robot step change

        The weld task is subscribed to the current step coming from the robot.
        If there is welding parameter changes for the current step,
        the task will request the WPS to change accordingly

        Arguments:
            step {int} -- robot step
        """

        #Save the last step if some lost
        last_known_step = self.step
        super(WeldTask, self).handle_robot_step_changed(step)

        if step < 0 or step >= len(self.welding_parameters):
            # invalid step
            return

        if self.job is None:
            # no jobs
            return

        if self.welding_parameters[step] == WeldingState():
            # default state, skip
            return

        if last_known_step > step:
            # moving to the other direction
            return

        if self.welding_parameters[last_known_step] != self.welding_parameters[step]:
            # if there is a difference, send the new params
            RosProxy().call_service(
                '/welding_driver/set_params',
                SetWeldingParameters,
                self.welding_parameters[step])

    def play(self, data=None):
        """Weld task play
        Before starting the movement the weld task will set the wps job number and start the arc

        Arguments:
            data {dict} -- weld parameter dict
        """

        if data is not None and "type" in data and data["type"] == "WeldTask":
            # only start welding if the wps job number is defined, otherwise notify and not move
            if self.current_bead.wps_job_number is not None:
                req = SetJobNumberRequest()
                req.value = self.current_bead.wps_job_number

                # set job number
                RosProxy().call_service(
                    "/welding_driver/set_job_number",
                    SetJobNumber,
                    req
                )

                # enable arc
                RosProxy().call_service("/welding_driver/arc_start", SetJobNumber,
                                        req, self.arc_set_complete)

                # send welding parameters
                RosProxy().call_service(
                    '/welding_driver/set_params',
                    SetWeldingParameters,
                    self.welding_parameters[0])
            else:
                RosProxy().notify("Missing welding job number", STATE.ERROR)
                return

        else:
            super(WeldTask, self).play(data)

    def arc_set_complete(self, *args, **kwardgs):
        """Continue the call when the arc request start is on
        """

        super(WeldTask, self).play()

    def pause(self):
        """Weld task pause

        Before stopping the movement, the weld task will stop the arc as well
        """

        RosProxy().call_service("/welding_driver/arc_stop", Empty, EmptyRequest())
        super(WeldTask, self).pause()
