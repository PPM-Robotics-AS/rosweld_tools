from pydispatch import dispatcher
from mock import Mock, patch

import rospy

from rosweld_drivers.msg import RobotState, WeldingState
from std_msgs.msg import Int16
from rosweld_tools.srv import SetServiceRequest

from ..rosweld.signals import Signals
from ..rosweld.project import Project
from ..rosweld.rosproxy import RosProxy
from ..rosweld.savableobject import SavableObject
from ..rosweld.calibration import Calibration
from ..rosweld.path import Path
from ..rosweld.bead import Bead
from ..rosweld.point import Point
from ..rosweld.tasks.weld import WeldTask
from ..rosweld.settings.torch import TorchSettings
from ..rosweld.settings.groove import GrooveSettings
from ..rosweld.settings.project import ProjectSettings

class MockLoadProject(object):
    def __init__(self, plan, settings):
        self.plan = plan
        self.settings = settings

class TestProject(object):
    
    def test_init(self):
        RosProxy.clear()

        with patch.object(RosProxy, 'advertise_service', return_value=None) as mock_adv_service, \
            patch.object(RosProxy, 'notify', return_value=None), \
            patch.object(rospy, 'init_node', return_value=None), \
            patch.object(Calibration, '__init__', return_value=None) as mock_init_calibration, \
            patch.object(Project, 'publish_plan', return_value=None) as mock_publish_plan:

            project = Project()

            assert mock_adv_service.called
            assert mock_publish_plan.called
            assert mock_init_calibration.called

            assert project.state == None
            assert project.model_torch == None
            assert project.model_workpiece == None
            assert project.cycle_state == None
            assert project.cad_path == None
            assert project.plan == None
            assert project.available_paths == None
            assert project.beads == None
            assert project.calibration != None
            assert project.settings != None

            Project.clear()
            c = Calibration()
            project = Project(calibration=c)
            assert project.calibration == c

            Project.clear()
            #try to clear the signleton when it is cleared
            try:
                Project.clear()
                assert True
            except:
                return

    def test_publish_plan(self):
        Project.clear()
        RosProxy.clear()

        with patch.object(RosProxy, 'subscribe_topic', return_value=None), \
            patch.object(RosProxy, 'advertise_service', return_value=None), \
            patch.object(RosProxy, 'notify', return_value=None), \
            patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(rospy, 'init_node', return_value=None), \
            patch.object(Calibration, '__init__', return_value=None) as mock_init_calibration:

            project = Project(plan = Project.default_plan(), calibration=Calibration())

            assert mock_publish.called

            #add a sample path to the current task to test the markers
            project.plan.current_task.path = Path([[1,0,0,1,0,0],[2,0,0,1,0,0],[3,0,0,1,0,0]], 0, None, [ Bead(None, {}, {}) ])
            project.plan.current_task.current_bead_index = 0
            #test to publish markers
            project.publish_plan()
            #test to remove markers with 2nd publish
            project.publish_plan()


    def test_robot_state_update(self):
        Project.clear()

        with patch.object(RosProxy, 'subscribe_topic', return_value=None), \
            patch.object(RosProxy, 'notify', return_value=None), \
            patch.object(RosProxy, 'advertise_service', return_value=None), \
            patch.object(rospy, 'init_node', return_value=None), \
            patch.object(WeldTask, 'job') as mock_job,\
            patch.object(Calibration, '__init__', return_value=None) as mock_init_calibration, \
            patch.object(Project, 'publish_plan', return_value=None) as mock_publish_plan:

            project = Project()

            project.load_default()
            mock_job.__get__ = Mock(return_value=WeldingState())

            dispatcher.send(
                signal=Signals['STEP_CHANGED'], sender=self, step=1
            )
            assert project.plan.current_task.step == 1

            dispatcher.send(
                signal=Signals['STEP_CHANGED'], sender=self, step=1
            )
            assert project.plan.current_task.step == 1

            dispatcher.send(
                signal=Signals['STEP_CHANGED'], sender=self, step=2
            )
            assert project.plan.current_task.step == 2

    def test_reset_state(self):
        Project.clear()

        with patch.object(RosProxy, 'subscribe_topic', return_value=None), \
            patch.object(RosProxy, 'notify', return_value=None), \
            patch.object(RosProxy, 'advertise_service', return_value=None), \
            patch.object(rospy, 'init_node', return_value=None), \
            patch.object(Calibration, '__init__', return_value=None), \
            patch.object(Project, 'publish_plan', return_value=None):

            project = Project()

            # Not implemented function, accept
            project.reset_state()

            assert True

    def test_set_state(self):
        Project.clear()

        with patch.object(RosProxy, 'subscribe_topic', return_value=None), \
            patch.object(RosProxy, 'notify', return_value=None), \
            patch.object(RosProxy, 'advertise_service', return_value=None), \
            patch.object(rospy, 'init_node', return_value=None), \
            patch.object(Calibration, '__init__', return_value=None), \
            patch.object(Project, 'publish_plan', return_value=None):

            project = Project()

            # Not implemented function, accept
            project.set_state()

            assert True

    def test_remove_path(self):
        Project.clear()

        with patch.object(RosProxy, 'subscribe_topic', return_value=None), \
            patch.object(RosProxy, 'notify', return_value=None), \
            patch.object(RosProxy, 'advertise_service', return_value=None), \
            patch.object(rospy, 'init_node', return_value=None), \
            patch.object(Calibration, '__init__', return_value=None), \
            patch.object(Project, 'publish_plan', return_value=None):

            project = Project()

            project.remove_path()

            assert True

    def test_load_object(self):
        Project.clear()

        with patch.object(RosProxy, 'subscribe_topic', return_value=None), \
            patch.object(RosProxy, 'notify', return_value=None), \
            patch.object(RosProxy, 'advertise_service', return_value=None), \
            patch.object(rospy, 'init_node', return_value=None), \
            patch.object(Calibration, '__init__', return_value=None), \
            patch.object(Project, 'publish_plan', return_value=None):

            project = Project()

            def_plan = Project.default_plan()
            settings = ProjectSettings(
                WeldingState(),
                TorchSettings.default(),
                GrooveSettings.default())

            project.load_object(MockLoadProject(plan=def_plan, settings=settings))

            assert project.plan == def_plan
            assert project.settings == settings

