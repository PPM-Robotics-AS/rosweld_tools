from mock import Mock, patch
import rospy
from rosweld_drivers.msg import Move

from ..rosweld.tasks.move import MoveTask
from ..rosweld.tasks.container import ContainerTask 
from ..rosweld.project import Project
from ..rosweld.camparser import CamParser
from ..rosweld.rosproxy import RosProxy
from ..rosweld.robotcontrollerhandler import RobotController, RobotControllerHandler

def mock_service(self, path, type, value, handler):
    assert True

def mock_robot_path(self, speed = 1):
    moves = []

    for i in range(len(self.path.points)):
        moves.append(Move())

    return moves

def mock_get_param(cls, p):
    return p

def get_plan():
    with patch.object(rospy, 'get_param', mock_get_param):
        return Project.default_plan()

class TestTasks(object):


    def test_pause(self):
        plan = get_plan()

        with patch.object(RosProxy, 'call_service', return_value=None), \
             patch.object(RosProxy, 'notify', return_value=None), \
             patch.object(MoveTask, 'get_robot_path', return_value=[]), \
             patch.object(MoveTask, 'pause', return_value=None) as mock_pause:

            plan.pause()
            assert mock_pause.called

    def test_input(self):
        plan = get_plan()

        with patch.object(CamParser, 'convert', return_value=None) as mock_convert:
            plan.input({'param': 'path', 'value':''})
            assert mock_convert.called

        with patch.object(RosProxy, 'call_service', return_value=None) as mock_call, \
             patch.object(RosProxy, 'notify', return_value=None), \
             patch.object(MoveTask, 'get_robot_path', return_value=[]), \
             patch.object(MoveTask, 'handle_modification_change', return_value=None) as mock_handler:
            
            plan.input({'param': 'step', 'value': { 'step': 5 } })
            assert mock_call.called

            plan.input({'param': 'modification', 'value': ''})
            assert mock_handler.called

        with patch.object(RosProxy, 'call_service', return_value=None) as mock_call, \
             patch.object(RosProxy, 'notify', return_value=None), \
             patch.object(MoveTask, 'get_robot_path', return_value=[]):

            plan.input({'param': 'setup_bead', 'value': {'z':1, 'y':1, 'angle':1}})
            assert plan.current_task.current_bead.angle == 1
            assert plan.current_task.current_bead.offset.z == 1
            assert plan.current_task.current_bead.offset.y == 1

            plan.input({'param': 'step', 'value': { 'step': 0 }})
            plan.input({'param': 'modification', 'value': {
                'z': 1,
                'y': 1,
                'angle': 1,
                'delta_r': 1,
                'amperage': 1,
                'voltage': 1,
                'filler_speed': 1
            }})
            mod = plan.current_task.current_bead.get_modification(0, True)
            assert mod is not None
            assert mod.angle == 1

    def test_play(self):
        plan = get_plan()

        with patch.object(RosProxy, 'call_service', return_value=None), \
             patch.object(RosProxy, 'notify', return_value=None), \
             patch.object(MoveTask, 'get_robot_path', return_value=[]), \
             patch.object(MoveTask, 'play', return_value=None) as mock_play:

            plan.play()
            assert mock_play.called

        with patch.object(RosProxy, 'call_service', autospec=True) as mock_call_service, \
             patch.object(RosProxy, 'notify', return_value=None), \
             patch.object(MoveTask, 'get_robot_path', autospec=True) as mock_get_robot_path, \
             patch.object(RobotController, 'check_alive', return_value=None), \
             patch.object(RobotController, 'set_speed', autospec=True) as mock_set_speed:

            path = "415.500	80.000	-10.000	0.000000	-0.707107	0.707107\r\n\
                    392.289	80.000	-10.000	0.000000	-0.707107	0.707107"

            mock_get_robot_path.side_effect = mock_robot_path
            plan.input({'param': 'path', 'value': path})
            assert len(plan.current_task.path.points) == 2

            plan.input({'param': 'step', 'value': { 'step': 1 } })
            mock_call_service.side_effect = mock_service
            plan.play({'type': 'continue'})

