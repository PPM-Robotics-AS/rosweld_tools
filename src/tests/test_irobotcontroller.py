import rospy
import mock
from src.rosweld.robotcontrollerhandler import RobotController, RobotControllerHandler
from src.rosweld.rosproxy import RosProxy
from rosweld_tools.srv import SetServiceRequest
from rosweld_drivers.msg import RobotState, Move
from rosweld_drivers.srv import MoveAlongRequest, MoveBetweenRequest, MoveBetweenRequest, SetSpeedRequest

class TestRobotController(object):
    def test_init(self):

        with mock.patch.object(RosProxy, 'advertise_service', return_value=None) as mock_adv_service, \
            mock.patch.object(RosProxy, 'publish_topic', return_value=None) as mock_pub_topic, \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, 'subscribe_topic', return_value=None) as mock_sub_topic, \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None):
        
            irc = RobotControllerHandler()

            assert irc._controllers['simulation'] is not None
            assert irc._controllers['robot'] is not None

    def test_controller_change(self):
        with mock.patch.object(RosProxy, 'advertise_service', return_value=None) as mock_adv_service, \
            mock.patch.object(RosProxy, 'publish_topic', return_value=None) as mock_pub_topic, \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, 'subscribe_topic', return_value=None) as mock_sub_topic, \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None):
        
            irc = RobotControllerHandler()
            irc._set_simulation(SetServiceRequest(command = "true"))
            assert irc.current_controller == irc._controllers['simulation']['controller']
            irc._set_simulation(SetServiceRequest(command = "false"))
            assert irc.current_controller == irc._controllers['robot']['controller']

    def test_commands(self):
        with mock.patch.object(RosProxy, 'advertise_service', return_value=None) as mock_adv_service, \
            mock.patch.object(RosProxy, 'publish_topic', return_value=None) as mock_pub_topic, \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, 'call_service', return_value=None) as mock_call_service, \
            mock.patch.object(RosProxy, 'subscribe_topic', return_value=None) as mock_sub_topic, \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None):

            moves = []
            moves.append(Move())
            moves.append(Move())
            moves.append(Move())

            irc = RobotControllerHandler()
            irc.current_controller.move_along(moves)
            call_count = 1
            assert mock_call_service.call_count == call_count
            assert "move_along" in mock_call_service.call_args[0][0] 
            assert mock_call_service.call_args[0][2] == moves

            irc.current_controller.store_poses(moves)
            call_count = call_count + 1
            assert mock_call_service.call_count == call_count
            assert "store_poses" in mock_call_service.call_args[0][0] 
            assert mock_call_service.call_args[0][2] == moves

            irc.current_controller.move_pose(moves)
            call_count = call_count + 1
            assert mock_call_service.call_count == call_count
            assert "move_pose" in mock_call_service.call_args[0][0] 
            assert mock_call_service.call_args[0][2] == moves

            irc.current_controller.move_between(0,0,None)
            call_count = call_count + 1
            assert mock_call_service.call_count == call_count
            assert "move_between" in mock_call_service.call_args[0][0] 
            assert mock_call_service.call_args[0][2] == MoveBetweenRequest(start = 0, end = 0)

            irc.current_controller.abort()
            call_count = call_count + 1
            assert mock_call_service.call_count == call_count
            assert "abort" in mock_call_service.call_args[0][0] 

            irc.current_controller.set_speed(1)
            call_count = call_count + 1
            assert mock_call_service.call_count == call_count
            assert "set_speed" in mock_call_service.call_args[0][0]
            assert mock_call_service.call_args[0][2] == SetSpeedRequest(value = 1)




    def test_handle_robot(self):
        with mock.patch.object(RosProxy, 'advertise_service', return_value=None) as mock_adv_service, \
            mock.patch.object(RosProxy, 'publish_topic', return_value=None) as mock_pub_topic, \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, 'subscribe_topic', return_value=None) as mock_sub_topic, \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None):

            irc = RobotControllerHandler()
            irc._set_simulation(SetServiceRequest(command = "true"))
            sc = irc._controllers['simulation']['controller']
            rc = irc._controllers['robot']['controller']

            state1 = RobotState(step = 1)
            state2 = RobotState(step = 2)

            assert state1 != state2
            
            sc.set_current_state(state1)
            assert irc.current_state == state1
            assert sc.current_state == state1

            rc.set_current_state(state2)
            assert rc.current_state == state2
            assert irc.current_state == state1 #the global state is the same

            irc._set_simulation(SetServiceRequest(command = "false"))
            assert irc.current_state == state2




