import mock
from rosweld_drivers.msg import RobotState
from rosweld_tools.srv import SetServiceRequest

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from src.rosweld.calibration import Calibration
from src.rosweld.calibrationpoint import CalibrationPoint
from src.rosweld.rosproxy import RosProxy
from src.rosweld.robotcontrollerhandler import RobotControllerHandler, RobotController
from src.rosweld.savableobject import SavableObject
from src.rosweld.transformation import Transformation

class DummyPublisher(object):

    def __init__(self, *args, **kwargs):
        pass

    def publish(self, *args, **kwargs):
        assert True

def void(cls, *args, **kwargs):
    return

class DummyCalibration(object):

    def __init__(self):
        self.poses = [CalibrationPoint([]), CalibrationPoint([]), CalibrationPoint([]), CalibrationPoint([])]
        self.selected_point = 1

def publish_topic(cls, *args, **kwargs):
    return DummyPublisher(*args, **kwargs)

test_setter_data = [
    { 'req': SetServiceRequest( command = "add", json = '{"x":1,"y":1,"z":1}' ), 'func': "add", 'handled': True},
    { 'req': SetServiceRequest( command = "add", json = '{"y":1,"z":1}' ), 'func': "add", 'handled': False },
    { 'req': SetServiceRequest( command = "edit", json = '{"x":1,"y":1,"z":1}'), 'func': "edit", 'handled': True },
    { 'req': SetServiceRequest( command = "edit", json = '{"y":1,"z":1}' ), 'func': "edit", 'handled': False },
    { 'req': SetServiceRequest( command = "goto", json = '{"speed": 10}' ), 'func': "goto", 'handled': True },
    { 'req': SetServiceRequest( command = "goto", json = "{}" ), 'func': "goto", 'handled': True },
    { 'req': SetServiceRequest( command = "save", json = "{}" ), 'func': "save", 'handled': True },
    { 'req': SetServiceRequest( command = "safe_goto", json = '{"speed": 10}' ), 'func': "safe_goto", 'handled': True },
    { 'req': SetServiceRequest( command = "safe_goto", json = "{}" ), 'func': "safe_goto", 'handled': True },
    { 'req': SetServiceRequest( command = "safe_goto", json = '{"distance": 10}' ), 'func': "safe_goto", 'handled': True },
    { 'req': SetServiceRequest( command = "safe_goto", json = '{"distance": 10, "speed": 10}' ), 'func': "safe_goto", 'handled': True },
    { 'req': SetServiceRequest( command = "cancel", json = "{}" ), 'func': "load", 'handled': True },
    { 'req': SetServiceRequest( command = "remove", json = "{}" ), 'func': "remove", 'handled': True },
    { 'req': SetServiceRequest( command = "record", json = "{}" ), 'func': "record", 'handled': True },
    { 'req': SetServiceRequest( command = "set_selected_point", json = '{"index": 1}' ), 'func': "set_selected_point", 'handled': True },
    { 'req': SetServiceRequest( command = "set_selected_point", json = "{}" ), 'func': "set_selected_point", 'handled': False },
    { 'req': SetServiceRequest( command = "not_exist", json = "{}" ), 'func': None, 'handled': True },
    ]

def mock_load(self):
    self.poses = []
    assert True

class TestCalibration(object):
    def test_init(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(SavableObject, 'load', mock_load) as mock_load_default, \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'transformation_matrix', void):

            c = Calibration()
            assert c.poses == []

        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'transformation_matrix', void):

            c = Calibration([])
            assert len(c.poses) == 0
            assert c.selected_point == None

            c = Calibration([CalibrationPoint(Point(), Pose())])
            assert len(c.poses) == 1
            assert c.selected_point == 0

            p = [CalibrationPoint(Point(), Pose()),CalibrationPoint(Point(), Pose()),CalibrationPoint(Point(), Pose())]
            selp = 2
            c = Calibration(p, selp)
            assert len(c.poses) == len(p)
            assert c.selected_point == selp
    
    def test_add(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'transformation_matrix', void):

            c = Calibration([CalibrationPoint(Point(), Pose())])
            assert len(c.poses) == 1
            assert c.selected_point == 0
            c.add(Point(x=1, y=1, z=1))
            assert len(c.poses) == 2
            assert c.selected_point == 1

    def test_set_calibration(self):
        global test_setter_data

        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'add', return_value=None) as mock_add, \
            mock.patch.object(Calibration, 'edit', return_value=None) as mock_edit, \
            mock.patch.object(Calibration, 'goto', return_value=None) as mock_goto, \
            mock.patch.object(Calibration, 'safe_goto', return_value=None) as mock_safe_goto, \
            mock.patch.object(Calibration, 'save', return_value=None) as mock_save, \
            mock.patch.object(Calibration, 'load', return_value=None) as mock_load, \
            mock.patch.object(Calibration, 'record', return_value=None) as mock_record, \
            mock.patch.object(Calibration, 'set_selected_point', return_value=None) as mock_set_selected_point, \
            mock.patch.object(Calibration, 'calibration_changed', return_value=None) as mock_calibration_changed, \
            mock.patch.object(Calibration, 'remove', return_value=None) as mock_remove:

            c = Calibration([])
            
            for data in test_setter_data:
                mock_obj = None

                if data['func'] is not None:
                    mock_obj = locals()['mock_' + data['func']]
                    call_count = mock_obj.call_count

                ret = c.set_calibration(data['req'])

                if mock_obj is not None:
                    if data['handled']:
                        assert mock_obj.call_count == call_count + 1
                    else:
                        assert mock_obj.call_count == call_count
                else:
                    assert ret == False


    def test_remove(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(Calibration, 'transformation_matrix', void), \
            mock.patch.object(RobotControllerHandler, '__init__', void), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'calibration_changed', return_value=None) as mock_calibration_changed, \
            mock.patch.object(RobotControllerHandler, 'current_state', RobotState()):

            c = Calibration([])
            c.selected_point = None
            c.remove()
            # only called after init, but not after record
            assert mock_calibration_changed.call_count == 1

            c = Calibration([CalibrationPoint(Point(), Pose()), CalibrationPoint(Point(), Pose())], 0)
            original_len = len(c.poses)
            c.selected_point = 0
            c.remove()
            assert len(c.poses) == original_len - 1 
            assert mock_calibration_changed.called

            c.remove()
            assert len(c.poses) == 0
            assert c.selected_point == None
            assert mock_calibration_changed.called

    def test_load_object(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'calibration_changed', return_value=None) as mock_calibration_changed, \
            mock.patch.object(Calibration, 'transformation_matrix', void):

            c = Calibration([])
            assert len(c.poses) == 0
            assert c.selected_point == None
            calibration_changed_count = mock_calibration_changed.call_count 

            c.load_object(DummyCalibration())
            assert len(c.poses) == 4
            assert c.selected_point == 1
            assert mock_calibration_changed.call_count == calibration_changed_count + 1

            c.load_default()
            assert len(c.poses) == 4
            assert c.selected_point == 0
            assert mock_calibration_changed.call_count == calibration_changed_count + 2

    def test_record(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(Calibration, 'transformation_matrix', void), \
            mock.patch.object(RobotControllerHandler, '__init__', void), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'calibration_changed', return_value=None) as mock_calibration_changed, \
            mock.patch.object(RobotControllerHandler, 'current_state', RobotState()):

            c = Calibration([])
            c.selected_point = None
            c.record()
            # only called after init, but not after record
            assert mock_calibration_changed.call_count == 1


            c = Calibration([CalibrationPoint(Point(), Pose()), CalibrationPoint(Point(), Pose())], 0)
            p = RobotControllerHandler.current_state.pose
            c.selected_point = 0
            c.record()
            assert c.poses[0].measured == p
            assert mock_calibration_changed.called

    def test_set_selected_point(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'transformation_matrix', void):

            c = Calibration([CalibrationPoint(Point(), Pose()), CalibrationPoint(Point(), Pose())], 0)
            c.set_selected_point(1)
            assert c.selected_point == 1
            assert c.poses[c.selected_point] == c.poses[1]

    def test_edit(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Calibration, 'transformation_matrix', void):

            c = Calibration([CalibrationPoint(Point(), Pose()), CalibrationPoint(Point(), Pose())], 0)
            c.edit(Point(1., 1., 1.))
            assert c.poses[c.selected_point].model.x == 1. and \
                c.poses[c.selected_point].model.y == 1. and \
                c.poses[c.selected_point].model.z == 1. 

    def test_properties(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(Transformation, 'get_transformation_matrix', return_value=None) as mock_trafo_getter:

            c = Calibration([CalibrationPoint(Point(), Pose()), CalibrationPoint(Point(), Pose())], 0)
            trafo = c.transformation_matrix
            assert mock_trafo_getter.called

    def test_goto(self):
        with mock.patch.object(RosProxy, 'advertise_service', void), \
            mock.patch.object(RosProxy, 'publish_topic', publish_topic), \
            mock.patch.object(RosProxy, 'notify', return_value=None), \
            mock.patch.object(RosProxy, '__init__', return_value=None), \
            mock.patch.object(Calibration, 'transformation_matrix', void), \
            mock.patch.object(Calibration, 'calibration_changed', return_value=None) as mock_calibration_changed, \
            mock.patch.object(RobotControllerHandler, '__init__', void), \
            mock.patch.object(RobotControllerHandler, 'current_state', RobotState()),\
            mock.patch.object(RobotControllerHandler, 'current_controller') as mock_current_controller,\
            mock.patch.object(RobotController, '__init__', return_value=None) as mock_rc_init,\
            mock.patch.object(RobotController, 'set_speed', return_value=None) as mock_set_speed,\
            mock.patch.object(RobotController, 'check_alive', return_value=None), \
            mock.patch.object(RobotController, 'move_pose', return_value=None) as mock_move: 

            mock_current_controller.__get__ = mock.Mock(return_value=RobotController("dummy"))
            c = Calibration([CalibrationPoint(Point(), Pose())], 0)
            c.goto()

            assert mock_set_speed.called
            assert mock_set_speed.call_args[0][0] == 1
            
            c.goto(speed = 10)
            assert mock_set_speed.call_args[0][0] == 10

            assert mock_set_speed.called
            assert mock_set_speed.call_count == 2
            assert mock_move.call_count == 2
            assert mock_move.call_args[0][0][0].pose == Pose()

            c.safe_goto(distance=1.0)
            exp_position = Point()
            exp_position.z += 1.0
            assert mock_move.call_args[0][0][0].pose == Pose( position = exp_position )
    