from ..rosweld.bead import Bead
from ..rosweld.point import Point
from ..rosweld.modification import Modification
from ..rosweld.weldingstate import WeldingState

class TestBead(object):
    def test_init(self):
        b1 = Bead(None, {}, {})
        b2 = Bead(b1, {}, {})

        assert b2.parent == b1
        assert len(b1.runtime_modifications) == 0 and len(b1.planned_modifications) == 0
        assert b1.angle == 0
        assert b1.offset == Point(0., 0., 0.)

    def test_remove_modification(self):
        b1 = Bead(None, {1: Modification(Point(), 0, 0, None)}, {2: Modification(Point(), 0, 0, None)})
        b1.remove_modification(2, True)
        b1.remove_modification(1, False)

        assert len(b1.planned_modifications) == 0
        assert len(b1.runtime_modifications) == 0

    def test_get_modification(self):
        b1 = Bead(None, {1: Modification(Point(), 0, 0, None)}, {2: Modification(Point(), 0, 0, None)})
        pm1 = b1.get_modification(1, True)
        pm2 = b1.get_modification(2, True)
        rm = b1.get_modification(1, False)

        assert pm1 is None and pm2 is not None and rm is not None
        assert rm.offset == Point() and rm.angle == 0
        assert pm2.offset == rm.offset

    def test_is_zero_modification(self):
        rm = Modification()
        assert Modification.is_zero(rm) == True

        rm.delta_r = 1.
        assert Modification.is_zero(rm) == False

        rm = Modification()
        assert Modification.is_zero(rm) == True

        ws = WeldingState()
        ws.amperage = 111.
        rm.welding_parameters = ws
        assert Modification.is_zero(rm) == False

        rm = Modification()
        assert Modification.is_zero(rm) == True

        rm.angle = 1.
        assert Modification.is_zero(rm) == False

        rm = Modification()
        assert Modification.is_zero(rm) == True

        rm.offset = Point(1., 0., 0.)
        assert Modification.is_zero(rm) == False


